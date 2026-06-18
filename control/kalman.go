package control

import (
	"math"
)

type EKFConfig struct {
	AdaptationRate            float64
	Regularization            float64
	ChiSquareThreshold        float64
	PrometheusSaturationLimit float64
	MinSensorNoise            float64
	MinProcessNoise           float64
}

func DefaultEKFConfig() EKFConfig {
	return EKFConfig{
		AdaptationRate:            0.05,
		Regularization:            1e-6,
		ChiSquareThreshold:        15.086,
		PrometheusSaturationLimit: 100000.0,
		MinSensorNoise:            1e-3,
		MinProcessNoise:           1e-4,
	}
}

type ExtendedKalmanFilter struct {
	X StateVector   
	P [9][9]float64 
	Q [9][9]float64 
	R [5][5]float64 
	CyRolling [5][5]float64 
	Cfg EKFConfig
}

func NewExtendedKalmanFilter(cfg EKFConfig) *ExtendedKalmanFilter {
	ekf := &ExtendedKalmanFilter{Cfg: cfg}
	for i := 0; i < 9; i++ { ekf.P[i][i] = 1.0; ekf.Q[i][i] = cfg.MinProcessNoise }
	for i := 0; i < 5; i++ { ekf.R[i][i] = 0.05; ekf.CyRolling[i][i] = 0.10 }
	ekf.P[8][8] = 0.01 
	return ekf
}

func (ekf *ExtendedKalmanFilter) ResetBlank() {
	for i := 0; i < 9; i++ {
		ekf.X[i] = 0.0
		for j := 0; j < 9; j++ { ekf.P[i][j] = 0.0 }
		ekf.P[i][i] = 1.0
	}
}

func (ekf *ExtendedKalmanFilter) Predict(u ControlVector, initial SystemState, simCfg SimConfig, dt float64) {
	xPrior := ekf.X
	stateNominal := vectorToDynamicState(xPrior)
	StateTransition(&stateNominal, u, initial, simCfg, 0.0)
	ekf.X = stateNominal.ToVector()

	var F [9][9]float64
	for i := 0; i < 9; i++ { F[i][i] = 1.0 }

	Q_prev, R_prev, C_prev, A_prev, S_prev := xPrior[0], xPrior[1], xPrior[2], xPrior[3], xPrior[8]

	F[3][3] = 1.0 - (simCfg.ArrivalTheta * dt)
	F[4][2] = -(simCfg.NaturalFrequency * simCfg.NaturalFrequency) * dt
	F[4][4] = 1.0 - (2.0 * simCfg.DampingRatio * simCfg.NaturalFrequency * dt)
	F[2][4], F[5][0] = dt, dt

	retryAmp := 1.0 + simCfg.RetryFeedbackGain*math.Log(1.0+R_prev)
	cacheGain := math.Exp(-u[2])
	queueLimit := math.Max(u[1], 1.0)
	
	admission := 1.0 / (1.0 + (Q_prev / queueLimit))
	dAdmission_dQ := -1.0 / (queueLimit * math.Pow(1.0+(Q_prev/queueLimit), 2))
	effectiveArrival := A_prev * retryAmp * cacheGain * admission

	F[0][3] = (retryAmp * cacheGain * admission) * dt              
	F[0][0] += (A_prev * retryAmp * cacheGain * dAdmission_dQ) * dt 
	
	dRetryAmp_dR := simCfg.RetryFeedbackGain / (1.0 + R_prev)
	F[0][1] = (A_prev * cacheGain * admission * dRetryAmp_dR) * dt  

	util := effectiveArrival / math.Max(C_prev, 0.001)
	if util > 1.0 { F[0][2] = -1.0 * dt } else { F[0][2] = -1.0 * dt }

	F[1][0] = (simCfg.RetryAlpha / queueLimit) * dt
	F[1][1] = 1.0 - (simCfg.RetryBeta * dt)
	
	currentL := ComputeLatency(Q_prev, effectiveArrival, C_prev, simCfg.BaseLatency, simCfg.MaxQueueDelay)
	if currentL > initial.SLATarget {
		F[6][0] = (1.0 / math.Max(effectiveArrival, 0.001)) * dt 
		F[2][1] = (simCfg.RetryGamma / math.Max(initial.SLATarget, 0.001)) * dt
	}

	F[7][0] = (A_prev * retryAmp * cacheGain * (-dAdmission_dQ)) * dt

	qOverload := math.Max(0.0, (Q_prev/queueLimit)-2.0)
	hazard := (qOverload * qOverload) + math.Max(0.0, R_prev-10.0)
	F[8][8] = math.Exp(-hazard * dt)
	if qOverload > 0.0 { F[8][0] = -2.0 * qOverload * (1.0 / queueLimit) * S_prev * dt }
	if R_prev > 10.0 { F[8][1] = -1.0 * S_prev * dt }

	var FP [9][9]float64
	for i := 0; i < 9; i++ {
		for j := 0; j < 9; j++ {
			sum := 0.0; for k := 0; k < 9; k++ { sum += F[i][k] * ekf.P[k][j] }; FP[i][j] = sum
		}
	}

	var newP [9][9]float64
	for i := 0; i < 9; i++ {
		for j := 0; j < 9; j++ {
			sum := 0.0; for k := 0; k < 9; k++ { sum += FP[i][k] * F[j][k] }; newP[i][j] = sum + ekf.Q[i][j]
		}
	}
	ekf.P = newP

	for i := 0; i < 9; i++ {
		if math.IsNaN(ekf.X[i]) || math.IsInf(ekf.X[i], 0) { ekf.ResetBlank(); return }
	}
}

func (ekf *ExtendedKalmanFilter) Update(z MeasurementVector) {
	// ========================================================================
	// DEFENSE IN DEPTH: BYZANTINE FAULT REJECTION
	// If a corrupted payload bypasses the PolicyController shield, the EKF
	// will completely ignore it and smoothly "Fly on Instruments" using its prediction.
	// ========================================================================
	for i := 0; i < 5; i++ {
		if math.IsNaN(z[i]) || math.IsInf(z[i], 0) { return }
	}

	currentDynamicState := vectorToDynamicState(ekf.X)
	hX := ObservationModel(currentDynamicState, ekf.Cfg)

	var y [5]float64
	for i := 0; i < 5; i++ { y[i] = z[i] - hX[i] }

	latencyError := math.Abs(y[1])
	queueError := math.Abs(y[0])
	
	if latencyError > 2.0 || queueError > 2000.0 {
		ekf.X[0] = math.Max(0.0, z[0]) 
		ekf.X[1] = math.Max(0.0, z[2]) 
		ekf.X[2] = math.Max(0.001, z[3]) 
		ekf.X[3] = math.Max(0.001, z[4]) 
		
		for i := 0; i < 9; i++ {
			ekf.Q[i][i] *= 100.0 
			for j := 0; j < 9; j++ { ekf.P[i][j] = 0.0 }
			ekf.P[i][i] = 100.0  
		}
		return 
	}

	alpha := ekf.Cfg.AdaptationRate
	for i := 0; i < 5; i++ {
		for j := 0; j < 5; j++ { ekf.CyRolling[i][j] = (1.0-alpha)*ekf.CyRolling[i][j] + alpha*(y[i]*y[j]) }
	}

	var H [5][9]float64
	H[0][0], H[2][1], H[3][2], H[4][3] = 1.0, 1.0, 1.0, 1.0

	if ekf.X[0] <= ekf.Cfg.PrometheusSaturationLimit {
		effectiveArrival := ekf.X[3] 
		if effectiveArrival > 0.01 {
			H[1][0] = 1.0 / effectiveArrival          
			H[1][3] = -ekf.X[0] / (effectiveArrival * effectiveArrival) 
		}
	}

	var HP [5][9]float64
	for i := 0; i < 5; i++ {
		for j := 0; j < 9; j++ {
			sum := 0.0; for k := 0; k < 9; k++ { sum += H[i][k] * ekf.P[k][j] }; HP[i][j] = sum
		}
	}

	var S [5][5]float64
	for i := 0; i < 5; i++ {
		for j := 0; j < 5; j++ {
			sum := 0.0; for k := 0; k < 9; k++ { sum += HP[i][k] * H[j][k] }
			S[i][j] = sum + ekf.R[i][j]; if i == j { S[i][j] += ekf.Cfg.Regularization }
		}
	}

	SInv, ok := choleskyInverse55(S)
	if !ok { return }

	var SInvY [5]float64
	for i := 0; i < 5; i++ {
		sum := 0.0; for j := 0; j < 5; j++ { sum += SInv[i][j] * y[j] }; SInvY[i] = sum
	}
	
	nis := 0.0; for i := 0; i < 5; i++ { nis += y[i] * SInvY[i] }
	
	scaleFactor := 1.0
	if nis > ekf.Cfg.ChiSquareThreshold { scaleFactor = math.Sqrt(ekf.Cfg.ChiSquareThreshold / nis) }

	for i := 0; i < 5; i++ {
		hPh := 0.0; for k := 0; k < 9; k++ { hPh += HP[i][k] * H[i][k] }
		ekf.R[i][i] = math.Max(ekf.Cfg.MinSensorNoise, ekf.CyRolling[i][i]-hPh)
	}

	var PHT [9][5]float64
	for i := 0; i < 9; i++ {
		for j := 0; j < 5; j++ {
			sum := 0.0; for k := 0; k < 9; k++ { sum += ekf.P[i][k] * H[j][k] }; PHT[i][j] = sum
		}
	}

	var K [9][5]float64
	for i := 0; i < 9; i++ {
		for j := 0; j < 5; j++ {
			sum := 0.0; for k := 0; k < 5; k++ { sum += PHT[i][k] * SInv[k][j] }
			K[i][j] = sum * scaleFactor
		}
	}

	var stateCorrection [9]float64
	for i := 0; i < 9; i++ {
		sum := 0.0; for j := 0; j < 5; j++ { sum += K[i][j] * y[j] }
		stateCorrection[i] = sum
		ekf.X[i] += sum
	}

	for i := 0; i < 9; i++ {
		if math.IsNaN(ekf.X[i]) || math.IsInf(ekf.X[i], 0) { ekf.ResetBlank(); return }
	}

	ekf.X[0] = math.Max(0.0, ekf.X[0])
	ekf.X[1] = math.Max(0.0, ekf.X[1])
	ekf.X[2] = math.Max(0.001, ekf.X[2])

	for i := 0; i < 9; i++ { ekf.Q[i][i] = (1.0-alpha)*ekf.Q[i][i] + alpha*math.Max(ekf.Cfg.MinProcessNoise, stateCorrection[i]*stateCorrection[i]) }

	var I_KH [9][9]float64
	for i := 0; i < 9; i++ {
		I_KH[i][i] = 1.0
		for j := 0; j < 9; j++ { sum := 0.0; for k := 0; k < 5; k++ { sum += K[i][k] * H[k][j] }; I_KH[i][j] -= sum }
	}

	var I_KH_P [9][9]float64
	for i := 0; i < 9; i++ {
		for j := 0; j < 9; j++ { sum := 0.0; for k := 0; k < 9; k++ { sum += I_KH[i][k] * ekf.P[k][j] }; I_KH_P[i][j] = sum }
	}

	var Term1 [9][9]float64
	for i := 0; i < 9; i++ {
		for j := 0; j < 9; j++ { sum := 0.0; for k := 0; k < 9; k++ { sum += I_KH_P[i][k] * I_KH[j][k] }; Term1[i][j] = sum }
	}

	var KR [9][5]float64
	for i := 0; i < 9; i++ {
		for j := 0; j < 5; j++ { sum := 0.0; for k := 0; k < 5; k++ { sum += K[i][k] * ekf.R[k][j] }; KR[i][j] = sum }
	}
	
	var Term2 [9][9]float64
	for i := 0; i < 9; i++ {
		for j := 0; j < 9; j++ { sum := 0.0; for k := 0; k < 5; k++ { sum += KR[i][k] * K[j][k] }; Term2[i][j] = sum }
	}

	for i := 0; i < 9; i++ {
		for j := 0; j < 9; j++ { ekf.P[i][j] = Term1[i][j] + Term2[i][j] }
	}
}

func ObservationModel(s DynamicState, cfg EKFConfig) MeasurementVector {
	qObs := s.Queue
	if qObs > cfg.PrometheusSaturationLimit { qObs = cfg.PrometheusSaturationLimit }
	effectiveArrival := s.Arrival * (1.0 + 0.2*math.Log(1.0+s.Retry)) 
	
	simulatedLatency := ComputeLatency(s.Queue, effectiveArrival, s.Capacity, 0.005, 100.0)
	return MeasurementVector{qObs, simulatedLatency, s.Retry, s.Capacity, s.Arrival}
}

func vectorToDynamicState(v StateVector) DynamicState { var s DynamicState; s.FromVector(v); return s }
func dynamicStateToVector(s DynamicState) StateVector { return s.ToVector() }

func choleskyInverse55(S [5][5]float64) ([5][5]float64, bool) {
	var L [5][5]float64
	for i := 0; i < 5; i++ {
		for j := 0; j <= i; j++ {
			sum := 0.0
			for k := 0; k < j; k++ { sum += L[i][k] * L[j][k] }
			if i == j {
				val := S[i][i] - sum
				if val <= 1e-11 { return [5][5]float64{}, false }
				L[i][j] = math.Sqrt(val)
			} else {
				if math.Abs(L[j][j]) < 1e-11 { return [5][5]float64{}, false }
				L[i][j] = (S[i][j] - sum) / L[j][j]
			}
		}
	}
	var Linv [5][5]float64
	for i := 0; i < 5; i++ {
		Linv[i][i] = 1.0 / L[i][i]
		for j := 0; j < i; j++ {
			sum := 0.0
			for k := j; k < i; k++ { sum += L[i][k] * Linv[k][j] }
			Linv[i][j] = -sum / L[i][i]
		}
	}
	var SInv [5][5]float64
	for i := 0; i < 5; i++ {
		for j := 0; j < 5; j++ {
			sum := 0.0
			start := i; if j > i { start = j }
			for k := start; k < 5; k++ { sum += Linv[k][i] * Linv[k][j] }
			SInv[i][j] = sum
		}
	}
	return SInv, true
}