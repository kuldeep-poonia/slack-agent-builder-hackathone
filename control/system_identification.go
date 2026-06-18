package control

import (
	"math"
)


// SYSTEM IDENTIFICATION LAYER (V4 - Production Grade)
// Upgrades: NIS-driven VFF, Trace-based Confidence, Full Physics Identification


type SysIdConfig struct {
	LearningRate  float64
	LambdaMin     float64 // Minimum forgetting factor (max learning speed)
	LambdaMax     float64 // Maximum forgetting factor (max memory)
	MaxTrace      float64 // Covariance explosion limit (Regularization)
	MinExcitation float64 // Deadband to prevent fitting to noise
	NISDecay      float64 // Sensitivity of lambda to NIS spikes
}

func DefaultSysIdConfig() SysIdConfig {
	return SysIdConfig{
		LearningRate:  0.02,
		LambdaMin:     0.85,
		LambdaMax:     0.999,
		MaxTrace:      5000.0,
		MinExcitation: 1e-4,
		NISDecay:      0.1, // Controls how fast lambda drops when NIS spikes
	}
}


// 1-Parameter Recursive Least Squares (For Feedback Gains)


type RLS1 struct {
	Theta    float64
	P        float64
	Cfg      SysIdConfig
	NoiseVar float64
}

func NewRLS1(cfg SysIdConfig) *RLS1 {
	return &RLS1{Cfg: cfg, P: 100.0, NoiseVar: 1.0}
}

func (r *RLS1) Update(phi float64, y float64) {
	if math.Abs(phi) < r.Cfg.MinExcitation {
		return
	}

	yhat := r.Theta * phi
	err := y - yhat

	// Update estimated noise variance (EWMA)
	r.NoiseVar = (1-r.Cfg.LearningRate)*r.NoiseVar + r.Cfg.LearningRate*(err*err)

	Pphi := r.P * phi
	phiPphi := phi * Pphi

	// Normalized Innovation Squared (NIS)
	S := r.NoiseVar + phiPphi
	NIS := (err * err) / math.Max(1e-6, S)

	// Exponentially map NIS to Lambda (Fixes Problem 1)
	lambda := r.Cfg.LambdaMin + (r.Cfg.LambdaMax-r.Cfg.LambdaMin)*math.Exp(-r.Cfg.NISDecay*NIS)

	denom := lambda + phiPphi
	if denom <= 0 {
		return
	}

	K := Pphi / denom
	r.Theta += K * err

	r.P = (r.P - K*phi*r.P) / lambda

	if r.P > r.Cfg.MaxTrace {
		r.P = r.Cfg.MaxTrace
	}
}


// 2-Parameter RLS (For Arrival Process)


type RLS2 struct {
	Theta    [2]float64
	P        [2][2]float64
	Cfg      SysIdConfig
	NoiseVar float64
}

func NewRLS2(cfg SysIdConfig) *RLS2 {
	r := &RLS2{Cfg: cfg, NoiseVar: 1.0}
	r.P[0][0] = 100.0
	r.P[1][1] = 100.0
	return r
}

func (r *RLS2) Update(phi [2]float64, y float64) {
	excitation := math.Abs(phi[0]) + math.Abs(phi[1])
	if excitation < r.Cfg.MinExcitation {
		return
	}

	yhat := r.Theta[0]*phi[0] + r.Theta[1]*phi[1]
	err := y - yhat

	r.NoiseVar = (1-r.Cfg.LearningRate)*r.NoiseVar + r.Cfg.LearningRate*(err*err)

	var Pphi [2]float64
	Pphi[0] = r.P[0][0]*phi[0] + r.P[0][1]*phi[1]
	Pphi[1] = r.P[1][0]*phi[0] + r.P[1][1]*phi[1]

	phiPphi := phi[0]*Pphi[0] + phi[1]*Pphi[1]

	S := r.NoiseVar + phiPphi
	NIS := (err * err) / math.Max(1e-6, S)

	lambda := r.Cfg.LambdaMin + (r.Cfg.LambdaMax-r.Cfg.LambdaMin)*math.Exp(-r.Cfg.NISDecay*NIS)

	denom := lambda + phiPphi
	if denom <= 0 {
		return
	}

	var K [2]float64
	K[0] = Pphi[0] / denom
	K[1] = Pphi[1] / denom

	r.Theta[0] += K[0] * err
	r.Theta[1] += K[1] * err

	var newP [2][2]float64
	newP[0][0] = (r.P[0][0] - K[0]*phi[0]*r.P[0][0] - K[0]*phi[1]*r.P[1][0]) / lambda
	newP[0][1] = (r.P[0][1] - K[0]*phi[0]*r.P[0][1] - K[0]*phi[1]*r.P[1][1]) / lambda
	newP[1][0] = (r.P[1][0] - K[1]*phi[0]*r.P[0][0] - K[1]*phi[1]*r.P[1][0]) / lambda
	newP[1][1] = (r.P[1][1] - K[1]*phi[0]*r.P[0][1] - K[1]*phi[1]*r.P[1][1]) / lambda

	trace := newP[0][0] + newP[1][1]
	if trace > r.Cfg.MaxTrace {
		scale := r.Cfg.MaxTrace / trace
		newP[0][0] *= scale
		newP[0][1] *= scale
		newP[1][0] *= scale
		newP[1][1] *= scale
	}

	r.P = newP
}


// 3-Parameter RLS (For Retry Process)


type RLS3 struct {
	Theta    [3]float64
	P        [3][3]float64
	Cfg      SysIdConfig
	NoiseVar float64
}

func NewRLS3(cfg SysIdConfig) *RLS3 {
	r := &RLS3{Cfg: cfg, NoiseVar: 1.0}
	r.P[0][0] = 100.0
	r.P[1][1] = 100.0
	r.P[2][2] = 100.0
	return r
}

func (r *RLS3) Update(phi [3]float64, y float64) {
	excitation := math.Abs(phi[0]) + math.Abs(phi[1]) + math.Abs(phi[2])
	if excitation < r.Cfg.MinExcitation {
		return
	}

	yhat := r.Theta[0]*phi[0] + r.Theta[1]*phi[1] + r.Theta[2]*phi[2]
	err := y - yhat

	r.NoiseVar = (1-r.Cfg.LearningRate)*r.NoiseVar + r.Cfg.LearningRate*(err*err)

	var Pphi [3]float64
	for i := 0; i < 3; i++ {
		Pphi[i] = r.P[i][0]*phi[0] + r.P[i][1]*phi[1] + r.P[i][2]*phi[2]
	}

	phiPphi := phi[0]*Pphi[0] + phi[1]*Pphi[1] + phi[2]*Pphi[2]

	S := r.NoiseVar + phiPphi
	NIS := (err * err) / math.Max(1e-6, S)

	lambda := r.Cfg.LambdaMin + (r.Cfg.LambdaMax-r.Cfg.LambdaMin)*math.Exp(-r.Cfg.NISDecay*NIS)

	denom := lambda + phiPphi
	if denom <= 0 {
		return
	}

	var K [3]float64
	for i := 0; i < 3; i++ {
		K[i] = Pphi[i] / denom
	}

	for i := 0; i < 3; i++ {
		r.Theta[i] += K[i] * err
	}

	var newP [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			sum := K[i]*phi[0]*r.P[0][j] + K[i]*phi[1]*r.P[1][j] + K[i]*phi[2]*r.P[2][j]
			newP[i][j] = (r.P[i][j] - sum) / lambda
		}
	}

	trace := newP[0][0] + newP[1][1] + newP[2][2]
	if trace > r.Cfg.MaxTrace {
		scale := r.Cfg.MaxTrace / trace
		for i := 0; i < 3; i++ {
			for j := 0; j < 3; j++ {
				newP[i][j] *= scale
			}
		}
	}

	r.P = newP
}


// DATA STRUCTURES


type ParameterEstimate struct {
	ArrivalTheta      float64
	ArrivalMean       float64
	ArrivalSigma      float64
	RetryAlpha        float64
	RetryBeta         float64
	RetryGamma        float64
	EfficiencyDecay   float64 // Learnt
	RetryFeedbackGain float64 // Learnt
}

type ParameterConfidence struct {
	ArrivalProcess    float64
	RetryProcess      float64
	ContentionProcess float64
	Amplification     float64
}

type Observation struct {
	Queue    float64
	Latency  float64
	Retry    float64
	Arrival  float64
	Capacity float64
	Time     float64
}


// SYSTEM IDENTIFIER WIRING


type SystemIdentifier struct {
	Last     *Observation
	Estimate ParameterEstimate

	ArrivalRLS *RLS2
	RetryRLS   *RLS3
	
	// Fix Problem 5: New Adapters
	ContentionRLS    *RLS1 
	AmplificationRLS *RLS1

	arrivalVariance float64 // Fix Problem 3
	Cfg             SysIdConfig
}

func NewSystemIdentifier() *SystemIdentifier {
	cfg := DefaultSysIdConfig()
	return &SystemIdentifier{
		Cfg:              cfg,
		ArrivalRLS:       NewRLS2(cfg),
		RetryRLS:         NewRLS3(cfg),
		ContentionRLS:    NewRLS1(cfg),
		AmplificationRLS: NewRLS1(cfg),
		arrivalVariance:  0.01,
		Estimate: ParameterEstimate{
			ArrivalTheta:      0.25,
			ArrivalMean:       100.0,
			ArrivalSigma:      0.10,
			RetryAlpha:        0.30,
			RetryBeta:         0.25,
			RetryGamma:        0.50,
			EfficiencyDecay:   0.15,
			RetryFeedbackGain: 0.20,
		},
	}
}

func (sid *SystemIdentifier) Update(current Observation) {
	if sid.Last == nil {
		sid.Last = &current
		return
	}

	prev := *sid.Last
	dt := current.Time - prev.Time

	if dt <= 0 {
		sid.Last = &current
		return
	}

	// ====================================
	// 1. ARRIVAL RLS
	// dA/dt = b0*A + b1 => dA/dt = -Theta*A + Theta*Mu
	// ====================================
	arrivalRate := (current.Arrival - prev.Arrival) / dt
	arrivalPhi := [2]float64{prev.Arrival, 1.0}

	sid.ArrivalRLS.Update(arrivalPhi, arrivalRate)

	// Fix Problem 2: Preserving physical sign boundary
	b0 := sid.ArrivalRLS.Theta[0]
	b1 := sid.ArrivalRLS.Theta[1]
	
	// System must mean-revert. If b0 > 0, system is exploding. Clamp to stable regime.
	theta := math.Max(1e-6, -b0) 
	mu := b1 / theta

	sid.Estimate.ArrivalTheta = theta
	sid.Estimate.ArrivalMean = mu

	// Fix Problem 3: Correct Sigma calculation
	predictedArrivalRate := b0*prev.Arrival + b1
	arrivalResidual := arrivalRate - predictedArrivalRate

	sid.arrivalVariance = (1-sid.Cfg.LearningRate)*sid.arrivalVariance + 
		sid.Cfg.LearningRate*(arrivalResidual*arrivalResidual)
	
	sid.Estimate.ArrivalSigma = math.Sqrt(sid.arrivalVariance)

	// ====================================
	// 2. RETRY RLS
	// dR/dt = Alpha*Q + Gamma*L - Beta*R
	// ====================================
	retryRate := (current.Retry - prev.Retry) / dt
	retryPhi := [3]float64{current.Queue, current.Latency, -current.Retry}

	sid.RetryRLS.Update(retryPhi, retryRate)

	if sid.RetryRLS.Theta[0] > 0 {
		sid.Estimate.RetryAlpha = sid.RetryRLS.Theta[0]
	}
	if sid.RetryRLS.Theta[1] > 0 {
		sid.Estimate.RetryGamma = sid.RetryRLS.Theta[1]
	}
	if sid.RetryRLS.Theta[2] > 0 {
		sid.Estimate.RetryBeta = sid.RetryRLS.Theta[2]
	}

	// ====================================
	// 3. EFFICIENCY DECAY RLS (Problem 5)
	// Contention model: Service = Capacity / (1 + Decay * max(0, Util - 1))
	// => Decay = (Capacity/Service - 1) / max(0.001, Util - 1)
	// ====================================
	
	// Approximate Service Rate from mass balance: Service = Arrival - dQueue/dt
	dQueue := (current.Queue - prev.Queue) / dt
	approxService := math.Max(0.001, current.Arrival - dQueue)
	utilization := current.Arrival / math.Max(0.001, current.Capacity)

	if utilization > 1.05 { // Only learn when system is actually under contention
		yContention := (current.Capacity / approxService) - 1.0
		phiContention := utilization - 1.0
		sid.ContentionRLS.Update(phiContention, yContention)
		
		if sid.ContentionRLS.Theta > 0 {
			sid.Estimate.EfficiencyDecay = sid.ContentionRLS.Theta
		}
	}

	// ====================================
	// 4. RETRY FEEDBACK GAIN RLS (Problem 5)
	// Amplification = 1 + Gain * log(1+Retry)
	// => Gain = (Amplification - 1) / log(1+Retry)
	// ====================================
	
	if current.Retry > 0.5 { // Only learn when retry storm is active
		// Approximate Effective Arrival (combining service and queue growth)
		effectiveArrival := approxService + math.Max(0, dQueue)
		amplification := effectiveArrival / math.Max(0.001, current.Arrival)
		
		yAmp := amplification - 1.0
		phiAmp := math.Log(1.0 + current.Retry)
		
		sid.AmplificationRLS.Update(phiAmp, yAmp)
		
		if sid.AmplificationRLS.Theta > 0 {
			sid.Estimate.RetryFeedbackGain = sid.AmplificationRLS.Theta
		}
	}

	sid.Last = &current
}

func (sid *SystemIdentifier) Parameters() ParameterEstimate {
	return sid.Estimate
}

// Fix Problem 4: Trace-based normalized confidence bounds
func (sid *SystemIdentifier) Confidence() ParameterConfidence {
	traceArrival := sid.ArrivalRLS.P[0][0] + sid.ArrivalRLS.P[1][1]
	traceRetry := sid.RetryRLS.P[0][0] + sid.RetryRLS.P[1][1] + sid.RetryRLS.P[2][2]
	
	return ParameterConfidence{
		ArrivalProcess:    1.0 / (1.0 + traceArrival/2.0),
		RetryProcess:      1.0 / (1.0 + traceRetry/3.0),
		ContentionProcess: 1.0 / (1.0 + sid.ContentionRLS.P),
		Amplification:     1.0 / (1.0 + sid.AmplificationRLS.P),
	}
}

func (sid *SystemIdentifier) Apply(cfg *SimConfig) {
	cfg.ArrivalTheta = sid.Estimate.ArrivalTheta
	cfg.ArrivalMean = sid.Estimate.ArrivalMean
	cfg.ArrivalSigma = sid.Estimate.ArrivalSigma
	cfg.RetryAlpha = sid.Estimate.RetryAlpha
	cfg.RetryBeta = sid.Estimate.RetryBeta
	cfg.RetryGamma = sid.Estimate.RetryGamma
	cfg.EfficiencyDecay = sid.Estimate.EfficiencyDecay
	cfg.RetryFeedbackGain = sid.Estimate.RetryFeedbackGain
}