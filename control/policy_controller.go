package control

import (
	"math"
	"time"
)

type ControllerConfig struct {
	ServiceRateFilterAlpha float64 
	MinServiceRateSafety   float64 
	MaxServiceRateSafety   float64 
	SystemWarmupWindowSec  float64 
	MinConfidenceGateLimit float64 
	DefaultHorizonSteps  int
	DefaultIntegrationDt float64
}

func DefaultControllerConfig() ControllerConfig {
	return ControllerConfig{
		ServiceRateFilterAlpha: 0.05,
		MinServiceRateSafety:   0.001, 
		MaxServiceRateSafety:   10000.0,
		SystemWarmupWindowSec:  60.0,
		MinConfidenceGateLimit: 0.15,
		DefaultHorizonSteps:    10,
		DefaultIntegrationDt:   1.0,
	}
}

type ServiceRateObserver struct { CurrentServiceRate float64 }

func (sro *ServiceRateObserver) Observe(zTelemetry [5]float64, activeReplicas int, cfg ControllerConfig) float64 {
	measuredCapacity := zTelemetry[3]
	if measuredCapacity > 0.001 && activeReplicas > 0 {
		rawServiceObservation := measuredCapacity / float64(activeReplicas)
		sro.CurrentServiceRate = (cfg.ServiceRateFilterAlpha * rawServiceObservation) +
			((1.0 - cfg.ServiceRateFilterAlpha) * sro.CurrentServiceRate)
	}
	if sro.CurrentServiceRate < cfg.MinServiceRateSafety { sro.CurrentServiceRate = cfg.MinServiceRateSafety }
	if sro.CurrentServiceRate > cfg.MaxServiceRateSafety { sro.CurrentServiceRate = cfg.MaxServiceRateSafety }
	return sro.CurrentServiceRate
}

type CalibrationManager struct {
	TimeElapsed  float64
	IsCalibrated bool
}
func (cm *CalibrationManager) Track(dt float64, cfg ControllerConfig) bool {
	cm.TimeElapsed += dt; if cm.TimeElapsed > cfg.SystemWarmupWindowSec { cm.IsCalibrated = true }; return cm.IsCalibrated
}

type ConfidenceGate struct{}
func (cg *ConfidenceGate) Validate(conf ParameterConfidence, cfg ControllerConfig) bool {
	return conf.ArrivalProcess > cfg.MinConfidenceGateLimit && conf.RetryProcess > cfg.MinConfidenceGateLimit
}

type Controller struct {
	EKF         *ExtendedKalmanFilter
	SysID       *SystemIdentifier
	ParamEst    *ParameterEstimator 
	MPC         *RobustMPC
	ActState    *ActuatorState
	SRObserver  *ServiceRateObserver
	Calibrator  *CalibrationManager
	GateKeeper  *ConfidenceGate

	CtrlCfg     ControllerConfig
	SimCfg      SimConfig      
	EconCfg     EconomicParams 
	ActuatorCfg ActuatorConfig 
	GenCfg      GeneratorConfig
	RegimeCfg   RegimeConfig   

	LastDecision Bundle
	MasterSeed   int64
}

func NewController(baseSeed int64, minDeps, maxDeps int, ctrlCfg ControllerConfig) *Controller {
	if baseSeed == 0 { baseSeed = time.Now().UnixNano() }

	return &Controller{
		EKF:          NewExtendedKalmanFilter(DefaultEKFConfig()),
		SysID:        NewSystemIdentifier(),
		ParamEst:     NewParameterEstimator(), 
		MPC:          NewRobustMPC(DefaultOptimizerConfig()),
		ActState:     &ActuatorState{ReadyReplicas: float64(minDeps), TargetReplicas: minDeps, SafeModeTicks: 0},
		SRObserver:   &ServiceRateObserver{CurrentServiceRate: 10.0},
		Calibrator:   &CalibrationManager{TimeElapsed: 0.0, IsCalibrated: false},
		GateKeeper:   &ConfidenceGate{},
		LastDecision: Bundle{Replicas: minDeps, QueueLimit: 1000.0, RetryLimit: 3, CacheAggression: 0.0},
		MasterSeed:   baseSeed,
		
		CtrlCfg:     ctrlCfg,
		RegimeCfg:   DefaultRegimeConfig(),
		EconCfg:     DefaultEconomicParams(),
		ActuatorCfg: DefaultActuatorConfig(),
		GenCfg:      DefaultGeneratorConfig(),
		SimCfg: SimConfig{
			HorizonSteps:      ctrlCfg.DefaultHorizonSteps,
			Dt:                ctrlCfg.DefaultIntegrationDt,
			BaseLatency:       0.005,
			MaxQueueDelay:     10.000, 
			NaturalFrequency:  0.25,
			DampingRatio:      0.707,
			ArrivalTheta:      0.20,
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

func (c *Controller) Tick(zTelemetry MeasurementVector, currentSysState *SystemState, mem *RegimeMemory, dt float64) SystemState {
	if dt <= 0.0 { dt = c.SimCfg.Dt }

	// Byzantine Fault Shield
	for i := 0; i < 5; i++ {
		if math.IsNaN(zTelemetry[i]) || math.IsInf(zTelemetry[i], 0) {
			switch i {
			case 0: zTelemetry[0] = currentSysState.QueueDepth
			case 1: zTelemetry[1] = currentSysState.Latency
			case 2: zTelemetry[2] = currentSysState.RetryPressure
			case 3: zTelemetry[3] = math.Max(0.001, float64(currentSysState.Replicas)*currentSysState.ServiceRate)
			case 4: zTelemetry[4] = currentSysState.PredictedArrival
			}
		}
	}

	c.ActuatorCfg.MinReplicas = currentSysState.MinReplicas
	c.ActuatorCfg.MaxReplicas = currentSysState.MaxReplicas

	var inputU ControlVector
	inputU[0] = float64(c.LastDecision.Replicas) * currentSysState.ServiceRate
	inputU[1] = c.LastDecision.QueueLimit
	inputU[2] = c.LastDecision.CacheAggression

	c.EKF.Predict(inputU, *currentSysState, c.SimCfg, dt)
	c.EKF.Update(zTelemetry)

	filteredState := c.EKF.X

	currentSysState.QueueDepth = math.Max(0.0, filteredState[0])
	currentSysState.RetryPressure = math.Max(0.0, filteredState[1])
	currentSysState.PredictedArrival = math.Max(0.001, filteredState[3])
	currentSysState.CapacityVelocity = filteredState[4]

	currentActiveReplicas := int(math.Round(c.ActState.ReadyReplicas))
	if currentActiveReplicas < 1 { currentActiveReplicas = 1 }

	currentSysState.ServiceRate = c.SRObserver.Observe([5]float64(zTelemetry), currentActiveReplicas, c.CtrlCfg)
	currentSysState.Utilisation = currentSysState.PredictedArrival / math.Max(filteredState[2], 0.001)

	predictedLatency := ComputeLatency(currentSysState.QueueDepth, currentSysState.PredictedArrival, filteredState[2], c.SimCfg.BaseLatency, c.SimCfg.MaxQueueDelay)
	observedLatency := zTelemetry[1] 

	currentSysState.Latency = (0.1 * math.Min(predictedLatency, 10.0)) + (0.9 * observedLatency)

	currentSysState.FailureMode = "Healthy"
	
	// 1. Little's Law Failure Detection (Removes 5.0x SLA hardcode)
	expectedSlaQueue := currentSysState.PredictedArrival * currentSysState.SLATarget
	if currentSysState.QueueDepth > expectedSlaQueue * 1.5 && currentSysState.Latency > currentSysState.SLATarget {
		if filteredState[2] < currentSysState.PredictedArrival * 0.5 {
			currentSysState.FailureMode = "CapacityExhausted"
		} else {
			currentSysState.FailureMode = "DegradedDownstream"
		}
	}

	qRisk := currentSysState.QueueDepth / math.Max(1.0, float64(c.LastDecision.QueueLimit))
	lRisk := currentSysState.Latency / math.Max(0.001, currentSysState.SLATarget) 
	rRisk := currentSysState.RetryPressure / 5.0 
	currentSysState.Risk = qRisk + lRisk + rRisk 

	if mem != nil { mem.Update(*currentSysState, currentSysState.SLATarget, 0.0, c.RegimeCfg) }

	// 2. Physics-based Partition Detection (Removes 3000.0 queue hardcode)
	absoluteTimeoutSec := 30.0
	physicalMaxDrain := math.Max(0.001, float64(currentActiveReplicas)*currentSysState.ServiceRate) * absoluteTimeoutSec
	isControlPlanePartitioned := zTelemetry[0] > physicalMaxDrain && currentSysState.PredictedArrival <= 0.01 
	isObservabilityBroken := (observedLatency > 5.0) && (filteredState[0] < 10.0)

	if isControlPlanePartitioned || isObservabilityBroken || c.ActState.SafeModeTicks > 0 {
		if isControlPlanePartitioned || isObservabilityBroken { c.ActState.SafeModeTicks = 15.0 }
		c.ActState.SafeModeTicks -= dt
		
		currentSysState.FailureMode = "NetworkPartition_SafeMode"
		safeTarget := int(math.Min(float64(currentSysState.MaxReplicas), float64(c.ActState.TargetReplicas + 2)))
		safeQueue := float64(c.LastDecision.QueueLimit) + 25.0 
		if safeQueue > 1000.0 { safeQueue = 1000.0 }
		
		safeBundle := Bundle{ Replicas: safeTarget, QueueLimit: safeQueue, RetryLimit: c.LastDecision.RetryLimit, CacheAggression: 1.0 }
		c.LastDecision = safeBundle
		ApplyActuatorDynamics(currentSysState, c.ActState, safeBundle, c.ActuatorCfg, dt)
		return *currentSysState
	}

	currentObs := Observation{ Queue: currentSysState.QueueDepth, Latency: currentSysState.Latency, Retry: currentSysState.RetryPressure, Arrival: currentSysState.PredictedArrival, Capacity: filteredState[2], Time: c.Calibrator.TimeElapsed }
	if c.Calibrator.Track(dt, c.CtrlCfg) {
		c.SysID.Update(currentObs)
		if c.GateKeeper.Validate(c.SysID.Confidence(), c.CtrlCfg) {
			c.ParamEst.Update(c.SysID.Parameters(), c.SysID.Confidence())
			c.ParamEst.Apply(&c.SimCfg)
		}
	}

	if float64(c.ActState.TargetReplicas) > (currentSysState.PredictedArrival / math.Max(0.001, currentSysState.ServiceRate)) * 1.5 {
		c.GenCfg.MaxScaleSurge = 0 
	} else if currentSysState.Utilisation < 0.5 && currentSysState.QueueDepth < 10.0 {
		c.GenCfg.MaxScaleSurge = 5 
	} else {
		c.GenCfg.MaxScaleSurge = 100 
	}

	candidates := GenerateBundles(*currentSysState, c.GenCfg, c.SimCfg)
	optimalBundle := c.MPC.Optimize(*currentSysState, candidates, c.SimCfg, c.EconCfg, mem, c.MasterSeed)

	maxPodJump := int(math.Max(10.0, float64(c.ActState.TargetReplicas)*0.15))
	if optimalBundle.Replicas > c.ActState.TargetReplicas + maxPodJump {
		optimalBundle.Replicas = c.ActState.TargetReplicas + maxPodJump
	}

	currentPhysicalCapacity := math.Max(0.001, float64(c.ActState.ReadyReplicas)*currentSysState.ServiceRate)
	maxSlaQueue := currentPhysicalCapacity * currentSysState.SLATarget
	absoluteMaxQueue := math.Max(25.0, maxSlaQueue * 1.5) 
	
	if optimalBundle.QueueLimit > absoluteMaxQueue { optimalBundle.QueueLimit = absoluteMaxQueue }
	if optimalBundle.QueueLimit < 25.0 { optimalBundle.QueueLimit = 25.0 }

	// 3. Control Barrier Functions (CBF) Slew Rates (Removes 20% hardcode drop)
	currentQ := float64(c.LastDecision.QueueLimit)
	gamma := 0.5 
	
	maxDrainPerTick := math.Max(1.0, float64(c.ActState.ReadyReplicas) * currentSysState.ServiceRate * dt)
	cbfLowerBound := currentQ - (gamma * maxDrainPerTick)
	
	maxAccumulationPerTick := math.Max(1.0, currentSysState.PredictedArrival * dt)
	cbfUpperBound := currentQ + (gamma * maxAccumulationPerTick)

	if currentSysState.FailureMode == "Healthy" {
		if optimalBundle.QueueLimit < cbfLowerBound {
			optimalBundle.QueueLimit = cbfLowerBound
		}
		if optimalBundle.QueueLimit > cbfUpperBound {
			optimalBundle.QueueLimit = cbfUpperBound
		}
	} else {
		if optimalBundle.QueueLimit < currentQ * 0.10 { optimalBundle.QueueLimit = currentQ * 0.10 }
	}

	c.LastDecision = optimalBundle
	ApplyActuatorDynamics(currentSysState, c.ActState, optimalBundle, c.ActuatorCfg, dt)

	c.MasterSeed++
	return *currentSysState
}