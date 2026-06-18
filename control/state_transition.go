package control

import (
	"math"
)

func StateTransition(
	state *DynamicState,
	u ControlVector, 
	initial SystemState,
	cfg SimConfig,
	noise float64, // CORE FIX: Pure float, zero PRNG overhead
) {
	const epsilon = 0.001

	targetCapacity  := u[0]
	queueLimit      := math.Max(u[1], 1.0)
	cacheAggression := clampFloat(u[2], 0.0, 1.0)

	drift := cfg.ArrivalTheta * (cfg.ArrivalMean - state.Arrival) * cfg.Dt
	diffusion := cfg.ArrivalSigma * math.Sqrt(cfg.Dt) * noise

	state.Arrival += drift + diffusion
	state.Arrival = math.Max(state.Arrival, epsilon)

	omega := cfg.NaturalFrequency
	zeta := cfg.DampingRatio

	capacityAcceleration := (omega * omega * (targetCapacity - state.Capacity)) -
		(2.0 * zeta * omega * state.CapacityVelocity)

	state.CapacityVelocity += capacityAcceleration * cfg.Dt
	state.Capacity += state.CapacityVelocity * cfg.Dt
	state.Capacity = math.Max(state.Capacity, initial.ServiceRate)

	retryAmplification := 1.0 + cfg.RetryFeedbackGain*math.Log(1.0+state.Retry)
	cacheReliefGain := math.Exp(-cacheAggression)
	totalInboundDemand := state.Arrival * retryAmplification * cacheReliefGain

	queueSaturatingPressure := state.Queue / queueLimit
	admissionProbability := 1.0 / (1.0 + queueSaturatingPressure)
	effectiveAdmittedArrival := totalInboundDemand * admissionProbability

	droppedRate := totalInboundDemand * (1.0 - admissionProbability)
	state.DroppedRequests += droppedRate * cfg.Dt

	systemUtilization := effectiveAdmittedArrival / math.Max(state.Capacity, epsilon)
	contentionPenaltyScale := 1.0 + cfg.EfficiencyDecay*math.Max(0.0, systemUtilization-1.0)
	
	candidateReplicas := targetCapacity / math.Max(initial.ServiceRate, 0.001)
	inferredBeta := 0.0001 
	if initial.FailureMode == "DegradedDownstream" { inferredBeta = 0.01 }
	uslPenalty := 1.0 + (inferredBeta * candidateReplicas * candidateReplicas)
	actualDegradedServiceRate := (state.Capacity / contentionPenaltyScale) / uslPenalty

	queueDerivative := effectiveAdmittedArrival - actualDegradedServiceRate
	state.Queue += queueDerivative * cfg.Dt
	state.Queue = math.Max(state.Queue, 0.0)
	state.QueueIntegral += state.Queue * cfg.Dt

	currentCalculatedLatency := ComputeLatency(
		state.Queue, effectiveAdmittedArrival, actualDegradedServiceRate, 
		cfg.BaseLatency, cfg.MaxQueueDelay,
	)

	if currentCalculatedLatency > initial.SLATarget {
		state.SLAIntegral += (currentCalculatedLatency - initial.SLATarget) * cfg.Dt
	}

	slaViolationDelta := math.Max(0.0, currentCalculatedLatency-initial.SLATarget)
	normalizedQueueState := state.Queue / queueLimit
	normalizedSLAViolation := slaViolationDelta / math.Max(initial.SLATarget, epsilon)

	retryDerivative := (cfg.RetryAlpha * normalizedQueueState) +
		(cfg.RetryGamma * normalizedSLAViolation) - (cfg.RetryBeta * state.Retry)

	state.Retry += retryDerivative * cfg.Dt
	state.Retry = math.Max(state.Retry, 0.0)

	queueOverloadBound := math.Max(0.0, (state.Queue/queueLimit)-2.0)
	retryStormOverload := math.Max(0.0, state.Retry-10.0)
	lethalHazardIntensity := (queueOverloadBound * queueOverloadBound) + retryStormOverload
	
	state.Survival *= math.Exp(-lethalHazardIntensity * cfg.Dt)
	state.Survival = math.Max(0.0, math.Min(1.0, state.Survival))
}

func clampFloat(v, lo, hi float64) float64 {
	if v < lo { return lo }
	if v > hi { return hi }
	return v
}