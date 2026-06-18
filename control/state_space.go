package control

import (
	"math"
)

type StateVector [9]float64
type ControlVector [3]float64
type MeasurementVector [5]float64

type Bundle struct {
	Replicas        int
	QueueLimit      float64
	RetryLimit      int
	CacheAggression float64
}

type SystemState struct {
	Replicas         int
	QueueLimit       int
	RetryLimit       int
	CacheAggression  float64
	QueueDepth       float64
	PredictedArrival float64
	ArrivalRate      float64
	ServiceRate      float64
	Latency          float64
	Risk             float64
	Utilisation      float64
	SLATarget        float64
	MinReplicas      int
	MaxReplicas      int
	MinRetry         int
	MaxRetry         int
	
	RetryPressure    float64
	CapacityVelocity float64
	QueueIntegral    float64
	SLAIntegral      float64
	DroppedRequests  float64
	Survival         float64
	
	// NEW: Failure Classification
	FailureMode string 
}

type DynamicState struct {
	Queue            float64
	Retry            float64
	Capacity         float64
	Arrival          float64
	CapacityVelocity float64
	QueueIntegral    float64
	SLAIntegral      float64
	DroppedRequests  float64
	Survival         float64
}

func (s *DynamicState) ToVector() StateVector {
	return StateVector{ s.Queue, s.Retry, s.Capacity, s.Arrival, s.CapacityVelocity, s.QueueIntegral, s.SLAIntegral, s.DroppedRequests, s.Survival }
}

func (s *DynamicState) FromVector(v StateVector) {
	s.Queue = math.Max(v[0], 0.0)
	s.Retry = math.Max(v[1], 0.0)
	s.Capacity = math.Max(v[2], 0.001)
	s.Arrival = math.Max(v[3], 0.001)
	s.CapacityVelocity = v[4]
	s.QueueIntegral = math.Max(v[5], 0.0)
	s.SLAIntegral = math.Max(v[6], 0.0)
	s.DroppedRequests = math.Max(v[7], 0.0)
	s.Survival = math.Max(0.0, math.Min(1.0, v[8]))
}

// CORE FIX: 5-Argument ComputeLatency using Kingman's formula + Little's Law
func ComputeLatency(queue, arrival, capacity, baseLatency, maxQueueDelay float64) float64 {
	const epsilon = 0.001
	latency := baseLatency

	if capacity > epsilon {
		latency += queue / capacity // Backlog clearance time
		util := arrival / capacity
		if util > 0.01 {
			safeUtil := math.Min(util, 0.99)
			latency += (safeUtil / (1.0 - safeUtil)) * (1.0 / capacity) // Contention delay
		}
	}
	
	if latency > maxQueueDelay { return maxQueueDelay }
	return latency
}

func NewDynamicState(s SystemState) DynamicState {
	cap := float64(s.Replicas) * s.ServiceRate
	surv := s.Survival
	if surv <= 0.0 || surv > 1.0 { surv = 1.0 }

	return DynamicState{
		Queue:            math.Max(s.QueueDepth, 0.0),
		Retry:            math.Max(s.RetryPressure, 0.0),
		Capacity:         math.Max(cap, 0.001),
		Arrival:          math.Max(s.PredictedArrival, 0.001),
		CapacityVelocity: s.CapacityVelocity,
		QueueIntegral:    math.Max(s.QueueIntegral, 0.0),
		SLAIntegral:      math.Max(s.SLAIntegral, 0.0),
		DroppedRequests:  math.Max(s.DroppedRequests, 0.0),
		Survival:         surv,
	}
}

func MapSystemState(sys *SystemState, v StateVector, serviceRate float64) {
	sys.QueueDepth = math.Max(0.0, v[0])
	sys.RetryPressure = math.Max(0.0, v[1])
	sys.PredictedArrival = math.Max(0.001, v[3])
	sys.CapacityVelocity = v[4]
	sys.QueueIntegral = v[5]
	sys.SLAIntegral = v[6]
	sys.DroppedRequests = v[7]
	sys.Survival = v[8]
	sys.ServiceRate = serviceRate
}

func (s DynamicState) Clone() DynamicState { return s }