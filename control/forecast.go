package control

type Trajectory struct {
	FinalBacklog    float64
	FinalRetry      float64
	PeakLatency     float64
	QueueIntegral   float64
	SLAIntegral     float64
	DroppedRequests float64
	CollapseRisk    float64
}

func Forecast(
	initial SystemState,
	b Bundle,
	cfg SimConfig,
	noiseSequence []float64, // CORE FIX: Receive the raw noise matrix row
) Trajectory {

	state := NewDynamicState(initial)
	var inputU ControlVector
	inputU[0] = float64(b.Replicas) * initial.ServiceRate
	inputU[1] = b.QueueLimit
	inputU[2] = b.CacheAggression

	peakLatency := ComputeLatency(state.Queue, state.Arrival, state.Capacity, cfg.BaseLatency, cfg.MaxQueueDelay)

	for step := 0; step < cfg.HorizonSteps; step++ {
		// Pass pre-computed noise directly to physics engine
		StateTransition(&state, inputU, initial, cfg, noiseSequence[step])

		currentL := ComputeLatency(state.Queue, state.Arrival, state.Capacity, cfg.BaseLatency, cfg.MaxQueueDelay)
		if currentL > peakLatency { peakLatency = currentL }
	}

	return Trajectory{
		FinalBacklog:    state.Queue, FinalRetry:      state.Retry, PeakLatency:     peakLatency,
		QueueIntegral:   state.QueueIntegral, SLAIntegral:     state.SLAIntegral,
		DroppedRequests: state.DroppedRequests, CollapseRisk:    1.0 - state.Survival,
	}
}