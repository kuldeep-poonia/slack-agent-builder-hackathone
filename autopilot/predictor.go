package autopilot

import "math"

type CongestionState struct {
	Backlog float64

	ArrivalMean float64
	ArrivalVar  float64

	BurstState       float64
	RegimeConfidence float64

	ServiceRate       float64
	ServiceEfficiency float64
	ConcurrencyLimit  float64

	CapacityActive  float64
	CapacityTarget  float64
	CapacityTauUp   float64
	CapacityTauDown float64

	RetryFactor float64

	Latency       float64
	CPUPressure   float64
	NetworkJitter float64

	Disturbance       float64
	DisturbanceEnergy float64

	UpstreamPressure      float64
	TopologyAmplification float64

	CacheRelief float64
}

type Predictor struct {
	Dt       float64
	MaxQueue float64

	BurstEntryRate         float64
	BurstCollapseThreshold float64
	BurstIntensity         float64

	ArrivalRiseGain   float64
	ArrivalDropGain   float64
	VarianceDecayRate float64

	RetryGain     float64
	RetryDelayTau float64

	DisturbanceSigma         float64
	DisturbanceInjectionGain float64
	DisturbanceBound         float64

	TopologyCouplingK float64
	TopologyAdaptTau  float64

	CacheAdaptTau float64

	LatencyGain float64

	CapacityJitterSigma float64

	BarrierExpK float64
	BarrierCap  float64
}

/*
Bounded burst regime with collapse threshold
*/
func (p *Predictor) updateBurstState(x CongestionState) float64 {

	entry :=
		p.BurstEntryRate * (1 + x.UpstreamPressure)

	val :=
		x.BurstState + entry*p.Dt

	if x.Backlog < p.BurstCollapseThreshold {
		val *= 0.85 // softened decay
	}

	// normalization
	val = val / (1 + val)

	// 🔒 FLOOR (signal ko marne se bachane ke liye)
	if val < 0.05 {
		val = 0.05
	}

	return val
}

/*
Arrival model
*/
func (p *Predictor) arrival(x CongestionState) float64 {

	burst :=
		p.BurstIntensity *
			x.BurstState *
			math.Sqrt(x.ArrivalVar+1)

	return x.ArrivalMean + burst
}

/*
Asymmetric mean + variance decay
*/
func (p *Predictor) updateArrivalStats(
	x CongestionState,
	arrival float64,
) (float64, float64) {

	var gain float64

	if arrival > x.ArrivalMean {
		gain = p.ArrivalRiseGain
	} else {
		gain = p.ArrivalDropGain
	}

	m := (1-gain)*x.ArrivalMean + gain*arrival

	// prevent collapse
	if m < 0.1*x.ArrivalMean {
		m = 0.1 * x.ArrivalMean
	}

	v :=
		(1-p.VarianceDecayRate)*x.ArrivalVar +
			p.VarianceDecayRate*math.Abs(arrival-m)

	return m, v
}

/*
Retry cascade
*/
func (p *Predictor) retryCascade(x CongestionState) float64 {

	lag :=
		math.Exp(-p.Dt / p.RetryDelayTau)

	pressure :=
		(1 - lag) * (x.Backlog + x.Latency)

	load := p.RetryGain * x.RetryFactor * math.Pow(pressure+1, 1.02)

	// clamp
	if load > 5.0 {
		load = 5.0
	}

	return load / (1 + load)
}

/*
Capacity evolution with stochastic jitter
*/
func (p *Predictor) capacityNext(x CongestionState) float64 {
	// Tau represents physical Kubernetes spin-up/down inertia in seconds
	tau := x.CapacityTauUp
	if x.CapacityTarget < x.CapacityActive {
		tau = x.CapacityTauDown
	}
	
	// Prevent division by zero mathematically
	if tau <= 0.1 { tau = 30.0 }

	// Differential Equation: dc/dt = (Target - Current) / Tau
	rate := (x.CapacityTarget - x.CapacityActive) / tau
	
	return x.CapacityActive + (rate * p.Dt)
}

/*
Latency dynamics with service recovery coupling
*/
func (p *Predictor) latencyNext(x CongestionState) float64 {
	activeConns := math.Max(1.0, x.CapacityActive)
	serviceRate := math.Max(0.001, x.ServiceRate)

	// Physics: Base execution time + Queuing delay (L = lambda * W)
	expectedBaseLat := 1.0 / serviceRate
	expectedWaitLat := x.Backlog / (activeConns * serviceRate)
	
	targetLatency := expectedBaseLat + expectedWaitLat + x.NetworkJitter + x.CPUPressure

	// Thermal inertia: Latency doesn't snap instantly, it propagates through network buffers.
	inertiaTau := 5.0 
	rate := (targetLatency - x.Latency) / inertiaTau

	return x.Latency + (rate * p.Dt)
}

/*
Consistent disturbance evolution
*/
func (p *Predictor) disturbanceNext(x CongestionState) (float64, float64) {

	injection :=
		p.DisturbanceInjectionGain *
			(1 + x.UpstreamPressure)

	noise :=
		p.DisturbanceSigma *
			math.Sqrt(x.DisturbanceEnergy+1)

	val :=
		0.8*x.Disturbance +
			injection +
			noise

	if val > p.DisturbanceBound {
		val = p.DisturbanceBound
	}

	energy :=
		math.Max(
			0,
			x.DisturbanceEnergy+
				injection*p.Dt-
				val*p.Dt,
		)

	return val, energy
}

/*
Topology amplification evolution
*/
func (p *Predictor) topologyNext(x CongestionState) float64 {

	target :=
		1 + x.UpstreamPressure

	return x.TopologyAmplification +
		(target-x.TopologyAmplification)*
			(1-math.Exp(-p.Dt/p.TopologyAdaptTau))
}

/*
Cache relief
*/
func (p *Predictor) cacheNext(x CongestionState) float64 {

	target :=
		1 / (1 + math.Sqrt(x.Backlog+1)) *
			(1 + 0.3*x.CapacityActive)

	return x.CacheRelief +
		(target-x.CacheRelief)*
			(1-math.Exp(-p.Dt/p.CacheAdaptTau))
}

/*
Numerically safe hybrid overload barrier
*/
func (p *Predictor) overloadBarrier(q float64) float64 {

	if math.IsNaN(q) || math.IsInf(q, -1) {
		return 0
	}

	if q <= 0 {
		return 0
	}

	if math.IsInf(q, 1) {
		return p.BarrierCap
	}

	if q <= p.MaxQueue {
		return q
	}

	ex := q - p.MaxQueue

	val :=
		p.MaxQueue +
			math.Exp(p.BarrierExpK*ex)

	if val > p.BarrierCap {
		return p.BarrierCap
	}

	return val
}

/*
Single propagation
*/
func (p *Predictor) Step(x CongestionState) CongestionState {
	next := x

	m, v := p.updateArrivalStats(x, p.arrival(x))
	next.ArrivalMean = m
	next.ArrivalVar = v

	// Strictly Physical Capacity
	cap := p.capacityNext(x)
	service := x.ServiceRate * cap

	// Physical Flow Equation (Mass Balance)
	// Accumulation (dQ) = Flow In (Arrival + Retries + Noise) - Flow Out (Service Capacity)
	flowIn := next.ArrivalMean + p.retryCascade(x) + x.Disturbance
	flowOut := service

	dQ := (flowIn - flowOut) * p.Dt

	next.Backlog = p.overloadBarrier(x.Backlog + dQ)
	if next.Backlog < 0 {
		next.Backlog = 0.0 // True physical floor (can't have negative requests)
	}

	next.CapacityActive = cap
	next.Latency = p.latencyNext(x)
	
	dist, energy := p.disturbanceNext(x)
	next.Disturbance = dist
	next.DisturbanceEnergy = energy

	return next
}

/*
Rollout
*/
func (p *Predictor) Rollout(
	x CongestionState,
	h int,
) []CongestionState {

	traj := make([]CongestionState, h)

	state := x

	for i := 0; i < h; i++ {

		state = p.Step(state)

		traj[i] = state
	}

	return traj
}
