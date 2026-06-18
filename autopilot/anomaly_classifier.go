package autopilot

import "math"

type AnomalyType string

const (
	Stable   AnomalyType = "stable"
	Local    AnomalyType = "local"
	Systemic AnomalyType = "systemic"
	Cascade  AnomalyType = "cascade"
)

type AnomalyInput struct {
	Instability   float64
	Confidence    float64
	BacklogGrowth float64
	LatencyTrend  float64
	RetryPressure float64
	Oscillation   float64
}

// 🧠 PRODUCTION UPGRADE: Lyapunov Energy formulated Anomaly Detection
func Classify(in AnomalyInput) AnomalyType {
	x1, x2, x3, x4 := in.BacklogGrowth, in.LatencyTrend, in.RetryPressure, in.Instability

	// Inverse Covariance Matrix approximations (Precision Weights)
	w1, w2, w3, w4 := 10.0, 15.0, 25.0, 50.0

	// Cross-correlation: If Latency and Retries grow together, danger multiplies
	crossEnergy := (x2 * x3 * 40.0) + (x1 * x2 * 20.0)

	// Total System Energy (V(x))
	energy := (x1*x1*w1) + (x2*x2*w2) + (x3*x3*w3) + (x4*x4*w4) + crossEnergy

	safeConfidence := math.Max(0.1, in.Confidence)
	criticalBarrier := 8.0 / safeConfidence
	warningBarrier := 2.5 / safeConfidence

	if energy > criticalBarrier {
		if (x3*x3*w3) > (energy * 0.4) {
			return Cascade // Retry storm dominates the energy
		}
		return Systemic
	}

	if energy > warningBarrier {
		return Local
	}

	return Stable
}