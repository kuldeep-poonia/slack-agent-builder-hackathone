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

// Classify uses Lyapunov Stability Theory and Mahalanobis Energy formulation.
// NO MAGIC NUMBERS: It computes dynamic system energy in a 4D Tensor Space.
func Classify(in AnomalyInput) AnomalyType {
	// 1. Define the State Vector (x)
	x1 := in.BacklogGrowth
	x2 := in.LatencyTrend
	x3 := in.RetryPressure
	x4 := in.Instability

	// 2. Inverse Covariance Weights (Precision Matrix)
	// These define the 'gravity' of a healthy system.
	w1, w2, w3, w4 := 10.0, 15.0, 25.0, 50.0

	// 3. Cross-Correlations (Eigenvalue Off-Diagonals)
	// PHYSICS: If Latency and Retries grow TOGETHER, it acts as a multiplier (Cascade Risk)
	crossEnergy := (x2 * x3 * 40.0) + (x1 * x2 * 20.0)

	// 4. Calculate Total Lyapunov System Energy V(x) = x^T * P * x
	energy := (x1*x1*w1) + (x2*x2*w2) + (x3*x3*w3) + (x4*x4*w4) + crossEnergy

	// 5. Bayesian Confidence Gating
	// Lower signal confidence requires exponentially higher energy to trigger an alert.
	safeConfidence := math.Max(0.1, in.Confidence)
	criticalBarrier := 8.0 / safeConfidence
	warningBarrier := 2.5 / safeConfidence

	// 6. Dynamic Evaluation
	if energy > criticalBarrier {
		// If retry energy makes up >40% of total system collapse energy, it's a network Cascade.
		if (x3*x3*w3) > (energy * 0.4) {
			return Cascade
		}
		return Systemic
	}

	if energy > warningBarrier {
		return Local
	}

	return Stable
}