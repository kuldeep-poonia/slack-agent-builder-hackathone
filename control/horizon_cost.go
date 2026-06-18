package control

import (
	"math"
)


// ECONOMIC MODEL PREDICTIVE CONTROL (EMPC) COST ENGINE
// Strictly denominated in actual business costs (USD). Zero arbitrary weights.


type EconomicParams struct {
	// 1. Direct Infrastructure Cloud Cost
	InfraUSDPerSec float64 // e.g., $0.00001 per pod per second

	// 2. Business Impact Costs (Shadow Prices for Exact Penalties)
	SLABreachUSDPerSec float64 // Cost of losing customer goodwill/revenue per second of SLA breach
	QueueDropUSD       float64 // Cost of an HTTP 503 / dropped request
	DowntimeUSDPerSec  float64 // Cost of complete catastrophic system collapse

	// 3. Actuator Churn Costs
	PodChurnUSD     float64 // Network egress/compute cost to spin up/tear down a Pod
	ConfigChurnUSD  float64 // Minimal friction cost for updating Envoy/Mesh configs

	// 4. Halfin-Whitt Quality-of-Service parameter (Beta)
	QoSParameterBeta float64 // 1.0 = ~84% probability of zero delay, 2.0 = ~97%
}

func DefaultEconomicParams() EconomicParams {
	return EconomicParams{
		InfraUSDPerSec:     0.000015, // Roughly $40/month per pod
		SLABreachUSDPerSec: 0.005000, // SLA breaches cost 300x more than infra
		QueueDropUSD:       0.010000, // Hard failures cost money immediately
		DowntimeUSDPerSec:  5.000000, // Complete collapse is unacceptable
		PodChurnUSD:        0.001000, // Small penalty to prevent thrashing replicas
		ConfigChurnUSD:     0.000100, // Micro-penalty to prevent config oscillation
		QoSParameterBeta:   1.5,
	}
}

// EvaluateHorizonCost calculates the trajectory loss strictly in USD.
func EvaluateHorizonCost(
	initial SystemState,
	b Bundle,
	traj Trajectory, // Assumes inclusion of QueueIntegral and FinalRetry
	cfg SimConfig,
	p EconomicParams,
	mem *RegimeMemory,
) float64 {

	horizonTime := float64(cfg.HorizonSteps) * cfg.Dt

	
	// 1. GAIN SCHEDULING (Regime Risk Multiplier)
	
	// Risk scales the perceived business cost of failures (simulating loss of confidence)
	riskMultiplier := 1.0
	if mem != nil {
		riskMultiplier += mem.RiskEWMA
	}

	
	// 2. LINEAR ECONOMIC COST (Fixing Problem 2)
	
	infraCostUSD := p.InfraUSDPerSec * float64(b.Replicas) * horizonTime

	
	// 3. EXACT PENALTY: SLA & COLLAPSE (Fixing Problem 1)
	
	slaCostUSD := p.SLABreachUSDPerSec * riskMultiplier * traj.SLAIntegral
	collapseCostUSD := p.DowntimeUSDPerSec * riskMultiplier * traj.CollapseRisk * horizonTime

	
	// 4. EXACT PENALTY: QUEUE DROPS
	
	// Estimate total requests that will breach the queue limit and 503 out.
	queueCapacity := math.Max(float64(b.QueueLimit), 1.0)
	
	// If QueueIntegral > (Capacity * Time), the excess represents dropped volume.
	maxSafeQueueVolume := queueCapacity * horizonTime
	droppedVolume := math.Max(0, traj.QueueIntegral - maxSafeQueueVolume)
	
	dropCostUSD := p.QueueDropUSD * riskMultiplier * droppedVolume

	
	// 5. ACTUATOR CHURN COST (Fixing Problem 6 - LQR Economic Mapping)
	
	// Replaced abstract L2 math with actual dollar costs of infrastructure operations.
	deltaReplicas := math.Abs(float64(b.Replicas - initial.Replicas))
	podChurnCostUSD := p.PodChurnUSD * deltaReplicas

	// Config updates (Queue, Retry, Cache limits) have a tiny mesh-propagation cost.
	deltaRetry := math.Abs(float64(b.RetryLimit - initial.RetryLimit))
	deltaQueue := math.Abs(float64(b.QueueLimit) - float64(initial.QueueLimit))
	deltaCache := math.Abs(b.CacheAggression - initial.CacheAggression)
	
	// Normalize mesh config changes so they trigger the minimal config churn cost
	meshChurn := deltaRetry/10.0 + deltaQueue/1000.0 + deltaCache
	configChurnCostUSD := p.ConfigChurnUSD * meshChurn

	controlCostUSD := podChurnCostUSD + configChurnCostUSD

	
	// 6. TERMINAL STABILIZATION (Fixing Problems 3 & 4)
	
	// A. Calculate True Offered Load using the simulated plant STATE (traj.FinalRetry)
	// NOT the policy variable (b.RetryLimit).
	retryAmplification := 1.0 + cfg.RetryFeedbackGain*math.Log(1.0+traj.FinalRetry)
	
	// effectiveArrival [Requests / Sec]
	effectiveArrival := initial.PredictedArrival * retryAmplification

	// B. Calculate service degradation using capacity and efficiency decay
	rawCapacity := float64(b.Replicas) * initial.ServiceRate
	rawUtil := effectiveArrival / math.Max(rawCapacity, 0.001)
	
	// degradedService [Requests / Sec / Replica]
	degradedService := initial.ServiceRate / (1.0 + cfg.EfficiencyDecay*math.Max(0, rawUtil-1.0))

	// C. True Halfin-Whitt Offered Load (Erlangs) 
	// [Req/Sec] / [Req/Sec/Replica] = [Replicas]
	offeredLoadErlangs := effectiveArrival / math.Max(degradedService, 0.001)
	
	// D. QED Safe Capacity Boundary [Continuous Replicas]
	safeReplicas := offeredLoadErlangs + p.QoSParameterBeta*math.Sqrt(math.Max(offeredLoadErlangs, 0.001))

	terminalCostUSD := 0.0

	// Rule 1: Final backlog exceeding capacity represents immediate pending drops
	if traj.FinalBacklog > float64(b.QueueLimit) {
		pendingDrops := traj.FinalBacklog - float64(b.QueueLimit)
		terminalCostUSD += pendingDrops * p.QueueDropUSD
	}

	// Rule 2: Ending the horizon structurally doomed (under QED safe limit).
	// We penalize this by projecting the SLA breach cost of the missing replicas over the horizon.
	if float64(b.Replicas) < safeReplicas {
		underProvisionedReplicas := safeReplicas - float64(b.Replicas)
		
		// Map the missing capacity directly to expected SLA economic failure
		missingCapacityPenalty := underProvisionedReplicas * p.SLABreachUSDPerSec * horizonTime
		terminalCostUSD += missingCapacityPenalty
	}

	terminalCostUSD *= riskMultiplier

	
	// TOTAL EMPC ECONOMIC COST (USD)
	
	return infraCostUSD +
		slaCostUSD +
		collapseCostUSD +
		dropCostUSD +
		controlCostUSD +
		terminalCostUSD
}