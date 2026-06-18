package control

import (
	"math"
	"math/rand"
	"sort"
)

type OptimizerConfig struct {
	ScenarioCount   int     
	ConfidenceLevel float64 
	RiskAversion    float64 
}

func DefaultOptimizerConfig() OptimizerConfig {
	return OptimizerConfig{
		ScenarioCount:   50,    
		ConfidenceLevel: 0.95,  
		RiskAversion:    0.30,  
	}
}

type RobustMPC struct {
	Cfg          OptimizerConfig
	costsBuffer  []float64 
	sortedBuffer []float64 
	noiseMatrix  [][]float64 
	fastRNG      *rand.Rand // CORE FIX: The zero-allocation, lifetime PRNG
}

func NewRobustMPC(cfg OptimizerConfig) *RobustMPC {
	matrix := make([][]float64, cfg.ScenarioCount)
	for i := range matrix { matrix[i] = make([]float64, 100) } 
	return &RobustMPC{
		Cfg:          cfg, 
		costsBuffer:  make([]float64, cfg.ScenarioCount),
		sortedBuffer: make([]float64, cfg.ScenarioCount),
		noiseMatrix:  matrix,
	}
}

func (mpc *RobustMPC) Optimize(
	initial SystemState,
	candidates []Bundle,
	simCfg SimConfig,
	econParams EconomicParams,
	mem *RegimeMemory,
	baseSeed int64,
) Bundle {

	if len(candidates) == 0 { return bundleFromState(initial) }

	if len(mpc.costsBuffer) < mpc.Cfg.ScenarioCount {
		mpc.costsBuffer = make([]float64, mpc.Cfg.ScenarioCount)
		mpc.sortedBuffer = make([]float64, mpc.Cfg.ScenarioCount)
		mpc.noiseMatrix = make([][]float64, mpc.Cfg.ScenarioCount)
		for i := range mpc.noiseMatrix { mpc.noiseMatrix[i] = make([]float64, 100) }
	}

	// ========================================================================
	// THE FINAL HYPERSCALER FIX: THE "LIFETIME" PRNG
	// This executes exactly ONCE per simulation universe. It drops the PRNG
	// initialization count from 1.6 Million down to 0 in the hot path.
	// ========================================================================
	if mpc.fastRNG == nil {
		mpc.fastRNG = rand.New(rand.NewSource(baseSeed))
	}

	// Generate the noise paths instantly
	for i := 0; i < mpc.Cfg.ScenarioCount; i++ {
		for step := 0; step < simCfg.HorizonSteps; step++ {
			mpc.noiseMatrix[i][step] = mpc.fastRNG.NormFloat64()
		}
	}

	bestBundle := candidates[0]
	minObjectiveCost := math.MaxFloat64

	for _, candidate := range candidates {
		for i := 0; i < mpc.Cfg.ScenarioCount; i++ {
			traj := Forecast(initial, candidate, simCfg, mpc.noiseMatrix[i])
			mpc.costsBuffer[i] = EvaluateHorizonCost(initial, candidate, traj, simCfg, econParams, mem)
		}

		meanCost, cvarCost := mpc.calculateMeanAndCVaR(mpc.costsBuffer)
		objectiveCost := (1.0 - mpc.Cfg.RiskAversion)*meanCost + (mpc.Cfg.RiskAversion)*cvarCost

		if objectiveCost < minObjectiveCost {
			minObjectiveCost = objectiveCost
			bestBundle = candidate
		} else if math.Abs(objectiveCost-minObjectiveCost) < 0.001 { 
			if actuatorChurnUSD(initial, candidate, econParams) < actuatorChurnUSD(initial, bestBundle, econParams) { 
				bestBundle = candidate 
			}
		}
	}
	return bestBundle
}

// Optimized internally to avoid all slice allocations
func (mpc *RobustMPC) calculateMeanAndCVaR(costs []float64) (mean float64, cvar float64) {
	if len(costs) == 0 { return 0.0, 0.0 }
	
	sum := 0.0
	for i, c := range costs { 
		sum += c
		mpc.sortedBuffer[i] = c 
	}
	mean = sum / float64(len(costs))

	sort.Float64s(mpc.sortedBuffer)

	varIdx := int(math.Floor(mpc.Cfg.ConfidenceLevel * float64(len(mpc.sortedBuffer))))
	if varIdx < 0 { varIdx = 0 }
	if varIdx >= len(mpc.sortedBuffer) { varIdx = len(mpc.sortedBuffer) - 1 }

	tailSum := 0.0
	tailCount := 0.0
	for i := varIdx; i < len(mpc.sortedBuffer); i++ { 
		tailSum += mpc.sortedBuffer[i]
		tailCount++ 
	}

	if tailCount > 0 { 
		cvar = tailSum / tailCount 
	} else { 
		cvar = mpc.sortedBuffer[len(mpc.sortedBuffer)-1] 
	}
	
	return mean, cvar
}

func actuatorChurnUSD(initial SystemState, b Bundle, p EconomicParams) float64 {
	return (p.PodChurnUSD * math.Abs(float64(b.Replicas - initial.Replicas))) + 
		(p.ConfigChurnUSD * (math.Abs(float64(b.RetryLimit - initial.RetryLimit))/10.0 + math.Abs(float64(b.QueueLimit) - float64(initial.QueueLimit))/1000.0 + math.Abs(b.CacheAggression - initial.CacheAggression)))
}

func bundleFromState(s SystemState) Bundle { 
	return Bundle{ Replicas: s.Replicas, RetryLimit: s.RetryLimit, QueueLimit: float64(s.QueueLimit), CacheAggression: s.CacheAggression } 
}