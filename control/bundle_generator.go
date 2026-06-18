package control

import (
	"math"
)

type GeneratorConfig struct {
	MaxScaleSurge int     
	MaxScaleDown  int     
	MaxQueueDelta float64 
	QoSParameterBeta float64
	ReplicaSteps int 
	QueueSteps   int
	CacheSteps   int
}

func DefaultGeneratorConfig() GeneratorConfig {
	return GeneratorConfig{
		MaxScaleSurge:    50,
		MaxScaleDown:     10,
		MaxQueueDelta:    1000.0,
		QoSParameterBeta: 1.5,
		ReplicaSteps:     5, 
		QueueSteps:       3,
		CacheSteps:       3,
	}
}

func GenerateBundles(current SystemState, cfg GeneratorConfig, simCfg SimConfig) []Bundle {
	retryAmplification := 1.0 + simCfg.RetryFeedbackGain*math.Log(1.0+current.RetryPressure)
	effectiveArrival := current.PredictedArrival * retryAmplification
	
	rawUtil := effectiveArrival / math.Max(float64(current.Replicas)*current.ServiceRate, 0.001)
	degradedService := current.ServiceRate / (1.0 + simCfg.EfficiencyDecay*math.Max(0, rawUtil-1.0))
	
	offeredLoad := effectiveArrival / math.Max(degradedService, 0.001)
	idealReplicasFloat := offeredLoad + cfg.QoSParameterBeta*math.Sqrt(math.Max(offeredLoad, 0.001))
	idealReplicas := int(math.Ceil(idealReplicasFloat))

	minReplicasNext := localMaxInt(current.MinReplicas, current.Replicas-cfg.MaxScaleDown)
	maxReplicasNext := localMinInt(current.MaxReplicas, current.Replicas+cfg.MaxScaleSurge)
	centerReplicas := localClampInt(idealReplicas, minReplicasNext, maxReplicasNext)

	replicaStepSize := localMaxInt(1, (maxReplicasNext-minReplicasNext)/cfg.ReplicaSteps)
	queueStepSize := cfg.MaxQueueDelta / float64(cfg.QueueSteps)

	// O(1) Pre-allocation
	bundles := make([]Bundle, 0, (cfg.ReplicaSteps+1)*(cfg.QueueSteps+1)*3*cfg.CacheSteps + 1)
	
	baseline := bundleFromState(current)
	bundles = append(bundles, baseline)

	for r := -cfg.ReplicaSteps / 2; r <= cfg.ReplicaSteps/2; r++ {
		rep := centerReplicas + (r * replicaStepSize)
		rep = localClampInt(rep, minReplicasNext, maxReplicasNext)

		for q := -cfg.QueueSteps / 2; q <= cfg.QueueSteps/2; q++ {
			queue := float64(current.QueueLimit) + (float64(q) * queueStepSize)
			queue = math.Max(1.0, queue) 

			retryStates := []int{current.MaxRetry, current.MaxRetry / 2, 0}
			
			for _, retry := range retryStates {
				retry = localClampInt(retry, current.MinRetry, current.MaxRetry)

				for c := 0; c < cfg.CacheSteps; c++ {
					cache := float64(c) / float64(math.Max(1.0, float64(cfg.CacheSteps-1)))

					bundle := Bundle{
						Replicas:        rep,
						QueueLimit:      queue,
						RetryLimit:      retry,
						CacheAggression: cache,
					}
					
					// Structural O(1) comparison natively deduplicates
					if bundle != baseline {
						bundles = append(bundles, bundle)
					}
				}
			}
		}
	}

	return bundles
}

func localMinInt(a, b int) int { if a < b { return a }; return b }
func localMaxInt(a, b int) int { if a > b { return a }; return b }
func localClampInt(val, min, max int) int {
	if val < min { return min }
	if val > max { return max }
	return val
}