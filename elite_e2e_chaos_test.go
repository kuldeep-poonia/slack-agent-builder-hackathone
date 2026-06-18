package autonomus_test

import (
	"fmt"
	"math"
	"math/rand"
	"testing"
	"time"

	"autonomus/autopilot"
	"autonomus/control"
	"autonomus/modelling"
	"autonomus/telemetry"
	"autonomus/topology"
)

func TestEliteAutonomousPipeline_MultiPhaseChaos(t *testing.T) {
	fmt.Println("🚀 INITIATING ELITE MULTI-PHASE CHAOS ENGINEERING SUITE")
	fmt.Println("=========================================================================================")

	rand.Seed(42)
	startTime := time.Now()

	ringBuf := telemetry.NewRingBuffer(100)
	queueEngine := modelling.NewQueuePhysicsEngine()

	ekfConfig := control.DefaultEKFConfig()
	ekf := control.NewExtendedKalmanFilter(ekfConfig)

	predictor := &autopilot.Predictor{
		Dt:                     2.0,
		MaxQueue:               10000.0,
		BurstEntryRate:         0.5,
		BurstCollapseThreshold: 500.0,
		BarrierCap:             20000.0,
		ArrivalRiseGain:        0.2,
		ArrivalDropGain:        0.05,
		VarianceDecayRate:      0.1,
		RetryGain:              0.8,
		LatencyGain:            1.2,
		TopologyCouplingK:      0.4, // Topology Amplifier
		TopologyAdaptTau:       5.0,
	}

	// Simulation Variables
	currentRPS := 1000.0
	currentLatency := 45.0
	currentQueue := 0.0
	currentRetryPressure := 0.0
	currentUpstreamPressure := 0.0 // TOPOLOGY: DB/Dependency Health

	replicas := 5.0
	targetReplicas := 5.0
	baseServiceRate := 300.0
	currentServiceRate := baseServiceRate
	slaLatency := 200.0

	var anomalyState autopilot.AnomalyType = autopilot.Stable
	var mpcActionTaken bool
	
	// 🧠 PREDICTOR VALIDATION TRACKER (MAPE)
	// Map to store: Expected T+1 Queue -> at Minute X
	forecastTracker := make(map[int]float64)
	var totalErrorPercentage float64
	var anomalyTicks int

	// TIME-SERIES SIMULATION LOOP (80 Minutes for Multi-Phase)
	for minute := 1; minute <= 80; minute++ {
		timestamp := startTime.Add(time.Duration(minute) * time.Minute)

		// ---------------------------------------------------------
		// 📊 PREDICTOR VALIDATION (MAPE Calculation)
		// ---------------------------------------------------------
		if expectedQueue, exists := forecastTracker[minute]; exists {
			// Only measure MAPE during congestion (Queue > 10) to avoid dividing by near-zero noise
			if currentQueue > 10.0 {
				absoluteError := math.Abs(currentQueue - expectedQueue)
				mape := (absoluteError / currentQueue) * 100.0
				totalErrorPercentage += mape
				anomalyTicks++
			}
		}

		// ---------------------------------------------------------
		// 🐳 ACTUATOR DYNAMICS
		// ---------------------------------------------------------
		if replicas < targetReplicas {
			step := math.Min(targetReplicas-replicas, 3.0)
			replicas += step
		} else if replicas > targetReplicas {
			step := math.Min(replicas-targetReplicas, 2.0)
			replicas -= step
		}

		// =========================================================
		// 🔥 MULTI-PHASE CHAOS INJECTION ENGINE
		// =========================================================
		
		// 🌀 PHASE 2: TOPOLOGY CASCADE (DB Slowdown) [Mins 15 - 30]
		if minute >= 15 && minute < 30 {
			if minute == 15 { fmt.Printf("\n💥 [CHAOS EVENT: TOPOLOGY] Database Dependency is Choking!\n") }
			currentServiceRate = baseServiceRate * 0.4 // DB slows down processing by 60%
			currentUpstreamPressure = 0.8              // Tell predictor upstream is failing
			currentRPS = 1000.0 + (rand.Float64() * 100) // Traffic is normal!
		} else if minute == 30 {
			fmt.Printf("\n✅ [RECOVERY] Database recovered.\n")
			currentServiceRate = baseServiceRate
			currentUpstreamPressure = 0.0
			mpcActionTaken = false // Reset for next phase
		}

		// 🌀 PHASE 3: THE BLACK FRIDAY RETRY STORM [Mins 40 - 55]
		if minute >= 40 && minute < 55 {
			if minute == 40 { fmt.Printf("\n💥 [CHAOS EVENT: TRAFFIC] Massive Traffic Spike & Retry Storm!\n") }
			spikeMult := math.Exp(float64(minute-40) * 0.15)
			currentRPS = 1000.0 + (spikeMult * 500.0)
		} else if minute == 55 {
			fmt.Printf("\n✅ [RECOVERY] Traffic normalized.\n")
			mpcActionTaken = false
		}

		// 🌀 PHASE 4: NODE DEATH (OOM KILL) [Min 65]
		if minute == 65 {
			fmt.Printf("\n💥 [CHAOS EVENT: INFRASTRUCTURE] OOM Kill! 60%% of Pods Terminated Instantly!\n")
			replicas = math.Max(1.0, math.Floor(replicas * 0.4)) // Kill 60% of nodes
			mpcActionTaken = false
		}

		// --- SHARED PHYSICS ENGINE ---
		totalCapacity := replicas * currentServiceRate
		if currentRPS > totalCapacity {
			currentQueue += (currentRPS - totalCapacity) * 2.0
			currentLatency = 45.0 + (currentQueue * 0.5) + (rand.Float64() * 5.0)

			// Retry Storm Physics
			if currentLatency > slaLatency*0.8 {
				currentRetryPressure = math.Min(5.0, math.Exp((currentLatency-slaLatency*0.8)/100.0)-1.0)
				currentRPS += (currentRetryPressure * 100.0) 
			}
		} else {
			if currentQueue > 0 {
				drainRate := totalCapacity - currentRPS
				actualDrain := math.Min(drainRate, currentQueue*0.25+20.0)
				currentQueue = math.Max(0, currentQueue-actualDrain)
			}
			
			if currentQueue == 0 {
				currentLatency = 45.0 + (rand.Float64() * 5.0) 
				currentRetryPressure = 0.0
			} else {
				currentLatency = 45.0 + (currentQueue * 0.5) + (rand.Float64() * 3.0) 
				currentRetryPressure = math.Max(0.0, currentRetryPressure-0.5)
			}
			if minute < 40 || minute >= 55 {
				currentRPS = 1000.0 + rand.Float64()*100
			}
		}

		// ---------------------------------------------------------
		// PIPELINE EXECUTION
		// ---------------------------------------------------------
		point := telemetry.MetricPoint{
			ServiceID:   "payment-service",
			Timestamp:   timestamp,
			RequestRate: currentRPS,
			Latency:     telemetry.LatencyStats{Mean: currentLatency},
			QueueDepth:  int64(currentQueue),
			CPUUsage:    (currentRPS / (math.Max(1.0, replicas) * currentServiceRate)) * 100.0,
		}
		ringBuf.Append(&point)
		stats := ringBuf.SummaryStats()

		mockWindow := telemetry.ServiceWindow{
			ServiceID:       "payment-service",
			MeanRequestRate: stats.MeanReqRate,
			StdRequestRate:  stats.StdReqRate,
			MeanLatencyMs:   stats.MeanLatencyMs,
			LastRequestRate: currentRPS,
			LastQueueDepth:  currentQueue,
			MeanActiveConns: replicas,
			SampleCount:     10,
			AppliedScale:    1.0,
		}
		queueModel := queueEngine.RunQueueModel(&mockWindow, topology.GraphSnapshot{}, true)

		latencySec := currentLatency / 1000.0
		z := control.MeasurementVector{currentQueue, latencySec, currentRetryPressure, replicas * currentServiceRate, currentRPS}
		ekf.Update(z)

		u := control.ControlVector{float64(replicas) * currentServiceRate, 5000.0, 0.1}
		simCfg := control.SimConfig{BaseLatency: 0.045, MaxQueueDelay: 5.0}
		sysState := control.SystemState{SLATarget: slaLatency / 1000.0, ServiceRate: currentServiceRate}
		ekf.Predict(u, sysState, simCfg, 2.0)

		lTrend := (currentLatency - 45.0) / 45.0
		if lTrend < 0.15 { lTrend = 0.0 }
		
		bGrowth := queueModel.UtilisationTrend
		if currentRPS > (replicas * currentServiceRate) {
			bGrowth = math.Max(bGrowth, (currentRPS-(replicas*currentServiceRate))/(replicas*currentServiceRate))
		}

		anomalyIn := autopilot.AnomalyInput{
			Instability:   math.Min(1.0, currentQueue/5000.0),
			Confidence:    queueModel.Confidence,
			BacklogGrowth: bGrowth,
			LatencyTrend:  lTrend,
			RetryPressure: currentRetryPressure, 
			Oscillation:   stats.StdReqRate / stats.MeanReqRate,
		}
		anomalyState = autopilot.Classify(anomalyIn)

		// ---------------------------------------------------------
		// 🧠 PREDICTOR FORECASTING (Stores T+1 for MAPE)
		// ---------------------------------------------------------
		predictState := autopilot.CongestionState{
			Backlog:          currentQueue, 
			ArrivalMean:      currentRPS, 
			CapacityActive:   replicas,
			CapacityTarget:   targetReplicas,
			CapacityTauUp:    60.0, 
			CapacityTauDown:  120.0,
			ServiceRate:      currentServiceRate, 
			ConcurrencyLimit: 100, 
			Latency:          latencySec,
			RetryFactor:      currentRetryPressure * 100.0,
			UpstreamPressure: currentUpstreamPressure, // TOPOLOGY AWARENESS!
		}
		futureTraj := predictor.Rollout(predictState, 5)
		
		// Save T+1 prediction to validate it in the next minute loop
		forecastTracker[minute+1] = futureTraj[0].Backlog 

		// Only print state if we are in an active chaos window to keep logs clean
		isChaosWindow := (minute >= 13 && minute <= 32) || (minute >= 38 && minute <= 57) || (minute >= 64 && minute <= 70)
		if isChaosWindow {
			fmt.Printf("[Min %02d] RPS: %7.1f | Lat: %6.1fms | Q: %5.1f | Topology: %.1f | Anomaly: %-8s\n",
				minute, currentRPS, currentLatency, currentQueue, currentUpstreamPressure, anomalyState)
		}

		// ---------------------------------------------------------
		// AUTONOMOUS DECISION ENGINE TRIGGER
		// ---------------------------------------------------------
		isSevereLocal := anomalyState == autopilot.Local && currentLatency > slaLatency*0.8

		if (anomalyState == autopilot.Cascade || anomalyState == autopilot.Systemic || isSevereLocal) && !mpcActionTaken {

			maxSafeQueue := currentRPS * (slaLatency / 1000.0) 
			maxSafePodJump := math.Max(2.0, replicas * 0.20) 

			dynamicBacklogCost := 10.0 / math.Max(1.0, maxSafeQueue)
			dynamicScalingCost := 50.0 / math.Max(1.0, maxSafePodJump)

			mpcOpt := &autopilot.MPCOptimiser{
				Horizon:      10,
				Dt:           2.0,
				BacklogCost:  dynamicBacklogCost, 
				LatencyCost:  dynamicBacklogCost * 0.5,
				ScalingCost:  dynamicScalingCost, 
				SmoothCost:   dynamicScalingCost * 2.0, 
				MinCapacity:  math.Max(1.0, replicas), 
				MaxCapacity:  math.Min(50.0, replicas + maxSafePodJump), 
				Iters:        100,
				IterModifier: 1.0,
			}

			initialMPC := autopilot.MPCState{
				Backlog: currentQueue, Latency: latencySec, ArrivalMean: currentRPS,
				ServiceRate: currentServiceRate, CapacityActive: replicas, PrevBacklog: currentQueue,
			}
			var prevSeq []autopilot.MPCControl

			optimalControls, _ := mpcOpt.Optimise(initialMPC, prevSeq)
			recommendedCapacity := math.Ceil(optimalControls[0].CapacityTarget)

			fmt.Printf("   🚨 [MPC ACTION] Target Replicas: %.0f -> %.0f | Forecast Q(T+5): %.1f\n", targetReplicas, recommendedCapacity, futureTraj[4].Backlog)
			
			targetReplicas = recommendedCapacity 
			mpcActionTaken = true
		}
	}

	// ---------------------------------------------------------
	// 🏆 FINAL VALIDATION & METRICS
	// ---------------------------------------------------------
	fmt.Println("=========================================================================================")
	if anomalyTicks > 0 {
		finalMAPE := totalErrorPercentage / float64(anomalyTicks)
		fmt.Printf("📊 PREDICTOR VALIDATION: Mean Absolute Percentage Error (MAPE) = %.2f%%\n", finalMAPE)
		
		if finalMAPE > 25.0 {
			t.Fatalf("❌ FATAL: Predictor MAPE is too high (%.2f%%). Physics model is inaccurate.", finalMAPE)
		}
	}

	fmt.Println("✅ ALL MULTI-PHASE E2E PIPELINE TESTS PASSED.")
}