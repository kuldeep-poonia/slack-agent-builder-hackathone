package control

import (
	"math"
)

type ActuatorState struct {
	TargetReplicas       int
	ReadyReplicas        float64 
	ProvisioningReplicas float64 
	TerminatingReplicas  float64 

	ScaleUpCooldown   float64
	ScaleDownCooldown float64

	QueueTarget float64
	QueueActual float64
	RetryTarget float64
	RetryActual float64
	CacheTarget float64
	CacheActual float64
	
	SafeModeTicks float64 // CORE FIX #1: Hysteresis Lockout
}

type ActuatorConfig struct {
	MinReplicas int
	MaxReplicas int
	MaxSurgeRate     float64 
	MaxScaleDownRate float64 
	WarmupTau      float64 
	TerminationTau float64 
	ScaleUpCooldownSec   float64
	ScaleDownCooldownSec float64
	ControlPlaneTau float64 
}

func DefaultActuatorConfig() ActuatorConfig {
	return ActuatorConfig{
		MinReplicas:          1,
		MaxReplicas:          1000,
		MaxSurgeRate:         20.0, 
		MaxScaleDownRate:     5.0,
		WarmupTau:            30.0,  
		TerminationTau:       15.0,  
		ScaleUpCooldownSec:   15.0,
		ScaleDownCooldownSec: 300.0,
		ControlPlaneTau:      5.0,   
	}
}

func ApplyActuatorDynamics(sys *SystemState, act *ActuatorState, cmd Bundle, cfg ActuatorConfig, dt float64) {
	requestedReplicas := localClampInt(cmd.Replicas, cfg.MinReplicas, cfg.MaxReplicas)

	nominalCapacity := float64(act.ReadyReplicas) * sys.ServiceRate
	utilizationNominal := sys.PredictedArrival / math.Max(1.0, nominalCapacity)
	
	if utilizationNominal < 0.20 && sys.QueueDepth < 10.0 {
		act.ScaleDownCooldown = 0.0 
		cfg.MaxScaleDownRate = math.Max(cfg.MaxScaleDownRate, act.ReadyReplicas*0.10) 
	} else {
		act.ScaleDownCooldown = math.Max(0.0, act.ScaleDownCooldown-dt)
	}

	act.ScaleUpCooldown = math.Max(0.0, act.ScaleUpCooldown-dt)

	if requestedReplicas > act.TargetReplicas && act.ScaleUpCooldown <= 0 {
		act.TargetReplicas = requestedReplicas
		act.ScaleUpCooldown = cfg.ScaleUpCooldownSec
	} else if requestedReplicas < act.TargetReplicas && act.ScaleDownCooldown <= 0 {
		act.TargetReplicas = requestedReplicas
		act.ScaleDownCooldown = cfg.ScaleDownCooldownSec
	}

	activeReplicas := act.ReadyReplicas + act.ProvisioningReplicas
	diff := float64(act.TargetReplicas) - activeReplicas
	
	var spawnRate, killRate float64
	if diff > 0 {
		spawnRate = math.Min(diff/dt, cfg.MaxSurgeRate)
	} else if diff < 0 {
		killRate = math.Min(-diff/dt, cfg.MaxScaleDownRate)
		actualKill := math.Min(killRate*dt, activeReplicas)
		killFromProvisioning := math.Min(actualKill, act.ProvisioningReplicas)
		killFromReady := actualKill - killFromProvisioning
		
		act.ProvisioningReplicas -= killFromProvisioning
		act.ReadyReplicas -= killFromReady
		act.TerminatingReplicas += actualKill
	}

	act.ProvisioningReplicas += spawnRate * dt
	newReady := (act.ProvisioningReplicas / math.Max(cfg.WarmupTau, 0.001)) * dt
	act.ProvisioningReplicas -= newReady
	act.ReadyReplicas += newReady

	terminationRate := act.TerminatingReplicas / math.Max(cfg.TerminationTau, 0.001)
	act.TerminatingReplicas -= terminationRate * dt

	act.ProvisioningReplicas = math.Max(act.ProvisioningReplicas, 0.0)
	act.ReadyReplicas = math.Max(act.ReadyReplicas, float64(cfg.MinReplicas))
	act.TerminatingReplicas = math.Max(act.TerminatingReplicas, 0.0)

	sys.Replicas = localMaxInt(1, int(math.Round(act.ReadyReplicas)))
	sys.CapacityVelocity = spawnRate - killRate 

	act.QueueTarget = float64(cmd.QueueLimit)
	act.RetryTarget = float64(cmd.RetryLimit)
	act.CacheTarget = math.Max(0.0, math.Min(1.0, cmd.CacheAggression))

	alpha := 1.0 - math.Exp(-dt/math.Max(cfg.ControlPlaneTau, 0.001))
	act.QueueActual = (alpha * act.QueueTarget) + ((1.0 - alpha) * act.QueueActual)
	act.RetryActual = (alpha * act.RetryTarget) + ((1.0 - alpha) * act.RetryActual)
	act.CacheActual = (alpha * act.CacheTarget) + ((1.0 - alpha) * act.CacheActual)

	sys.QueueLimit = int(math.Round(act.QueueActual))
	sys.RetryLimit = int(math.Round(act.RetryActual))
	sys.RetryPressure = act.RetryActual 
	sys.CacheAggression = act.CacheActual
}