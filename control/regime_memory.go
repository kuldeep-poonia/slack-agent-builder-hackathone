package control

import (
	"math"
)

type ControlRegime int

const (
	RegimeCalm ControlRegime = iota
	RegimeStressed
	RegimeUnstable
)

type RegimeConfig struct {
	EWMAAlpha        float64
	HistorySize      int
	BaseUtilStress   float64
	BaseRiskUnstable float64
	HysteresisMargin float64
}

func DefaultRegimeConfig() RegimeConfig {
	return RegimeConfig{
		EWMAAlpha:        0.10, // Faster memory decay allows risk to drop dynamically when system heals
		HistorySize:      64,
		BaseUtilStress:   0.80,
		BaseRiskUnstable: 100.0,
		HysteresisMargin: 0.05,
	}
}

type RegimeMemory struct {
	UtilEWMA          float64
	RiskEWMA          float64 
	CostEWMA          float64 
	GainScheduledRisk float64 
	Regime            ControlRegime

	// Legacy Support Fields (Crucial: These must exist so compact.go can compile)
	CostTrendEWMA   float64
	OscillationEWMA float64
	LastAction      Bundle
}

func NewRegimeMemory(cfg RegimeConfig) *RegimeMemory {
	return &RegimeMemory{
		Regime:            RegimeCalm,
		GainScheduledRisk: 0.0,
		LastAction:        Bundle{},
	}
}

// Update applies a strict Exponential Moving Average to current risk signals.
// This replaces the old infinite accumulator (RunningRiskSum) from the legacy system,
// permanently curing the "Monotonic Risk Saturation" bug.
func (r *RegimeMemory) Update(s SystemState, sla float64, cost float64, cfg RegimeConfig) {
	alpha := cfg.EWMAAlpha
	
	r.UtilEWMA = (alpha * s.Utilisation) + ((1.0 - alpha) * r.UtilEWMA)
	r.RiskEWMA = (alpha * s.Risk) + ((1.0 - alpha) * r.RiskEWMA) 
	r.CostEWMA = (alpha * cost) + ((1.0 - alpha) * r.CostEWMA)

	if r.RiskEWMA > cfg.BaseRiskUnstable {
		r.Regime = RegimeUnstable
	} else if r.UtilEWMA > cfg.BaseUtilStress {
		r.Regime = RegimeStressed
	} else {
		r.Regime = RegimeCalm
	}

	r.GainScheduledRisk = math.Max(0.0, r.RiskEWMA)
}