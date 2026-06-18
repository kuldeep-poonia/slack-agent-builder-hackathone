package modelling

import "time"

type QueueModel struct {
	ServiceID                string
	ComputedAt               time.Time
	Confidence               float64
	Hazard                   float64
	Reservoir                float64
	Concurrency              float64
	ArrivalRate              float64
	ServiceRate              float64
	Utilisation              float64
	MeanQueueLen             float64
	MeanWaitMs               float64
	AdjustedWaitMs           float64
	MeanSojournMs            float64
	BurstFactor              float64
	UtilisationTrend         float64
	SaturationHorizon        time.Duration
	NetworkSaturationHorizon time.Duration
	UpstreamPressure         float64
}

type NetworkField struct {
	Edges     map[string]*EdgeField
	Junctions []*Junction
}

type EdgeField struct {
	Cells       []Cell
	Dx          float64
	ServiceRate float64
	NoiseAmp    float64
	SourceGain  float64
}

type Cell struct {
	Rho float64
}

type Junction struct {
	In  []string
	Out []string
	R   [][]float64
}