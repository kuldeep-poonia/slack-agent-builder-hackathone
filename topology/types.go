package topology

type GraphSnapshot struct {
	Nodes []Node
	Edges []Edge
}

type Node struct {
	ServiceID string
}

type Edge struct {
	Source string
	Target string
	
	// --- MISSING FIELD ADDED BELOW ---
	Weight float64 
}