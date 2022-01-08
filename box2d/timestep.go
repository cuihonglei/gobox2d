package box2d

// Profiling data. Times are in milliseconds.
type Profile struct {
	step          float64
	collide       float64
	solve         float64
	solveInit     float64
	solveVelocity float64
	solvePosition float64
	broadphase    float64
	solveTOI      float64
}

/// This is an internal structure.
type timeStep struct {
	dt                 float64 // time step
	inv_dt             float64 // inverse time step (0 if dt == 0).
	dtRatio            float64 // dt * inv_dt0
	velocityIterations int32
	positionIterations int32
	warmStarting       bool
}

// This is an internal structure.
type position struct {
	c Vec2
	a float64
}

/// This is an internal structure.
type velocity struct {
	v Vec2
	w float64
}

/// Solver Data
type solverData struct {
	step       timeStep
	positions  []position
	velocities []velocity
}
