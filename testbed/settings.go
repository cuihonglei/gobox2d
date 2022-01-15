package main

type Settings struct {
	testIndex           int
	windowWidth         int
	windowHeight        int
	hertz               float64
	velocityIterations  int
	positionIterations  int
	drawShapes          bool
	drawJoints          bool
	drawAABBs           bool
	drawContactPoints   bool
	drawContactNormals  bool
	drawContactImpulse  bool
	drawFrictionImpulse bool
	drawCOMs            bool
	drawStats           bool
	drawProfile         bool
	enableWarmStarting  bool
	enableContinuous    bool
	enableSubStepping   bool
	enableSleep         bool
	pause               bool
	singleStep          bool
}

func MakeSettings() Settings {
	s := Settings{}
	s.reset()
	return s
}

func (s *Settings) reset() {
	s.testIndex = 0
	s.windowWidth = 1600
	s.windowHeight = 900
	s.hertz = 60.0
	s.velocityIterations = 8
	s.positionIterations = 3
	s.drawShapes = true
	s.drawJoints = true
	s.drawAABBs = false
	s.drawContactPoints = false
	s.drawContactNormals = false
	s.drawContactImpulse = false
	s.drawFrictionImpulse = false
	s.drawCOMs = false
	s.drawStats = false
	s.drawProfile = false
	s.enableWarmStarting = true
	s.enableContinuous = true
	s.enableSubStepping = false
	s.enableSleep = true
	s.pause = false
	s.singleStep = false
}

func (s *Settings) save() {
	// TODO
}

func (s *Settings) load() {
	// TODO
}
