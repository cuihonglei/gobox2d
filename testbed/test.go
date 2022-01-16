package main

import (
	"math/rand"

	"github.com/cuihonglei/gobox2d/box2d"
)

const RAND_LIMIT = 32767

/// Random number in range [-1,1]
func RandomFloat() float64 {
	// TODO
	r := rand.Float64()
	r = 2.0*r - 1.0
	return r
}

/// Random floating point number in range [lo, hi]
func RandomFloat2(lo float64, hi float64) float64 {
	// TODO
	r := rand.Float64()
	r = (hi-lo)*r + lo
	return r
}

type ITest interface {
	step(*Settings)

	BeginContact(contact box2d.IContact)
	EndContact(contact box2d.IContact)
	PreSolve(contact box2d.IContact, oldManifold *box2d.Manifold)
	PostSolve(contact box2d.IContact, impulse *box2d.ContactImpulse)
}

const k_maxContactPoints = 2048

// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
type DestructionListener struct {
	test ITest
}

func (dl *DestructionListener) SayGoodbyeFixture(fixture *box2d.Fixture) {

}

func (dl *DestructionListener) SayGoodbyeJoint(joint box2d.IJoint) {

}

type ContactPoint struct {
	fixtureA       *box2d.Fixture
	fixtureB       *box2d.Fixture
	normal         box2d.Vec2
	position       box2d.Vec2
	state          box2d.PointState
	normalImpulse  float64
	tangentImpulse float64
	separation     float64
}

type Test struct {
	groundBody          *box2d.Body
	worldAABB           box2d.AABB
	points              [k_maxContactPoints]ContactPoint
	pointCount          int
	destructionListener DestructionListener
	textLine            int
	world               *box2d.World
	bomb                *box2d.Body
	mouseJoint          *box2d.MouseJoint
	bombSpawnPoint      box2d.Vec2
	bombSpawning        bool
	mouseWorld          box2d.Vec2
	stepCount           int
	textIncrement       int
	maxProfile          box2d.Profile
	totalProfile        box2d.Profile
}

func (t *Test) init(test ITest) {

	gravity := box2d.Vec2{}
	gravity.Set(0.0, -10.0)
	t.world = box2d.NewWorld(gravity)
	t.bomb = nil
	t.textLine = 30
	t.textIncrement = 13
	t.mouseJoint = nil
	t.pointCount = 0

	t.destructionListener.test = test
	t.world.SetDestructionListener(&t.destructionListener)
	t.world.SetContactListener(test)
	t.world.SetDebugDraw(&g_debugDraw)

	t.bombSpawning = false

	t.stepCount = 0

	bodyDef := box2d.MakeBodyDef()
	t.groundBody = t.world.CreateBody(&bodyDef)
}

// Callbacks for derived classes.
func (t *Test) BeginContact(contact box2d.IContact) {

}

func (t *Test) EndContact(contact box2d.IContact) {

}

func (t *Test) PreSolve(contact box2d.IContact, oldManifold *box2d.Manifold) {

}

func (t *Test) PostSolve(contact box2d.IContact, impulse *box2d.ContactImpulse) {

}

func (t *Test) step(settings *Settings) {

	b2i := func(b bool) uint {
		if b {
			return 1
		}
		return 0
	}

	timeStep := 0.0
	if settings.hertz > 0.0 {
		timeStep = 1.0 / settings.hertz
	}

	if settings.pause {
		if settings.singleStep {
			settings.singleStep = false
		} else {
			timeStep = 0.0
		}

		g_debugDraw.DrawString(5, t.textLine, "****PAUSED****")
		t.textLine += t.textIncrement
	}

	flags := uint(0)
	flags += b2i(settings.drawShapes) * box2d.Draw_e_shapeBit
	flags += b2i(settings.drawJoints) * box2d.Draw_e_jointBit
	flags += b2i(settings.drawAABBs) * box2d.Draw_e_aabbBit
	flags += b2i(settings.drawCOMs) * box2d.Draw_e_centerOfMassBit
	g_debugDraw.SetFlags(flags)

	t.world.SetAllowSleeping(settings.enableSleep)
	t.world.SetWarmStarting(settings.enableWarmStarting)
	t.world.SetContinuousPhysics(settings.enableContinuous)
	t.world.SetSubStepping(settings.enableSubStepping)

	t.pointCount = 0

	t.world.Step(timeStep, settings.velocityIterations, settings.positionIterations)

	t.world.DebugDraw()
	g_debugDraw.flush()

	if timeStep > 0.0 {
		t.stepCount += 1
	}

	if settings.drawStats {
		// TODO
	}

	// Track maximum profile times
	{
		// TODO
	}

	if settings.drawProfile {
		// TODO
	}

	if t.bombSpawning {
		// TODO
	}

	if settings.drawContactPoints {
		// TODO
	}
}

func (t *Test) shiftOrigin(newOrigin box2d.Vec2) {
	// TODO
	//t.world.ShiftOrigin(newOrigin)
}

const MAX_TESTS = 256

var g_testEntries [MAX_TESTS]TestEntry
var g_testCount = 0

type TestEntry struct {
	category  string
	name      string
	createFcn TestCreateFcn
}

type TestCreateFcn func() ITest

func RegisterTest(category string, name string, fcn TestCreateFcn) int {

	index := g_testCount
	if index < MAX_TESTS {
		g_testEntries[index] = TestEntry{category, name, fcn}
		g_testCount += 1
		return index
	}

	return -1
}
