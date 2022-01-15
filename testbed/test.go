package main

import (
	"math/rand"

	"github.com/cuihonglei/gobox2d/box2d"
)

const MAX_TESTS = 256

var g_testEntries [MAX_TESTS]TestEntry
var g_testCount = 0

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

type ITest interface {
	step(*Settings)
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
	t.world.SetContactListener(t)
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

}
