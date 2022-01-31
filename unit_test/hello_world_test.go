package main

import (
	"fmt"
	"testing"

	"github.com/cuihonglei/gobox2d/box2d"
)

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box2D is meant to be used
// with your rendering engine in your game engine.
func Test_hello_world(t *testing.T) {

	// Define the gravity vector.
	gravity := box2d.MakeVec2(0.0, -10.0)

	// Construct a world object, which will hold and simulate the rigid bodies.
	world := box2d.MakeWorld(gravity)

	// Define the ground body.
	groundBodyDef := box2d.MakeBodyDef()
	groundBodyDef.Position.Set(0.0, -10.0)

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	groundBody := world.CreateBody(&groundBodyDef)

	// Define the ground box shape.
	groundBox := box2d.MakePolygonShape()

	// The extents are the half-widths of the box.
	groundBox.SetAsBox(50.0, 10.0)

	// Add the ground fixture to the ground body.
	groundBody.CreateFixture2(&groundBox, 0.0)

	// Define the dynamic body. We set its position and call the body factory.
	bodyDef := box2d.MakeBodyDef()
	bodyDef.Type = box2d.DynamicBody
	bodyDef.Position.Set(0.0, 4.0)
	body := world.CreateBody(&bodyDef)

	// Define another box shape for our dynamic body.
	dynamicBox := box2d.MakePolygonShape()
	dynamicBox.SetAsBox(1.0, 1.0)

	// Define the dynamic body fixture.
	fixtureDef := box2d.MakeFixtureDef()
	fixtureDef.Shape = &dynamicBox

	// Set the box density to be non-zero, so it will be dynamic.
	fixtureDef.Density = 1.0

	// Override the default friction.
	fixtureDef.Friction = 0.3

	// Add the shape to the body.
	body.CreateFixture(&fixtureDef)

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	timeStep := 1.0 / 60.0
	velocityIterations := 6
	positionIterations := 2

	position := body.GetPosition()
	angle := body.GetAngle()

	// This is our little game loop.
	for i := 0; i < 60; i++ {
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		world.Step(timeStep, velocityIterations, positionIterations)

		// Now print the position and angle of the body.
		position = body.GetPosition()
		angle = body.GetAngle()

		fmt.Printf("%4.2f %4.2f %4.2f\n", position.X, position.Y, angle)
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned pointers, so be careful about your world management.

	if !(box2d.AbsF(position.X) < 0.01) {
		t.Fail()
	}
	if !(box2d.AbsF(position.Y-1.01) < 0.01) {
		t.Fail()
	}
	if !(box2d.AbsF(angle) < 0.01) {
		t.Fail()
	}
}
