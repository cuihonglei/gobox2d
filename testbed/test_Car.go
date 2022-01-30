package main

import "github.com/cuihonglei/gobox2d/box2d"

type Car struct {
	Test

	car    *box2d.Body
	wheel1 *box2d.Body
	wheel2 *box2d.Body

	speed   float64
	spring1 *box2d.WheelJoint
	spring2 *box2d.WheelJoint
}

func CreateCar() ITest {
	c := &Car{}
	c.Test.init(c)

	c.speed = 50.0

	var ground *box2d.Body
	{
		bd := box2d.MakeBodyDef()
		ground = c.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Density = 0.0
		fd.Friction = 0.6

		// TODO
		//shape.SetTwoSided(box2d.MakeVec2(-20.0, 0.0), box2d.MakeVec2(20.0, 0.0))
		shape.Set(box2d.MakeVec2(-20.0, 0.0), box2d.MakeVec2(20.0, 0.0))
		ground.CreateFixture(&fd)
	}

	// Car
	{

	}

	return c
}

func (c *Car) step(settings *Settings) {

	g_debugDraw.DrawString(5, c.textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e")
	c.textLine += c.textIncrement

	//g_camera.center.X = c.car.GetPosition().X
	c.Test.step(settings)
}

var _ = RegisterTest("Examples", "Car", CreateCar)
