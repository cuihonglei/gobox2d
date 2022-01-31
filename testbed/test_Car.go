package main

import (
	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/glfw/v3.3/glfw"
)

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

		hs := [10]float64{0.25, 1.0, 4.0, 0.0, 0.0, -1.0, -2.0, -2.0, -1.25, 0.0}

		x, y1, dx := 20.0, 0.0, 5.0

		for i := 0; i < 10; i++ {
			y2 := hs[i]
			// TODO
			//shape.SetTwoSided(box2d.MakeVec2(x, y1), box2d.MakeVec2(x+dx, y2))
			shape.Set(box2d.MakeVec2(x, y1), box2d.MakeVec2(x+dx, y2))
			ground.CreateFixture(&fd)
			y1 = y2
			x += dx
		}

		for i := 0; i < 10; i++ {
			y2 := hs[i]
			// TODO
			//shape.SetTwoSided(box2d.MakeVec2(x, y1), box2d.MakeVec2(x+dx, y2))
			shape.Set(box2d.MakeVec2(x, y1), box2d.MakeVec2(x+dx, y2))
			ground.CreateFixture(&fd)
			y1 = y2
			x += dx
		}

		shape.Set(box2d.MakeVec2(x, 0.0), box2d.MakeVec2(x+40.0, 0.0))
		ground.CreateFixture(&fd)

		x += 80.0
		shape.Set(box2d.MakeVec2(x, 0.0), box2d.MakeVec2(x+40.0, 0.0))
		ground.CreateFixture(&fd)

		x += 40.0
		shape.Set(box2d.MakeVec2(x, 0.0), box2d.MakeVec2(x+10.0, 5.0))
		ground.CreateFixture(&fd)

		x += 20.0
		shape.Set(box2d.MakeVec2(x, 0.0), box2d.MakeVec2(x+40.0, 0.0))
		ground.CreateFixture(&fd)

		x += 40.0
		shape.Set(box2d.MakeVec2(x, 0.0), box2d.MakeVec2(x, 20.0))
		ground.CreateFixture(&fd)
	}

	// Teeter
	{
		bd := box2d.MakeBodyDef()
		bd.Position.Set(140.0, 1.0)
		bd.Type = box2d.DynamicBody
		body := c.world.CreateBody(&bd)

		box := box2d.MakePolygonShape()
		box.SetAsBox(10.0, 0.25)
		body.CreateFixture2(&box, 1.0)

		jd := box2d.MakeRevoluteJointDef()
		jd.Initialize(ground, body, body.GetPosition())
		jd.LowerAngle = -8.0 * box2d.Pi / 180.0
		jd.UpperAngle = 8.0 * box2d.Pi / 180.0
		jd.EnableLimit = true
		c.world.CreateJoint(&jd)

		// TODO
		//body.ApplyAngularImpulse(100.0, true)
		body.ApplyAngularImpulse(100.0)
	}

	// Bridge
	{
		N := 20
		shape := box2d.MakePolygonShape()
		shape.SetAsBox(1.0, 0.125)

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Density = 1.0
		fd.Friction = 0.6

		jd := box2d.MakeRevoluteJointDef()

		prevBody := ground
		for i := 0; i < N; i++ {

			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody
			bd.Position.Set(161.0+2.0*float64(i), -0.125)
			body := c.world.CreateBody(&bd)
			body.CreateFixture(&fd)

			anchor := box2d.MakeVec2(160.0+2.0*float64(i), -0.125)
			jd.Initialize(prevBody, body, anchor)
			c.world.CreateJoint(&jd)

			prevBody = body
		}

		anchor := box2d.MakeVec2(160.0+2.0*float64(N), -0.125)
		jd.Initialize(prevBody, ground, anchor)
		c.world.CreateJoint(&jd)
	}

	// Boxes
	{
		box := box2d.MakePolygonShape()
		box.SetAsBox(0.5, 0.5)

		var body *box2d.Body
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody

		bd.Position.Set(230.0, 0.5)
		body = c.world.CreateBody(&bd)
		body.CreateFixture2(&box, 0.5)

		bd.Position.Set(230.0, 1.5)
		body = c.world.CreateBody(&bd)
		body.CreateFixture2(&box, 0.5)

		bd.Position.Set(230.0, 2.5)
		body = c.world.CreateBody(&bd)
		body.CreateFixture2(&box, 0.5)

		bd.Position.Set(230.0, 3.5)
		body = c.world.CreateBody(&bd)
		body.CreateFixture2(&box, 0.5)

		bd.Position.Set(230.0, 4.5)
		body = c.world.CreateBody(&bd)
		body.CreateFixture2(&box, 0.5)
	}

	// Car
	{
		chassis := box2d.MakePolygonShape()
		vertices := make([]box2d.Vec2, 6)
		vertices[0].Set(-1.5, -0.5)
		vertices[1].Set(1.5, -0.5)
		vertices[2].Set(1.5, 0.0)
		vertices[3].Set(0.0, 0.9)
		vertices[4].Set(-1.15, 0.9)
		vertices[5].Set(-1.5, 0.2)
		chassis.Set(vertices)

		circle := box2d.MakeCircleShape()
		circle.Radius = 0.4

		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(0.0, 1.0)
		c.car = c.world.CreateBody(&bd)
		c.car.CreateFixture2(&chassis, 1.0)

		fd := box2d.MakeFixtureDef()
		fd.Shape = &circle
		fd.Density = 1.0
		fd.Friction = 0.9

		bd.Position.Set(-1.0, 0.35)
		c.wheel1 = c.world.CreateBody(&bd)
		c.wheel1.CreateFixture(&fd)

		bd.Position.Set(1.0, 0.4)
		c.wheel2 = c.world.CreateBody(&bd)
		c.wheel2.CreateFixture(&fd)

		jd := box2d.MakeWheelJointDef()
		axis := box2d.MakeVec2(0.0, 1.0)

		// TODO
		//mass1 := c.wheel1.GetMass()
		//mass2 := c.wheel2.GetMass()

		//hertz := 4.0
		//dampingRatio := 0.7
		//omega := 2.0 * box2d.Pi * hertz

		jd.Initialize(c.car, c.wheel1, c.wheel1.GetPosition(), axis)
		jd.MotorSpeed = 0.0
		jd.MaxMotorTorque = 20.0
		jd.EnableMotor = true
		// TODO
		//jd.Stiffness = mass1 * omega * omega
		//jd.Damping = 2.0 * mass1 * dampingRatio * omega
		//jd.LowerTranslation = -0.25
		//jd.UpperTranslation = 0.25
		//jd.EnableLimit = true
		c.spring1 = c.world.CreateJoint(&jd).(*box2d.WheelJoint)

		jd.Initialize(c.car, c.wheel2, c.wheel2.GetPosition(), axis)
		jd.MotorSpeed = 0.0
		jd.MaxMotorTorque = 10.0
		jd.EnableMotor = false
		// TODO
		//jd.Stiffness = mass1 * omega * omega
		//jd.Damping = 2.0 * mass1 * dampingRatio * omega
		//jd.LowerTranslation = -0.25
		//jd.UpperTranslation = 0.25
		//jd.EnableLimit = true
		c.spring2 = c.world.CreateJoint(&jd).(*box2d.WheelJoint)
	}

	return c
}

func (c *Car) keyboard(key glfw.Key) {

	switch key {
	case glfw.KeyA:
		c.spring1.SetMotorSpeed(c.speed)

	case glfw.KeyS:
		c.spring1.SetMotorSpeed(0.0)

	case glfw.KeyD:
		c.spring1.SetMotorSpeed(-c.speed)
	}
}

func (c *Car) step(settings *Settings) {

	g_debugDraw.DrawString(5, c.textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e")
	c.textLine += c.textIncrement

	g_camera.center.X = c.car.GetPosition().X
	c.Test.step(settings)
}

var _ = RegisterTest("Examples", "Car", CreateCar)
