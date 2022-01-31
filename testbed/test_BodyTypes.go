package main

import "github.com/cuihonglei/gobox2d/box2d"

type BodyTypes struct {
	Test

	attachment *box2d.Body
	platform   *box2d.Body
	speed      float64
}

func CreateBodyTypes() ITest {
	bt := &BodyTypes{}
	bt.Test.init(bt)

	var ground *box2d.Body
	{
		bd := box2d.MakeBodyDef()
		ground = bt.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()
		// TODO
		//shape.SetTwoSided(box2d.Vec2{-20.0, 0.0}, box2d.Vec2{20.0, 0.0})
		shape.Set(box2d.Vec2{X: -20.0, Y: 0.0}, box2d.Vec2{X: 20.0, Y: 0.0})

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape

		ground.CreateFixture(&fd)
	}

	// Define attachment
	{
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(0.0, 3.0)
		bt.attachment = bt.world.CreateBody(&bd)

		shape := box2d.MakePolygonShape()
		shape.SetAsBox(0.5, 2.0)
		bt.attachment.CreateFixture2(&shape, 2.0)
	}

	// Define platform
	{
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(-4.0, 5.0)
		bt.platform = bt.world.CreateBody(&bd)

		shape := box2d.MakePolygonShape()
		shape.SetAsOrientedBox(0.5, 4.0, box2d.Vec2{X: 4.0, Y: 0.0}, 0.5*box2d.Pi)

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Friction = 0.6
		fd.Density = 2.0
		bt.platform.CreateFixture(&fd)

		rjd := box2d.MakeRevoluteJointDef()
		rjd.Initialize(bt.attachment, bt.platform, box2d.Vec2{X: 0.0, Y: 5.0})
		rjd.MaxMotorTorque = 50.0
		rjd.EnableMotor = true
		bt.world.CreateJoint(&rjd)

		pjd := box2d.MakePrismaticJointDef()
		pjd.Initialize(ground, bt.platform, box2d.Vec2{X: 0.0, Y: 5.0}, box2d.Vec2{X: 1.0, Y: 0.0})

		pjd.MaxMotorForce = 1000.0
		pjd.EnableMotor = true
		pjd.LowerTranslation = -10.0
		pjd.UpperTranslation = 10.0
		pjd.EnableLimit = true

		bt.world.CreateJoint(&pjd)

		bt.speed = 3.0
	}

	// Create a payload
	{
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(0.0, 8.0)
		body := bt.world.CreateBody(&bd)

		shape := box2d.MakePolygonShape()
		shape.SetAsBox(0.75, 0.75)

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Friction = 0.6
		fd.Density = 2.0

		body.CreateFixture(&fd)
	}

	return bt
}

func (bt *BodyTypes) step(settings *Settings) {

	// Drive the kinematic body.
	if bt.platform.GetType() == box2d.KinematicBody {
		p := bt.platform.GetTransform().P
		v := bt.platform.GetLinearVelocity()

		if (p.X < -10.0 && v.X < 0.0) ||
			(p.X > 10.0 && v.X > 0.0) {
			v.X = -v.X
			bt.platform.SetLinearVelocity(v)
		}
	}

	bt.Test.step(settings)

	g_debugDraw.DrawString(5, bt.textLine, "Keys: (d) dynamic, (s) static, (k) kinematic")
	bt.textLine += bt.textIncrement
}

var _ = RegisterTest("Examples", "Body Types", CreateBodyTypes)
