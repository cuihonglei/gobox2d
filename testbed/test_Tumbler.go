package main

import "github.com/cuihonglei/gobox2d/box2d"

const (
	Tumbler_e_count int = 800
)

type Tumbler struct {
	Test

	joint *box2d.RevoluteJoint
	count int
}

func CreateTumbler() ITest {
	t := &Tumbler{}
	t.Test.init(t)

	var ground *box2d.Body
	{
		bd := box2d.MakeBodyDef()
		ground = t.world.CreateBody(&bd)
	}

	{
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.AllowSleep = false
		bd.Position.Set(0.0, 10.0)
		body := t.world.CreateBody(&bd)

		shape := box2d.MakePolygonShape()
		shape.SetAsOrientedBox(0.5, 10.0, box2d.MakeVec2(10.0, 0.0), 0.0)
		body.CreateFixture2(&shape, 5.0)
		shape.SetAsOrientedBox(0.5, 10.0, box2d.MakeVec2(-10.0, 0.0), 0.0)
		body.CreateFixture2(&shape, 5.0)
		shape.SetAsOrientedBox(10.0, 0.5, box2d.MakeVec2(0.0, 10.0), 0.0)
		body.CreateFixture2(&shape, 5.0)
		shape.SetAsOrientedBox(10.0, 0.5, box2d.MakeVec2(0.0, -10.0), 0.0)
		body.CreateFixture2(&shape, 5.0)

		jd := box2d.MakeRevoluteJointDef()
		jd.BodyA = ground
		jd.BodyB = body
		jd.LocalAnchorA.Set(0.0, 10.0)
		jd.LocalAnchorB.Set(0.0, 0.0)
		jd.ReferenceAngle = 0.0
		jd.MotorSpeed = 0.05 * box2d.Pi
		jd.MaxMotorTorque = 1e8
		jd.EnableMotor = true
		t.joint = t.world.CreateJoint(&jd).(*box2d.RevoluteJoint)
	}

	t.count = 0

	return t
}

func (t *Tumbler) step(settings *Settings) {
	t.Test.step(settings)

	if t.count < Tumbler_e_count {
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(0.0, 10.0)
		body := t.world.CreateBody(&bd)

		shape := box2d.MakePolygonShape()
		shape.SetAsBox(0.125, 0.125)
		body.CreateFixture2(&shape, 1.0)

		t.count += 1
	}
}

var _ = RegisterTest("Benchmark", "Tumbler", CreateTumbler)
