package main

import "github.com/cuihonglei/gobox2d/box2d"

const (
	Bridge_e_count int = 30
)

type Bridge struct {
	Test

	middle *box2d.Body
}

func CreateBridge() ITest {
	b := &Bridge{}
	b.Test.init(b)

	var ground *box2d.Body
	{
		bd := box2d.MakeBodyDef()
		ground = b.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()
		//TODO
		//shape.SetTwoSided(box2d.Vec2{X: -40.0, Y: 0.0}, box2d.Vec2{X: 40.0, Y: 0.0})
		shape.Set(box2d.Vec2{X: -40.0, Y: 0.0}, box2d.Vec2{X: 40.0, Y: 0.0})
		ground.CreateFixture2(&shape, 0.0)
	}

	{
		shape := box2d.MakePolygonShape()
		shape.SetAsBox(0.5, 0.125)

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		fd.Friction = 0.2

		jd := box2d.MakeRevoluteJointDef()

		prevBody := ground
		for i := 0; i < Bridge_e_count; i++ {
			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody
			bd.Position.Set(-14.5+1.0*float64(i), 5.0)
			body := b.world.CreateBody(&bd)
			body.CreateFixture(&fd)

			anchor := box2d.MakeVec2(-15.0+1.0*float64(i), 5.0)
			jd.Initialize(prevBody, body, anchor)
			b.world.CreateJoint(&jd)

			if i == (Bridge_e_count >> 1) {
				b.middle = body
			}
			prevBody = body
		}

		anchor := box2d.MakeVec2(-15.0+1.0*float64(Bridge_e_count), 5.0)
		jd.Initialize(prevBody, ground, anchor)
		b.world.CreateJoint(&jd)
	}

	for i := 0; i < 2; i++ {

		var vertices [3]box2d.Vec2
		vertices[0].Set(-0.5, 0.0)
		vertices[1].Set(0.5, 0.0)
		vertices[2].Set(0.0, 1.5)

		shape := box2d.MakePolygonShape()
		shape.Set(vertices[:])

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Density = 1.0

		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(-8.0+8.0*float64(i), 12.0)
		body := b.world.CreateBody(&bd)
		body.CreateFixture(&fd)
	}

	for i := 0; i < 3; i++ {

		shape := box2d.MakeCircleShape()
		shape.Radius = 0.5

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Density = 1.0

		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(-6.0+6.0*float64(i), 10.0)
		body := b.world.CreateBody(&bd)
		body.CreateFixture(&fd)
	}

	return b
}

var _ = RegisterTest("Joints", "Bridge", CreateBridge)
