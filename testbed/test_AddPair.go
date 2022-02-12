package main

import "github.com/cuihonglei/gobox2d/box2d"

type AddPair struct {
	Test
}

func CreateAddPair() ITest {
	ap := &AddPair{}
	ap.Test.init(ap)

	ap.world.SetGravity(box2d.MakeVec2(0.0, 0.0))
	{
		shape := box2d.MakeCircleShape()
		shape.P.SetZero()
		shape.Radius = 0.1

		minX := -6.0
		maxX := 0.0
		minY := 4.0
		maxY := 6.0

		for i := 0; i < 400; i++ {
			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody
			bd.Position = box2d.MakeVec2(RandomFloat2(minX, maxX), RandomFloat2(minY, maxY))
			body := ap.world.CreateBody(&bd)
			body.CreateFixture2(&shape, 0.01)
		}
	}

	{
		shape := box2d.MakePolygonShape()
		shape.SetAsBox(1.5, 1.5)
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(-40.0, 5.0)
		bd.Bullet = true
		body := ap.world.CreateBody(&bd)
		body.CreateFixture2(&shape, 1.0)
		body.SetLinearVelocity(box2d.MakeVec2(10.0, 0.0))
	}

	return ap
}

var _ = RegisterTest("Benchmark", "Add Pair", CreateAddPair)
