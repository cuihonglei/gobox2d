package main

import "github.com/cuihonglei/gobox2d/box2d"

type Restitution struct {
	Test
}

func CreateRestitution() ITest {
	r := &Restitution{}
	r.Test.init(r)

	threshold := 10.0

	{
		bd := box2d.MakeBodyDef()
		ground := r.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()
		//TODO
		//shape.SetTwoSided(box2d.Vec2{-40.0, 0.0}, box2d.Vec2{40.0, 0.0})
		shape.Set(box2d.Vec2{X: -40.0, Y: 0.0}, box2d.Vec2{X: 40.0, Y: 0.0})

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		// TODO
		_ = threshold
		//fd.RestitutionThreshold = threshold
		ground.CreateFixture2(&shape, 0.0)
	}

	{
		shape := box2d.MakeCircleShape()
		shape.Radius = 1.0

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Density = 1.0

		restitution := [7]float64{0.0, 0.1, 0.3, 0.5, 0.75, 0.9, 1.0}

		for i := 0; i < 7; i++ {

			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody
			bd.Position.Set(-10.0+3.0*float64(i), 20.0)

			body := r.world.CreateBody(&bd)

			fd.Restitution = restitution[i]
			// TODO
			//fd.RestitutionThreshold = threshold
			body.CreateFixture(&fd)
		}
	}

	return r
}

var _ = RegisterTest("Forces", "Restitution", CreateRestitution)
