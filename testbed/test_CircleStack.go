package main

import "github.com/cuihonglei/gobox2d/box2d"

const (
	CircleStack_e_count int = 10
)

type CircleStack struct {
	Test

	bodies [CircleStack_e_count]*box2d.Body
}

func CreateCircleStack() ITest {
	cs := &CircleStack{}
	cs.Test.init(cs)

	{
		bd := box2d.MakeBodyDef()
		ground := cs.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()
		shape.Set(box2d.Vec2{X: -40.0, Y: 0.0}, box2d.Vec2{X: 40.0, Y: 0.0})
		ground.CreateFixture2(&shape, 0.0)
	}

	{
		shape := box2d.MakeCircleShape()
		shape.Radius = 1.0

		for i := 0; i < CircleStack_e_count; i++ {

			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody
			bd.Position.Set(0.0, 4.0+3.0*float64(i))

			cs.bodies[i] = cs.world.CreateBody(&bd)

			cs.bodies[i].CreateFixture2(&shape, 1.0)

			cs.bodies[i].SetLinearVelocity(box2d.Vec2{X: 0.0, Y: -50.0})
		}
	}

	return cs
}

func (cs *CircleStack) step(settings *Settings) {

	cs.Test.step(settings)
}

var _ = RegisterTest("Stacking", "Circles", CreateCircleStack)
