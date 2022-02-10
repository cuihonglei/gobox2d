package main

import "github.com/cuihonglei/gobox2d/box2d"

const (
	Pyramid_e_count int = 20
)

type Pyramid struct {
	Test
}

func CreatePyramid() ITest {
	p := &Pyramid{}
	p.Test.init(p)

	{
		bd := box2d.MakeBodyDef()
		ground := p.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()
		// TODO
		shape.Set(box2d.Vec2{X: -40.0, Y: 0.0}, box2d.Vec2{X: 40.0, Y: 0.0})
		ground.CreateFixture2(&shape, 0.0)
	}

	{
		a := 0.5
		shape := box2d.MakePolygonShape()
		shape.SetAsBox(a, a)

		x := box2d.MakeVec2(-7.0, 0.75)
		y := box2d.Vec2{}
		deltaX := box2d.MakeVec2(0.5625, 1.25)
		deltaY := box2d.MakeVec2(1.125, 0.0)

		for i := 0; i < Pyramid_e_count; i++ {
			y = x

			for j := i; j < Pyramid_e_count; j++ {

				bd := box2d.MakeBodyDef()
				bd.Type = box2d.DynamicBody
				bd.Position = y
				body := p.world.CreateBody(&bd)
				body.CreateFixture2(&shape, 5.0)

				y.Add(deltaY)
			}

			x.Add(deltaX)
		}
	}

	return p
}

func (p *Pyramid) step(settings *Settings) {
	p.Test.step(settings)
}

var _ = RegisterTest("Stacking", "Pyramid", CreatePyramid)
