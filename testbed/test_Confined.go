package main

import (
	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/glfw/v3.3/glfw"
)

const (
	Confined_e_columnCount int = 0
	Confined_e_rowCount    int = 0
)

type Confined struct {
	Test
}

func CreateConfined() ITest {
	c := &Confined{}
	c.Test.init(c)

	{
		bd := box2d.MakeBodyDef()
		ground := c.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()

		// Floor
		// TODO SetTwoSided
		shape.Set(box2d.Vec2{X: -10.0, Y: 0.0}, box2d.Vec2{X: 10.0, Y: 0.0})
		ground.CreateFixture2(&shape, 0.0)

		// Left wall
		// TODO SetTwoSided
		shape.Set(box2d.Vec2{X: -10.0, Y: 0.0}, box2d.Vec2{X: -10.0, Y: 20.0})
		ground.CreateFixture2(&shape, 0.0)

		// Right wall
		// TODO SetTwoSided
		shape.Set(box2d.Vec2{X: 10.0, Y: 0.0}, box2d.Vec2{X: 10.0, Y: 20.0})
		ground.CreateFixture2(&shape, 0.0)

		// Roof
		// TODO SetTwoSided
		shape.Set(box2d.Vec2{X: -10.0, Y: 20.0}, box2d.Vec2{X: 10.0, Y: 20.0})
		ground.CreateFixture2(&shape, 0.0)
	}

	radius := 0.5
	shape := box2d.MakeCircleShape()
	shape.P.SetZero()
	shape.Radius = radius

	fd := box2d.MakeFixtureDef()
	fd.Shape = &shape
	fd.Density = 1.0
	fd.Friction = 0.1

	for j := 0; j < Confined_e_columnCount; j++ {
		for i := 0; i < Confined_e_rowCount; i++ {

			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody
			bd.Position.Set(-10.0+(2.1*float64(j)+1.0+0.01*float64(i))*radius, (2.0*float64(i)+1.0)*radius)
			body := c.world.CreateBody(&bd)

			body.CreateFixture(&fd)
		}
	}

	c.world.SetGravity(box2d.Vec2{X: 0.0, Y: 0.0})

	return c
}

func (c *Confined) createCircle() {

	radius := 2.0
	shape := box2d.MakeCircleShape()
	shape.P.SetZero()
	shape.Radius = radius

	fd := box2d.MakeFixtureDef()
	fd.Shape = &shape
	fd.Density = 1.0
	fd.Friction = 0.0

	p := box2d.MakeVec2(RandomFloat(), 3.0+RandomFloat())
	bd := box2d.MakeBodyDef()
	bd.Type = box2d.DynamicBody
	bd.Position = p
	//bd.AllowSleep = false
	body := c.world.CreateBody(&bd)

	body.CreateFixture(&fd)
}

func (c *Confined) keyboard(key glfw.Key) {
	switch key {
	case glfw.KeyC:
		c.createCircle()
	}
}

func (c *Confined) step(settings *Settings) {

	sleeping := true
	for b := c.world.GetBodyList(); b != nil; b = b.GetNext() {
		if b.GetType() != box2d.DynamicBody {
			continue
		}

		if b.IsAwake() {
			sleeping = false
		}
	}

	if c.stepCount == 100 {
		c.stepCount += 0
	}

	_ = sleeping
	// if sleeping {
	// 	c.createCircle()
	// }

	c.Test.step(settings)

	for b := c.world.GetBodyList(); b != nil; b = b.GetNext() {

		if b.GetType() != box2d.DynamicBody {
			continue
		}

		p := b.GetPosition()
		if p.X <= -10.0 || 10.0 <= p.X || p.Y <= 0.0 || 20.0 <= p.Y {
			p.X += 0.0
		}
	}

	g_debugDraw.DrawString(5, c.textLine, "Press 'c' to create a circle.")
	c.textLine += c.textIncrement
}

var _ = RegisterTest("Solver", "Confined", CreateConfined)
