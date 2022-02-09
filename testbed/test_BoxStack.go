package main

import (
	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/glfw/v3.3/glfw"
)

const (
	BoxStack_e_columnCount int = 1
	BoxStack_e_rowCount    int = 15
)

type BoxStack struct {
	Test

	bullet  *box2d.Body
	bodies  [BoxStack_e_columnCount * BoxStack_e_rowCount]*box2d.Body
	indices [BoxStack_e_columnCount * BoxStack_e_rowCount]int
}

func CreateBoxStack() ITest {
	bs := &BoxStack{}
	bs.Test.init(bs)

	{
		bd := box2d.MakeBodyDef()
		ground := bs.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()
		//TODO
		//shape.SetTwoSided(box2d.Vec2{-40.0, 0.0}, box2d.Vec2{40.0, 0.0})
		shape.Set(box2d.Vec2{X: -40.0, Y: 0.0}, box2d.Vec2{X: 40.0, Y: 0.0})
		ground.CreateFixture2(&shape, 0.0)

		//shape.SetTwoSided(box2d.Vec2{-40.0, 0.0}, box2d.Vec2{40.0, 0.0})
		shape.Set(box2d.Vec2{X: 20.0, Y: 0.0}, box2d.Vec2{X: 20.0, Y: 20.0})
		ground.CreateFixture2(&shape, 0.0)
	}

	xs := [5]float64{0.0, -10.0, -5.0, 5.0, 10.0}

	for j := 0; j < BoxStack_e_columnCount; j++ {

		shape := box2d.MakePolygonShape()
		shape.SetAsBox(0.5, 0.5)

		fd := box2d.MakeFixtureDef()
		fd.Shape = &shape
		fd.Density = 1.0
		fd.Friction = 0.3

		for i := 0; i < BoxStack_e_rowCount; i++ {

			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody

			n := j*BoxStack_e_rowCount + i
			bs.indices[n] = n
			bd.UserData = n

			x := 0.0
			//float x = RandomFloat(-0.02f, 0.02f);
			//float x = i % 2 == 0 ? -0.01f : 0.01f;
			bd.Position.Set(xs[j]+x, 0.55+1.0*float64(i))
			body := bs.world.CreateBody(&bd)

			bs.bodies[n] = body

			body.CreateFixture(&fd)
		}
	}

	bs.bullet = nil

	return bs
}

func (bs *BoxStack) keyboard(key glfw.Key) {

	switch key {
	case glfw.KeyComma:
		if bs.bullet != nil {
			bs.world.DestroyBody(bs.bullet)
			bs.bullet = nil
		}

		{
			shape := box2d.MakeCircleShape()
			shape.Radius = 0.25

			fd := box2d.MakeFixtureDef()
			fd.Shape = &shape
			fd.Density = 20.0
			fd.Restitution = 0.05

			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody
			bd.Bullet = true
			bd.Position.Set(-31.0, 5.0)

			bs.bullet = bs.world.CreateBody(&bd)
			bs.bullet.CreateFixture(&fd)

			bs.bullet.SetLinearVelocity(box2d.Vec2{X: 400.0, Y: 0.0})
		}

	case glfw.KeyB:
		// TODO
		//g_blockSolve = !g_blockSolve;
	}
}

func (bs *BoxStack) step(settings *Settings) {

	bs.Test.step(settings)
	g_debugDraw.DrawString(5, bs.textLine, "Press: (,) to launch a bullet.")
	bs.textLine += bs.textIncrement
	// TODO
	//g_debugDraw.DrawString(5, bs.textLine, "Blocksolve = %d", box2d.)
}

var _ = RegisterTest("Stacking", "Boxes", CreateBoxStack)
