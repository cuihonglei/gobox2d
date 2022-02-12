package main

import "github.com/cuihonglei/gobox2d/box2d"

type BulletTest struct {
	Test

	body   *box2d.Body
	bullet *box2d.Body
	x      float64
}

func CreateBulletTest() ITest {
	bt := &BulletTest{}
	bt.Test.init(bt)

	{
		bd := box2d.MakeBodyDef()
		bd.Position.Set(0.0, 0.0)
		body := bt.world.CreateBody(&bd)

		edge := box2d.MakeEdgeShape()

		//TODO
		//edge.SetTwoSided(box2d.Vec2{X: -10.0, Y: 0.0}, box2d.Vec2{X: 10.0, Y: 0.0})
		edge.Set(box2d.Vec2{X: -10.0, Y: 0.0}, box2d.Vec2{X: 10.0, Y: 0.0})
		body.CreateFixture2(&edge, 0.0)

		shape := box2d.MakePolygonShape()
		shape.SetAsOrientedBox(0.2, 1.0, box2d.MakeVec2(0.5, 1.0), 0.0)
		body.CreateFixture2(&shape, 0.0)
	}

	{
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(0.0, 4.0)

		box := box2d.MakePolygonShape()
		box.SetAsBox(2.0, 0.1)

		bt.body = bt.world.CreateBody(&bd)
		bt.body.CreateFixture2(&box, 1.0)

		box.SetAsBox(0.25, 0.25)

		bt.x = 0.20352793
		bd.Position.Set(bt.x, 10.0)
		bd.Bullet = true

		bt.bullet = bt.world.CreateBody(&bd)
		bt.bullet.CreateFixture2(&box, 100.0)

		bt.bullet.SetLinearVelocity(box2d.MakeVec2(0.0, -50.0))

	}

	return bt
}

func (bt *BulletTest) launch() {

	bt.body.SetTransform(box2d.MakeVec2(0.0, 4.0), 0.0)
	bt.body.SetLinearVelocity(box2d.Vec2_zero)
	bt.body.SetAngularVelocity(0.0)

	bt.x = RandomFloat2(-1.0, 1.0)
	bt.bullet.SetTransform(box2d.MakeVec2(bt.x, 10.0), 0.0)
	bt.bullet.SetLinearVelocity(box2d.MakeVec2(0.0, -50.0))
	bt.bullet.SetAngularVelocity(0.0)

	box2d.GjkCalls = 0
	box2d.GjkIters = 0
	box2d.GjkMaxIters = 0

	box2d.ToiCalls = 0
	box2d.ToiIters = 0
	box2d.ToiMaxIters = 0
	box2d.ToiRootIters = 0
	box2d.ToiMaxRootIters = 0
}

func (bt *BulletTest) step(settings *Settings) {
	bt.Test.step(settings)

	if box2d.GjkCalls > 0 {
		g_debugDraw.DrawString(5, bt.textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
			box2d.GjkCalls, float64(box2d.GjkIters)/float64(box2d.GjkCalls), box2d.GjkMaxIters)
		bt.textLine += bt.textIncrement
	}

	if box2d.ToiCalls > 0 {
		g_debugDraw.DrawString(5, bt.textLine, "toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
			box2d.ToiCalls, float64(box2d.ToiIters)/float64(box2d.ToiCalls), box2d.ToiMaxRootIters)
		bt.textLine += bt.textIncrement

		g_debugDraw.DrawString(5, bt.textLine, "ave toi root iters = %3.1f, max toi root iters = %d",
			float64(box2d.ToiRootIters)/float64(box2d.ToiCalls), box2d.ToiMaxRootIters)
		bt.textLine += bt.textIncrement
	}

	if bt.stepCount%60 == 0 {
		bt.launch()
	}
}

var _ = RegisterTest("Continuous", "Bullet Test", CreateBulletTest)
