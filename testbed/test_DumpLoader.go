package main

import "github.com/cuihonglei/gobox2d/box2d"

type DumpLoader struct {
	Test

	ball *box2d.Body
}

func CreateDumpLoader() ITest {
	dl := &DumpLoader{}
	dl.Test.init(dl)

	chainShape := box2d.MakeChainShape()
	vertices := []box2d.Vec2{box2d.MakeVec2(-5, 0), box2d.MakeVec2(5, 0), box2d.MakeVec2(5, 5), box2d.MakeVec2(4, 1), box2d.MakeVec2(-4, 1), box2d.MakeVec2(-5, 5)}
	chainShape.CreateLoop(vertices)

	groundFixtureDef := box2d.MakeFixtureDef()
	groundFixtureDef.Density = 0
	groundFixtureDef.Shape = &chainShape

	groundBodyDef := box2d.MakeBodyDef()
	groundBodyDef.Type = box2d.StaticBody

	groundBody := dl.world.CreateBody(&groundBodyDef)
	/*groundBodyFixture :=*/ groundBody.CreateFixture(&groundFixtureDef)

	ballShape := box2d.MakeCircleShape()
	ballShape.Radius = 1

	ballFixtureDef := box2d.MakeFixtureDef()
	ballFixtureDef.Restitution = 0.75
	ballFixtureDef.Density = 1
	ballFixtureDef.Shape = &ballShape

	ballBodyDef := box2d.MakeBodyDef()
	ballBodyDef.Type = box2d.DynamicBody
	ballBodyDef.Position = box2d.MakeVec2(0, 10)
	//ballBodyDef.AngularDamping = 0.2

	dl.ball = dl.world.CreateBody(&ballBodyDef)
	/*ballFixture :=*/ dl.ball.CreateFixture(&ballFixtureDef)
	// TODO
	//dl.ball.ApplyForceToCenter(box2d.MakeVec2(-1000, -400), true)
	dl.ball.ApplyForceToCenter(box2d.MakeVec2(-1000, -400))

	return dl
}

func (dl *DumpLoader) step(settings *Settings) {

	v := dl.ball.GetLinearVelocity()
	omega := dl.ball.GetAngularVelocity()

	var massData box2d.MassData
	dl.ball.GetMassData(&massData)

	ke := 0.5*massData.Mass*box2d.DotVV(v, v) + 0.5*massData.I*omega*omega

	g_debugDraw.DrawString(5, dl.textLine, "kinetic energy = %.6f", ke)
	dl.textLine += dl.textIncrement

	dl.Test.step(settings)
}

var _ = RegisterTest("Bugs", "Dump Loader", CreateDumpLoader)
