package main

import (
	"math"

	"github.com/cuihonglei/gobox2d/box2d"
)

const (
	Tiles_e_count int = 20
)

// This stress tests the dynamic tree broad-phase. This also shows that tile
// based collision is _not_ smooth due to Box2D not knowing about adjacency.
type Tiles struct {
	Test

	fixtureCount int
	createTime   float64
}

func CreateTiles() ITest {
	t := &Tiles{}
	t.Test.init(t)

	t.fixtureCount = 0
	timer := box2d.MakeTimer()

	{
		a := 0.5
		bd := box2d.MakeBodyDef()
		bd.Position.Y = -a
		ground := t.world.CreateBody(&bd)

		N := 200
		M := 10
		var position box2d.Vec2
		position.Y = 0.0
		for j := 0; j < M; j++ {
			position.X = -float64(N) * a
			for i := 0; i < N; i++ {
				shape := box2d.MakePolygonShape()
				shape.SetAsOrientedBox(a, a, position, 0.0)
				ground.CreateFixture2(&shape, 0.0)
				t.fixtureCount += 1
				position.X += 2.0 * a
			}
			position.Y -= 2.0 * a
		}
	}

	{
		a := 0.5
		shape := box2d.MakePolygonShape()
		shape.SetAsBox(a, a)

		x := box2d.MakeVec2(-7.0, 0.75)
		y := box2d.Vec2{}
		deltaX := box2d.MakeVec2(0.5625, 1.25)
		deltaY := box2d.MakeVec2(1.125, 0.0)

		for i := 0; i < Tiles_e_count; i++ {
			y = x

			for j := i; j < Tiles_e_count; j++ {
				bd := box2d.MakeBodyDef()
				bd.Type = box2d.DynamicBody
				bd.Position = y

				body := t.world.CreateBody(&bd)
				body.CreateFixture2(&shape, 5.0)
				t.fixtureCount += 1
				y.Add(deltaY)
			}

			x.Add(deltaX)
		}
	}

	t.createTime = timer.GetMilliseconds()

	return t
}

func (t *Tiles) step(settings *Settings) {

	cm := t.world.GetContactManager()
	height := cm.BroadPhase.GetTreeHeight()
	leafCount := cm.BroadPhase.GetProxyCount()
	minimumNodeCount := 2*leafCount - 1
	minimumHeight := math.Ceil(math.Log(float64(minimumNodeCount)) / math.Log(2.0))
	g_debugDraw.DrawString(5, t.textLine, "dynamic tree height = %d, min = %d", height, int(minimumHeight))
	t.textLine += t.textIncrement

	t.Test.step(settings)

	g_debugDraw.DrawString(5, t.textLine, "create time = %6.2f ms, fixture count = %d",
		t.createTime, t.fixtureCount)
	t.textLine += t.textIncrement
}

var _ = RegisterTest("Benchmark", "Tiles", CreateTiles)
