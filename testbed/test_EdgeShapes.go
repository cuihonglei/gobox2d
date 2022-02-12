package main

import (
	"math"

	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/glfw/v3.3/glfw"
)

type EdgeShapesCallback struct {
	fixture *box2d.Fixture
	point   box2d.Vec2
	normal  box2d.Vec2
}

func (cb *EdgeShapesCallback) ReportFixture(fixture *box2d.Fixture, point box2d.Vec2, normal box2d.Vec2, fraction float64) float64 {

	cb.fixture = fixture
	cb.point = point
	cb.normal = normal

	return fraction
}

const (
	EdgeShapes_e_maxBodies int = 256
)

type EdgeShapes struct {
	Test

	bodyIndex int
	bodies    [EdgeShapes_e_maxBodies]*box2d.Body
	polygons  [4]box2d.PolygonShape
	circle    box2d.CircleShape

	angle float64
}

func CreateEdgeShapes() ITest {
	es := &EdgeShapes{}
	for i := 0; i < len(es.polygons); i++ {
		es.polygons[i] = box2d.MakePolygonShape()
	}
	es.circle = box2d.MakeCircleShape()

	es.Test.init(es)

	// Ground body
	{
		bd := box2d.MakeBodyDef()
		ground := es.world.CreateBody(&bd)

		x1 := -20.0
		y1 := 2.0 * math.Cos(x1/10.0*box2d.Pi)
		for i := 0; i < 80; i++ {

			x2 := x1 + 0.5
			y2 := 2.0 * math.Cos(x2/10.0*box2d.Pi)

			shape := box2d.MakeEdgeShape()
			// TODO
			//shape.SetTwoSided(box2d.Vec2{X: x1, Y: y1}, box2d.Vec2{X: x2, Y: y2})
			shape.Set(box2d.Vec2{X: x1, Y: y1}, box2d.Vec2{X: x2, Y: y2})
			ground.CreateFixture2(&shape, 0.0)

			x1 = x2
			y1 = y2
		}
	}

	{
		var vertices [3]box2d.Vec2
		vertices[0].Set(-0.5, 0.0)
		vertices[1].Set(0.5, 0.0)
		vertices[2].Set(0.0, 1.5)
		es.polygons[0].Set(vertices[:])
	}

	{
		var vertices [3]box2d.Vec2
		vertices[0].Set(-0.1, 0.0)
		vertices[1].Set(0.1, 0.0)
		vertices[2].Set(0.0, 1.5)
		es.polygons[1].Set(vertices[:])
	}

	{
		w := 1.0
		b := w / (2.0 + box2d.Sqrt(2.0))
		s := box2d.Sqrt(2.0) * b

		var vertices [8]box2d.Vec2
		vertices[0].Set(0.5*s, 0.0)
		vertices[1].Set(0.5*w, b)
		vertices[2].Set(0.5*w, b+s)
		vertices[3].Set(0.5*s, w)
		vertices[4].Set(-0.5*s, w)
		vertices[5].Set(-0.5*w, b+s)
		vertices[6].Set(-0.5*w, b)
		vertices[7].Set(-0.5*s, 0.0)

		es.polygons[2].Set(vertices[:])
	}

	{
		es.polygons[3].SetAsBox(0.5, 0.5)
	}

	{
		es.circle.Radius = 0.5
	}

	es.bodyIndex = 0

	es.angle = 0.0

	return es
}

func (es *EdgeShapes) create(index int) {
	if es.bodies[es.bodyIndex] != nil {
		es.world.DestroyBody(es.bodies[es.bodyIndex])
		es.bodies[es.bodyIndex] = nil
	}

	bd := box2d.MakeBodyDef()

	x := RandomFloat2(-10.0, 10.0)
	y := RandomFloat2(10.0, 20.0)
	bd.Position.Set(x, y)
	bd.Angle = RandomFloat2(-box2d.Pi, box2d.Pi)
	bd.Type = box2d.DynamicBody

	if index == 4 {
		bd.AngularDamping = 0.02
	}

	es.bodies[es.bodyIndex] = es.world.CreateBody(&bd)

	if index < 4 {
		fd := box2d.MakeFixtureDef()
		fd.Shape = &es.polygons[index]
		fd.Friction = 0.3
		fd.Density = 20.0
		es.bodies[es.bodyIndex].CreateFixture(&fd)
	} else {
		fd := box2d.MakeFixtureDef()
		fd.Shape = &es.circle
		fd.Friction = 0.3
		fd.Density = 20.0
		es.bodies[es.bodyIndex].CreateFixture(&fd)
	}

	es.bodyIndex = (es.bodyIndex + 1) % EdgeShapes_e_maxBodies
}

func (es *EdgeShapes) destroyBody() {
	for i := 0; i < EdgeShapes_e_maxBodies; i++ {
		if es.bodies[i] != nil {
			es.world.DestroyBody(es.bodies[i])
			es.bodies[i] = nil
			return
		}
	}
}

func (es *EdgeShapes) keyboard(key glfw.Key) {

	switch key {
	case glfw.Key1, glfw.Key2, glfw.Key3, glfw.Key4, glfw.Key5:
		es.create(int(key - glfw.Key1))

	case glfw.KeyD:
		es.destroyBody()
	}
}

func (es *EdgeShapes) step(settings *Settings) {

	advanceRay := !settings.pause || settings.singleStep

	es.Test.step(settings)
	g_debugDraw.DrawString(5, es.textLine, "Press 1-5 to drop stuff")
	es.textLine += es.textIncrement

	L := 25.0
	point1 := box2d.MakeVec2(0.0, 10.0)
	d := box2d.MakeVec2(L*math.Cos(es.angle), -L*box2d.AbsF(math.Sin(es.angle)))
	point2 := box2d.AddVV(point1, d)

	var callback EdgeShapesCallback

	es.world.RayCast(&callback, point1, point2)

	if callback.fixture != nil {

		g_debugDraw.DrawPoint(callback.point, 5.0, box2d.MakeColor(0.4, 0.9, 0.4))

		g_debugDraw.DrawSegment(point1, callback.point, box2d.MakeColor(0.8, 0.8, 0.8))

		head := box2d.AddVV(callback.point, box2d.MulFV(0.5, callback.normal))
		g_debugDraw.DrawSegment(callback.point, head, box2d.MakeColor(0.9, 0.9, 0.4))
	} else {

		g_debugDraw.DrawSegment(point1, point2, box2d.MakeColor(0.8, 0.8, 0.8))
	}

	if advanceRay {
		es.angle += 0.25 * box2d.Pi / 180.0
	}
}

var _ = RegisterTest("Geometry", "Edge Shapes", CreateEdgeShapes)
