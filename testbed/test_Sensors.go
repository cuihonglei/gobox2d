package main

import (
	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/inkyblackness/imgui-go/v4"
)

const (
	Sensors_e_count int = 7
)

type Sensors struct {
	Test

	sensor   *box2d.Fixture
	bodies   [Sensors_e_count]*box2d.Body
	force    float32
	touching [Sensors_e_count]bool
}

func CreateSensors() ITest {
	s := &Sensors{}
	s.Test.init(s)

	{
		bd := box2d.MakeBodyDef()
		ground := s.world.CreateBody(&bd)

		{
			shape := box2d.MakeEdgeShape()
			//TODO
			//shape.SetTwoSided(box2d.Vec2{-40.0, 0.0}, box2d.Vec2{40.0, 0.0})
			shape.Set(box2d.Vec2{X: -40.0, Y: 0.0}, box2d.Vec2{X: 40.0, Y: 0.0})
			ground.CreateFixture2(&shape, 0.0)
		}

		// TODO
		// if false {

		// b2FixtureDef sd;
		// sd.SetAsBox(10.0f, 2.0f, b2Vec2(0.0f, 20.0f), 0.0f);
		// sd.isSensor = true;
		// m_sensor = ground->CreateFixture(&sd);

		// } else {
		{
			shape := box2d.MakeCircleShape()
			shape.Radius = 5.0
			shape.P.Set(0.0, 10.0)

			fd := box2d.MakeFixtureDef()
			fd.Shape = &shape
			fd.IsSensor = true
			s.sensor = ground.CreateFixture(&fd)
		}
		// }
	}

	{
		shape := box2d.MakeCircleShape()
		shape.Radius = 1.0

		for i := 0; i < Sensors_e_count; i++ {

			bd := box2d.MakeBodyDef()
			bd.Type = box2d.DynamicBody
			bd.Position.Set(-10.0+3.0*float64(i), 20.0)
			bd.UserData = i

			s.touching[i] = false
			s.bodies[i] = s.world.CreateBody(&bd)

			s.bodies[i].CreateFixture2(&shape, 1.0)
		}
	}

	s.force = 100.0

	return s
}

// Implement contact listener.
func (s *Sensors) BeginContact(contact box2d.IContact) {

	fixtureA := contact.GetFixtureA()
	fixtureB := contact.GetFixtureB()

	if fixtureA == s.sensor {
		index := fixtureB.GetBody().GetUserData().(int)
		if index < Sensors_e_count {
			s.touching[index] = true
		}
	}

	if fixtureB == s.sensor {
		index := fixtureA.GetBody().GetUserData().(int)
		if index < Sensors_e_count {
			s.touching[index] = true
		}
	}
}

// Implement contact listener.
func (s *Sensors) EndContact(contact box2d.IContact) {

	fixtureA := contact.GetFixtureA()
	fixtureB := contact.GetFixtureB()

	if fixtureA == s.sensor {
		index := fixtureB.GetBody().GetUserData().(int)
		if index < Sensors_e_count {
			s.touching[index] = false
		}
	}

	if fixtureB == s.sensor {
		index := fixtureA.GetBody().GetUserData().(int)
		if index < Sensors_e_count {
			s.touching[index] = false
		}
	}
}

func (s *Sensors) updateUI() {
	imgui.SetNextWindowPos(imgui.Vec2{X: 10.0, Y: 100.0})
	imgui.SetNextWindowSize(imgui.Vec2{X: 200.0, Y: 60.0})
	imgui.BeginV("Sensor Controls", nil, imgui.WindowFlagsNoMove|imgui.WindowFlagsNoResize)

	imgui.SliderFloatV("Force", &s.force, 0.0, 2000.0, "%.0f", imgui.SliderFlagsNone)

	imgui.End()
}

func (s *Sensors) step(settings *Settings) {
	s.Test.step(settings)

	// Traverse the contact results. Apply a force on shapes
	// that overlap the sensor.
	for i := 0; i < Sensors_e_count; i++ {

		if !s.touching[i] {
			continue
		}

		body := s.bodies[i]
		ground := s.sensor.GetBody()

		circle := s.sensor.GetShape().(*box2d.CircleShape)
		center := ground.GetWorldPoint(circle.P)

		position := body.GetPosition()

		d := box2d.SubVV(center, position)
		if d.LengthSquared() < box2d.Epsilon*box2d.Epsilon {
			continue
		}

		d.Normalize()
		F := box2d.MulFV(float64(s.force), d)
		// TODO
		//body.ApplyForce(F, position, false)
		body.ApplyForce(F, position)
	}
}

var _ = RegisterTest("Collision", "Sensors", CreateSensors)
