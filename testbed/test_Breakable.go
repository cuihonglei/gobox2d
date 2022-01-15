package main

import "github.com/cuihonglei/gobox2d/box2d"

const (
	Breakable_e_count int = 7
)

// This is used to test sensor shapes.
type Breakable struct {
	Test

	body1           *box2d.Body
	velocity        box2d.Vec2
	angularVelocity float64
	shape1          box2d.PolygonShape
	shape2          box2d.PolygonShape
	piece1          *box2d.Fixture
	piece2          *box2d.Fixture

	broke  bool
	xbreak bool
}

func CreateBreakable() ITest {
	b := &Breakable{
		shape1: box2d.MakePolygonShape(),
		shape2: box2d.MakePolygonShape(),
	}
	b.Test.init(b)

	// Ground body
	{
		bd := box2d.MakeBodyDef()
		ground := b.world.CreateBody(&bd)

		shape := box2d.MakeEdgeShape()
		// TODO
		//shape.SetTwoSided(box2d.Vec2{-40.0, 0.0}, box2d.Vec2{40.0, 0.0})
		shape.Set(box2d.Vec2{X: -40.0, Y: 0.0}, box2d.Vec2{X: 40.0, Y: 0.0})
		ground.CreateFixture2(&shape, 0.0)
	}

	// Breakable dynamic body
	{
		bd := box2d.MakeBodyDef()
		bd.Type = box2d.DynamicBody
		bd.Position.Set(0.0, 40.0)
		bd.Angle = 0.25 * box2d.Pi
		b.body1 = b.world.CreateBody(&bd)

		b.shape1.SetAsOrientedBox(0.5, 0.5, box2d.Vec2{X: -0.5, Y: 0.0}, 0.0)
		b.piece1 = b.body1.CreateFixture2(&b.shape1, 1.0)

		b.shape2.SetAsOrientedBox(0.5, 0.5, box2d.Vec2{X: 0.5, Y: 0.0}, 0.0)
		b.piece2 = b.body1.CreateFixture2(&b.shape2, 1.0)
	}

	b.xbreak = false
	b.broke = false

	return b
}

func (b *Breakable) PostSolve(contact box2d.IContact, impulse *box2d.ContactImpulse) {

	if b.broke {
		// The body already broke.
		return
	}

	// Should the body break?
	count := contact.GetManifold().PointCount

	maxImpulse := 0.0
	for i := 0; i < count; i++ {
		maxImpulse = box2d.MaxF(maxImpulse, impulse.NormalImpulses[i])
	}

	if maxImpulse > 40.0 {
		// Flag the body for breaking.
		b.xbreak = true
	}
}

func (b *Breakable) Break() {

	// Create two bodies from one.
	body1 := b.piece1.GetBody()
	center := body1.GetWorldCenter()

	body1.DestroyFixture(b.piece2)
	b.piece2 = nil

	bd := box2d.MakeBodyDef()
	bd.Type = box2d.DynamicBody
	bd.Position = body1.GetPosition()
	bd.Angle = body1.GetAngle()

	body2 := b.world.CreateBody(&bd)
	b.piece2 = body2.CreateFixture2(&b.shape2, 1.0)

	// Compute consistent velocities for new bodies based on
	// cached velocity.
	center1 := body1.GetWorldCenter()
	center2 := body2.GetWorldCenter()

	velocity1 := box2d.AddVV(b.velocity, box2d.CrossFV(b.angularVelocity, box2d.SubVV(center1, center)))
	velocity2 := box2d.AddVV(b.velocity, box2d.CrossFV(b.angularVelocity, box2d.SubVV(center2, center)))

	body1.SetAngularVelocity(b.angularVelocity)
	body1.SetLinearVelocity(velocity1)

	body2.SetAngularVelocity(b.angularVelocity)
	body2.SetLinearVelocity(velocity2)
}

func (b *Breakable) step(settings *Settings) {

	if b.xbreak {
		b.Break()
		b.broke = true
		b.xbreak = false
	}

	// Cache velocities to improve movement on breakage.
	if !b.broke {
		b.velocity = b.body1.GetLinearVelocity()
		b.angularVelocity = b.body1.GetAngularVelocity()
	}

	b.Test.step(settings)
}

var _ = RegisterTest("Examples", "Breakable", CreateBreakable)
