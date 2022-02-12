package box2d

// Joints and fixtures are destroyed when their associated
// body is destroyed. Implement this listener so that you
// may nullify references to these joints and shapes.
type IDestructionListener interface {
	SayGoodbyeJoint(joint IJoint)
	SayGoodbyeFixture(fixture *Fixture)
}

// Implement this class to provide collision filtering. In other words, you can implement
// this class if you want finer control over contact creation.
type IContactFilter interface {
	ShouldCollide(fixtureA *Fixture, fixtureB *Fixture) bool
}

type ContactFilter struct {
}

func NewContactFilter() *ContactFilter {
	cf := new(ContactFilter)
	return cf
}

// Return true if contact calculations should be performed between these two shapes.
// If you implement your own collision filter you may want to build from this implementation.
func (cf *ContactFilter) ShouldCollide(fixtureA *Fixture, fixtureB *Fixture) bool {
	filterA := fixtureA.GetFilterData()
	filterB := fixtureB.GetFilterData()

	if filterA.GroupIndex == filterB.GroupIndex && filterA.GroupIndex != 0 {
		return filterA.GroupIndex > 0
	}

	return (filterA.MaskBits&filterB.CategoryBits) != 0 && (filterA.CategoryBits&filterB.MaskBits) != 0
}

// Contact impulses for reporting. Impulses are used instead of forces because
// sub-step forces may approach infinity for rigid body collisions. These
// match up one-to-one with the contact points in b2Manifold.
type ContactImpulse struct {
	NormalImpulses  [MaxManifoldPoints]float64
	TangentImpulses [MaxManifoldPoints]float64
	Count           int
}

// Implement this class to get contact information. You can use these results for
// things like sounds and game logic. You can also get contact results by
// traversing the contact lists after the time step. However, you might miss
// some contacts because continuous physics leads to sub-stepping.
// Additionally you may receive multiple callbacks for the same contact in a
// single time step.
// You should strive to make your callbacks efficient because there may be
// many callbacks per time step.
// @warning You cannot create/destroy Box2D entities inside these callbacks.
type IContactListener interface {
	// Called when two fixtures begin to touch.
	BeginContact(contact IContact)

	// Called when two fixtures cease to touch.
	EndContact(contact IContact)

	// This is called after a contact is updated. This allows you to inspect a
	// contact before it goes to the solver. If you are careful, you can modify the
	// contact manifold (e.g. disable contact).
	// A copy of the old manifold is provided so that you can detect changes.
	// Note: this is called only for awake bodies.
	// Note: this is called even when the number of contact points is zero.
	// Note: this is not called for sensors.
	// Note: if you set the number of contact points to zero, you will not
	// get an EndContact callback. However, you may get a BeginContact callback
	// the next step.
	PreSolve(contact IContact, oldManifold *Manifold)

	// This lets you inspect a contact after the solver is finished. This is useful
	// for inspecting impulses.
	// Note: the contact manifold does not include time of impact impulses, which can be
	// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
	// in a separate data structure.
	// Note: this is only called for contacts that are touching, solid, and awake.
	PostSolve(contact IContact, impulse *ContactImpulse)
}

type ContactListener struct {
}

func NewContactListener() *ContactListener {
	cl := new(ContactListener)
	return cl
}

func (cl *ContactListener) BeginContact(contact IContact)                       {}
func (cl *ContactListener) EndContact(contact IContact)                         {}
func (cl *ContactListener) PreSolve(contact IContact, oldManifold *Manifold)    {}
func (cl *ContactListener) PostSolve(contact IContact, impulse *ContactImpulse) {}

// Callback class for AABB queries.
// See b2World::Query
type IQueryCallback interface {
	ReportFixture(fixture *Fixture) bool
}

// Callback class for ray casts.
// See b2World::RayCast
type IRayCastCallback interface {

	// Called for each fixture found in the query. You control how the ray cast
	// proceeds by returning a float:
	// return -1: ignore this fixture and continue
	// return 0: terminate the ray cast
	// return fraction: clip the ray to this point
	// return 1: don't clip the ray and continue
	// @param fixture the fixture hit by the ray
	// @param point the point of initial intersection
	// @param normal the normal vector at the point of intersection
	// @param fraction the fraction along the ray at the point of intersection
	// @return -1 to filter, 0 to terminate, fraction to clip the ray for
	// closest hit, 1 to continue
	ReportFixture(fixture *Fixture, point Vec2, normal Vec2, fraction float64) float64
}
