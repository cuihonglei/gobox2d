package box2d

/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
/// For example, anything slides on ice.
func MixFriction(friction1, friction2 float64) float64 {
	return Sqrt(friction1 * friction2)
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
func MixRestitution(restitution1, restitution2 float64) float64 {
	if restitution1 > restitution2 {
		return restitution1
	}
	return restitution2
}

type ContactRegister struct {
	CreateFcn func(*Fixture, int, *Fixture, int) IContact
	Primary   bool
}

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// nodes, one for each attached body.
type ContactEdge struct {
	Other   *Body        // provides quick access to the other body attached.
	Contact IContact     // the contact
	Prev    *ContactEdge // the previous contact edge in the body's contact list
	Next    *ContactEdge // the next contact edge in the body's contact list
}

type IContact interface {
	GetNext() IContact
	GetPrev() IContact
	SetNext(IContact)
	SetPrev(IContact)

	GetNodeA() *ContactEdge
	GetNodeB() *ContactEdge

	GetManifold() *Manifold
	GetWorldManifold() *WorldManifold

	GetToiCount() int
	SetToiCount(int)
	GetToi() float64
	SetToi(float64)

	GetFlags() uint
	SetFlags(uint)

	GetFriction() float64
	GetRestitution() float64

	// Get fixture A in this contact.
	GetFixtureA() *Fixture

	// Get the child primitive index for fixture A.
	GetChildIndexA() int

	// Get fixture B in this contact.
	GetFixtureB() *Fixture

	// Get the child primitive index for fixture B.
	GetChildIndexB() int

	// Flag this contact for filtering. Filtering will occur the next time step.
	FlagForFiltering()

	SetEnabled(bool)
	IsEnabled() bool
	IsTouching() bool

	Update(listener IContactListener)
}

type IContactEx interface {
	/// Evaluate this contact with your own manifold and transforms.
	Evaluate(manifold *Manifold, xfA Transform, xfB Transform)
}

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
type Contact struct {
	Ex IContactEx

	Flags uint

	// World pool and list pointers.
	Prev IContact
	Next IContact

	// Nodes for connecting bodies.
	NodeA ContactEdge
	NodeB ContactEdge

	FixtureA *Fixture
	FixtureB *Fixture

	IndexA int
	IndexB int

	Manifold Manifold

	ToiCount int
	Toi      float64

	Friction    float64
	Restitution float64
}

// Flags stored in flags
const (
	// Used when crawling contact graph when forming islands.
	Contact_e_islandFlag uint = 0x0001

	// Set when the shapes are touching.
	Contact_e_touchingFlag uint = 0x0002

	// This contact can be disabled (by user)
	Contact_e_enabledFlag uint = 0x0004

	// This contact needs filtering because a fixture filter was changed.
	Contact_e_filterFlag uint = 0x0008

	// This bullet contact had a TOI event
	Contact_e_bulletHitFlag uint = 0x0010

	// This contact has a valid TOI in m_toi
	Contact_e_toiFlag uint = 0x0020
)

func (c *Contact) Init(ex IContactEx, fA *Fixture, indexA int, fB *Fixture, indexB int) {
	c.Ex = ex

	c.Flags = Contact_e_enabledFlag

	c.FixtureA = fA
	c.FixtureB = fB

	c.IndexA = indexA
	c.IndexB = indexB

	c.Friction = MixFriction(c.FixtureA.Friction, c.FixtureB.Friction)
	c.Restitution = MixRestitution(c.FixtureA.Restitution, c.FixtureB.Restitution)
}

func (c *Contact) GetNext() IContact {
	return c.Next
}

func (c *Contact) GetPrev() IContact {
	return c.Prev
}

func (c *Contact) SetNext(next IContact) {
	c.Next = next
}

func (c *Contact) SetPrev(prev IContact) {
	c.Prev = prev
}

func (c *Contact) GetNodeA() *ContactEdge {
	return &c.NodeA
}

func (c *Contact) GetNodeB() *ContactEdge {
	return &c.NodeB
}

/// Get the contact manifold. Do not modify the manifold unless you understand the
/// internals of Box2D.
func (c *Contact) GetManifold() *Manifold {
	return &c.Manifold
}

func (c *Contact) GetToiCount() int {
	return c.ToiCount
}

func (c *Contact) SetToiCount(toiCount int) {
	c.ToiCount = toiCount
}

func (c *Contact) GetToi() float64 {
	return c.Toi
}

func (c *Contact) SetToi(toi float64) {
	c.Toi = toi
}

func (c *Contact) GetFlags() uint {
	return c.Flags
}

func (c *Contact) SetFlags(flags uint) {
	c.Flags = flags
}

/// Get the world manifold.
func (c *Contact) GetWorldManifold() *WorldManifold {
	worldManifold := new(WorldManifold)

	bodyA := c.FixtureA.GetBody()
	bodyB := c.FixtureB.GetBody()
	shapeA := c.FixtureA.GetShape()
	shapeB := c.FixtureB.GetShape()

	worldManifold.Initialize(&c.Manifold, bodyA.GetTransform(), shapeA.GetRadius(), bodyB.GetTransform(), shapeB.GetRadius())
	return worldManifold
}

/// Is this contact touching?
func (c *Contact) IsTouching() bool {
	return (c.Flags & Contact_e_touchingFlag) == Contact_e_touchingFlag
}

/// Enable/disable this contact. This can be used inside the pre-solve
/// contact listener. The contact is only disabled for the current
/// time step (or sub-step in continuous collisions).
func (c *Contact) SetEnabled(flag bool) {
	if flag {
		c.Flags |= Contact_e_enabledFlag
	} else {
		c.Flags &= ^Contact_e_enabledFlag
	}
}

/// Has this contact been disabled?
func (c *Contact) IsEnabled() bool {
	return (c.Flags & Contact_e_enabledFlag) == Contact_e_enabledFlag
}

// Get fixture A in this contact.
func (c *Contact) GetFixtureA() *Fixture {
	return c.FixtureA
}

// Get the child primitive index for fixture A.
func (c *Contact) GetChildIndexA() int {
	return c.IndexA
}

// Get fixture B in this contact.
func (c *Contact) GetFixtureB() *Fixture {
	return c.FixtureB
}

func (c *Contact) GetChildIndexB() int {
	return c.IndexB
}

/// Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
/// This value persists until set or reset.
func (c *Contact) SetFriction(friction float64) {
	c.Friction = friction
}

/// Get the friction.
func (c *Contact) GetFriction() float64 {
	return c.Friction
}

/// Reset the friction mixture to the default value.
func (c *Contact) ResetFriction() {
	c.Friction = MixFriction(c.FixtureA.Friction, c.FixtureB.Friction)
}

/// Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
/// The value persists until you set or reset.
func (c *Contact) SetRestitution(restitution float64) {
	c.Restitution = restitution
}

/// Get the restitution.
func (c *Contact) GetRestitution() float64 {
	return c.Restitution
}

/// Reset the restitution to the default value.
func (c *Contact) ResetRestitution() {
	c.Restitution = MixRestitution(c.FixtureA.Restitution, c.FixtureB.Restitution)
}

func (c *Contact) FlagForFiltering() {
	c.Flags |= Contact_e_filterFlag
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
func (c *Contact) Update(listener IContactListener) {
	oldManifold := c.Manifold

	// Re-enable this contact.
	c.Flags |= Contact_e_enabledFlag

	touching := false
	wasTouching := (c.Flags & Contact_e_touchingFlag) == Contact_e_touchingFlag

	sensorA := c.FixtureA.GetSensor()
	sensorB := c.FixtureB.GetSensor()
	sensor := sensorA || sensorB

	bodyA := c.FixtureA.GetBody()
	bodyB := c.FixtureB.GetBody()
	xfA := bodyA.GetTransform()
	xfB := bodyB.GetTransform()

	// Is this contact a sensor?
	if sensor {
		shapeA := c.FixtureA.GetShape()
		shapeB := c.FixtureB.GetShape()
		touching = TestOverlap(shapeA, c.IndexA, shapeB, c.IndexB, xfA, xfB)

		// Sensors don't generate manifolds.
		c.Manifold.PointCount = 0
	} else {
		c.Ex.Evaluate(&c.Manifold, xfA, xfB)
		touching = c.Manifold.PointCount > 0

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for i := 0; i < c.Manifold.PointCount; i++ {
			mp2 := &c.Manifold.Points[i]
			mp2.NormalImpulse = 0.0
			mp2.TangentImpulse = 0.0
			id2 := mp2.Id

			for j := 0; j < oldManifold.PointCount; j++ {
				mp1 := &oldManifold.Points[j]

				if mp1.Id.Cf == id2.Cf {
					mp2.NormalImpulse = mp1.NormalImpulse
					mp2.TangentImpulse = mp1.TangentImpulse
					break
				}
			}
		}

		if touching != wasTouching {
			bodyA.SetAwake(true)
			bodyB.SetAwake(true)
		}
	}

	if touching {
		c.Flags |= Contact_e_touchingFlag
	} else {
		c.Flags &= ^Contact_e_touchingFlag
	}

	if !wasTouching && touching && listener != nil {
		listener.BeginContact(c.Ex.(IContact))
	}

	if wasTouching && !touching && listener != nil {
		listener.EndContact(c.Ex.(IContact))
	}

	if !sensor && touching && listener != nil {
		listener.PreSolve(c.Ex.(IContact), &oldManifold)
	}
}

//
// ChainAndCircleContact
//
type ChainAndCircleContact struct {
	Contact
}

func ChainAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	c := new(ChainAndCircleContact)
	c.Contact.Init(c, fixtureA, indexA, fixtureB, indexB)
	return c
}

func (c *ChainAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	chain := c.FixtureA.GetShape().(*ChainShape)
	edge := chain.GetChildEdge(c.IndexA)
	CollideEdgeAndCircle(manifold, edge, xfA,
		c.FixtureB.GetShape().(*CircleShape), xfB)
}

//
// ChainAndPolygonContact
//
type ChainAndPolygonContact struct {
	Contact
}

func ChainAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	c := new(ChainAndPolygonContact)
	c.Contact.Init(c, fixtureA, indexA, fixtureB, indexB)
	return c
}

func (c *ChainAndPolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	chain := c.FixtureA.GetShape().(*ChainShape)
	edge := chain.GetChildEdge(c.IndexA)
	CollideEdgeAndPolygon(manifold, edge, xfA,
		c.FixtureB.GetShape().(*PolygonShape), xfB)
}

//
// CircleContact
//
type CircleContact struct {
	Contact
}

func CircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	c := new(CircleContact)
	c.Contact.Init(c, fixtureA, indexA, fixtureB, indexB)
	return c
}

func (c *CircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollideCircles(manifold, c.FixtureA.GetShape().(*CircleShape), xfA,
		c.FixtureB.GetShape().(*CircleShape), xfB)
}

//
// EdgeAndCircleContact
//
type EdgeAndCircleContact struct {
	Contact
}

func EdgeAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	c := new(EdgeAndCircleContact)
	c.Contact.Init(c, fixtureA, indexA, fixtureB, indexB)
	return c
}

func (c *EdgeAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollideEdgeAndCircle(manifold, c.FixtureA.GetShape().(*EdgeShape), xfA,
		c.FixtureB.GetShape().(*CircleShape), xfB)
}

//
// EdgeAndPolygonContact
//
type EdgeAndPolygonContact struct {
	Contact
}

func EdgeAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	c := new(EdgeAndPolygonContact)
	c.Contact.Init(c, fixtureA, indexA, fixtureB, indexB)
	return c
}

func (c *EdgeAndPolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollideEdgeAndPolygon(manifold, c.FixtureA.GetShape().(*EdgeShape), xfA,
		c.FixtureB.GetShape().(*PolygonShape), xfB)
}

//
// PolygonAndCircleContact
//
type PolygonAndCircleContact struct {
	Contact
}

func PolygonAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	c := new(PolygonAndCircleContact)
	c.Contact.Init(c, fixtureA, indexA, fixtureB, indexB)
	return c
}

func (c *PolygonAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollidePolygonAndCircle(manifold, c.FixtureA.GetShape().(*PolygonShape), xfA,
		c.FixtureB.GetShape().(*CircleShape), xfB)
}

//
// PolygonContact
//
type PolygonContact struct {
	Contact
}

func PolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	c := new(PolygonContact)
	c.Contact.Init(c, fixtureA, indexA, fixtureB, indexB)
	return c
}

func (c *PolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollidePolygons(manifold, c.FixtureA.GetShape().(*PolygonShape), xfA,
		c.FixtureB.GetShape().(*PolygonShape), xfB)
}

var Contact_s_registers [Shape_e_typeCount][Shape_e_typeCount]ContactRegister
var Contact_s_initialized bool = false

func Contact_AddType(createFcn func(*Fixture, int, *Fixture, int) IContact, type1, type2 ShapeType) {
	//Assert(0 <= type1 && type1 < Shape_e_typeCount)
	//Assert(0 <= type2 && type2 < Shape_e_typeCount)

	Contact_s_registers[type1][type2].CreateFcn = createFcn
	Contact_s_registers[type1][type2].Primary = true

	if type1 != type2 {
		Contact_s_registers[type2][type1].CreateFcn = createFcn
		Contact_s_registers[type2][type1].Primary = false
	}
}
func Contact_InitializeRegisters() {
	Contact_AddType(CircleContact_Create, Shape_e_circle, Shape_e_circle)
	Contact_AddType(PolygonAndCircleContact_Create, Shape_e_polygon, Shape_e_circle)
	Contact_AddType(PolygonContact_Create, Shape_e_polygon, Shape_e_polygon)
	Contact_AddType(EdgeAndCircleContact_Create, Shape_e_edge, Shape_e_circle)
	Contact_AddType(EdgeAndPolygonContact_Create, Shape_e_edge, Shape_e_polygon)
	Contact_AddType(ChainAndCircleContact_Create, Shape_e_chain, Shape_e_circle)
	Contact_AddType(ChainAndPolygonContact_Create, Shape_e_chain, Shape_e_polygon)
}
func Contact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	if !Contact_s_initialized {
		Contact_InitializeRegisters()
		Contact_s_initialized = true
	}

	// TODO 不知道什么原因调用此方法时有时候fixture是已经被销毁了的对象 shape==nil
	if fixtureA.Shape == nil || fixtureB.Shape == nil {
		return nil
	}

	type1 := fixtureA.GetType()
	type2 := fixtureB.GetType()

	//Assert(0 <= type1 && type1 < Shape_e_typeCount)
	//Assert(0 <= type2 && type2 < Shape_e_typeCount)

	createFcn := Contact_s_registers[type1][type2].CreateFcn
	if createFcn != nil {
		if Contact_s_registers[type1][type2].Primary {
			return createFcn(fixtureA, indexA, fixtureB, indexB)
		} else {
			return createFcn(fixtureB, indexB, fixtureA, indexA)
		}
	} else {
		return nil
	}
}

func Contact_Destroy(contact IContact) {
	//Assert(Contact_s_initialized == true)

	if contact.GetManifold().PointCount > 0 {
		contact.GetFixtureA().GetBody().SetAwake(true)
		contact.GetFixtureB().GetBody().SetAwake(true)
	}

	//typeA := contact.GetFixtureA().GetType()
	//typeB := contact.GetFixtureB().GetType()

	//Assert(0 <= typeA && typeB < Shape_e_typeCount)
	//Assert(0 <= typeA && typeB < Shape_e_typeCount)

	//contact.Destroy()
}

//func Contact_Destroy1(contact *Contact, typeA ShapeType, typeB ShapeType) {

//}
