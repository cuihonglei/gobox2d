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

func (this *Contact) Init(ex IContactEx, fA *Fixture, indexA int, fB *Fixture, indexB int) {
	this.Ex = ex

	this.Flags = Contact_e_enabledFlag

	this.FixtureA = fA
	this.FixtureB = fB

	this.IndexA = indexA
	this.IndexB = indexB

	this.Friction = MixFriction(this.FixtureA.Friction, this.FixtureB.Friction)
	this.Restitution = MixRestitution(this.FixtureA.Restitution, this.FixtureB.Restitution)
}

func (this *Contact) GetNext() IContact {
	return this.Next
}

func (this *Contact) GetPrev() IContact {
	return this.Prev
}

func (this *Contact) SetNext(next IContact) {
	this.Next = next
}

func (this *Contact) SetPrev(prev IContact) {
	this.Prev = prev
}

func (this *Contact) GetNodeA() *ContactEdge {
	return &this.NodeA
}

func (this *Contact) GetNodeB() *ContactEdge {
	return &this.NodeB
}

/// Get the contact manifold. Do not modify the manifold unless you understand the
/// internals of Box2D.
func (this *Contact) GetManifold() *Manifold {
	return &this.Manifold
}

func (this *Contact) GetToiCount() int {
	return this.ToiCount
}

func (this *Contact) SetToiCount(toiCount int) {
	this.ToiCount = toiCount
}

func (this *Contact) GetToi() float64 {
	return this.Toi
}

func (this *Contact) SetToi(toi float64) {
	this.Toi = toi
}

func (this *Contact) GetFlags() uint {
	return this.Flags
}

func (this *Contact) SetFlags(flags uint) {
	this.Flags = flags
}

/// Get the world manifold.
func (this *Contact) GetWorldManifold() *WorldManifold {
	worldManifold := new(WorldManifold)

	bodyA := this.FixtureA.GetBody()
	bodyB := this.FixtureB.GetBody()
	shapeA := this.FixtureA.GetShape()
	shapeB := this.FixtureB.GetShape()

	worldManifold.Initialize(&this.Manifold, bodyA.GetTransform(), shapeA.GetRadius(), bodyB.GetTransform(), shapeB.GetRadius())
	return worldManifold
}

/// Is this contact touching?
func (this *Contact) IsTouching() bool {
	return (this.Flags & Contact_e_touchingFlag) == Contact_e_touchingFlag
}

/// Enable/disable this contact. This can be used inside the pre-solve
/// contact listener. The contact is only disabled for the current
/// time step (or sub-step in continuous collisions).
func (this *Contact) SetEnabled(flag bool) {
	if flag {
		this.Flags |= Contact_e_enabledFlag
	} else {
		this.Flags &= ^Contact_e_enabledFlag
	}
}

/// Has this contact been disabled?
func (this *Contact) IsEnabled() bool {
	return (this.Flags & Contact_e_enabledFlag) == Contact_e_enabledFlag
}

// Get fixture A in this contact.
func (this *Contact) GetFixtureA() *Fixture {
	return this.FixtureA
}

// Get the child primitive index for fixture A.
func (this *Contact) GetChildIndexA() int {
	return this.IndexA
}

// Get fixture B in this contact.
func (this *Contact) GetFixtureB() *Fixture {
	return this.FixtureB
}

func (this *Contact) GetChildIndexB() int {
	return this.IndexB
}

/// Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
/// This value persists until set or reset.
func (this *Contact) SetFriction(friction float64) {
	this.Friction = friction
}

/// Get the friction.
func (this *Contact) GetFriction() float64 {
	return this.Friction
}

/// Reset the friction mixture to the default value.
func (this *Contact) ResetFriction() {
	this.Friction = MixFriction(this.FixtureA.Friction, this.FixtureB.Friction)
}

/// Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
/// The value persists until you set or reset.
func (this *Contact) SetRestitution(restitution float64) {
	this.Restitution = restitution
}

/// Get the restitution.
func (this *Contact) GetRestitution() float64 {
	return this.Restitution
}

/// Reset the restitution to the default value.
func (this *Contact) ResetRestitution() {
	this.Restitution = MixRestitution(this.FixtureA.Restitution, this.FixtureB.Restitution)
}

func (this *Contact) FlagForFiltering() {
	this.Flags |= Contact_e_filterFlag
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
func (this *Contact) Update(listener IContactListener) {
	oldManifold := this.Manifold

	// Re-enable this contact.
	this.Flags |= Contact_e_enabledFlag

	touching := false
	wasTouching := (this.Flags & Contact_e_touchingFlag) == Contact_e_touchingFlag

	sensorA := this.FixtureA.GetSensor()
	sensorB := this.FixtureB.GetSensor()
	sensor := sensorA || sensorB

	bodyA := this.FixtureA.GetBody()
	bodyB := this.FixtureB.GetBody()
	xfA := bodyA.GetTransform()
	xfB := bodyB.GetTransform()

	// Is this contact a sensor?
	if sensor {
		shapeA := this.FixtureA.GetShape()
		shapeB := this.FixtureB.GetShape()
		touching = TestOverlap(shapeA, this.IndexA, shapeB, this.IndexB, xfA, xfB)

		// Sensors don't generate manifolds.
		this.Manifold.PointCount = 0
	} else {
		this.Ex.Evaluate(&this.Manifold, xfA, xfB)
		touching = this.Manifold.PointCount > 0

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for i := 0; i < this.Manifold.PointCount; i++ {
			mp2 := &this.Manifold.Points[i]
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
		this.Flags |= Contact_e_touchingFlag
	} else {
		this.Flags &= ^Contact_e_touchingFlag
	}

	if !wasTouching && touching && listener != nil {
		listener.BeginContact(this.Ex.(IContact))
	}

	if wasTouching && !touching && listener != nil {
		listener.EndContact(this.Ex.(IContact))
	}

	if !sensor && touching && listener != nil {
		listener.PreSolve(this.Ex.(IContact), &oldManifold)
	}
}

//
// ChainAndCircleContact
//
type ChainAndCircleContact struct {
	Contact
}

func ChainAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	this := new(ChainAndCircleContact)
	this.Contact.Init(this, fixtureA, indexA, fixtureB, indexB)
	return this
}

func (this *ChainAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	chain := this.FixtureA.GetShape().(*ChainShape)
	edge := chain.GetChildEdge(this.IndexA)
	CollideEdgeAndCircle(manifold, edge, xfA,
		this.FixtureB.GetShape().(*CircleShape), xfB)
}

//
// ChainAndPolygonContact
//
type ChainAndPolygonContact struct {
	Contact
}

func ChainAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	this := new(ChainAndPolygonContact)
	this.Contact.Init(this, fixtureA, indexA, fixtureB, indexB)
	return this
}

func (this *ChainAndPolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	chain := this.FixtureA.GetShape().(*ChainShape)
	edge := chain.GetChildEdge(this.IndexA)
	CollideEdgeAndPolygon(manifold, edge, xfA,
		this.FixtureB.GetShape().(*PolygonShape), xfB)
}

//
// CircleContact
//
type CircleContact struct {
	Contact
}

func CircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	this := new(CircleContact)
	this.Contact.Init(this, fixtureA, indexA, fixtureB, indexB)
	return this
}

func (this *CircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollideCircles(manifold, this.FixtureA.GetShape().(*CircleShape), xfA,
		this.FixtureB.GetShape().(*CircleShape), xfB)
}

//
// EdgeAndCircleContact
//
type EdgeAndCircleContact struct {
	Contact
}

func EdgeAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	this := new(EdgeAndCircleContact)
	this.Contact.Init(this, fixtureA, indexA, fixtureB, indexB)
	return this
}

func (this *EdgeAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollideEdgeAndCircle(manifold, this.FixtureA.GetShape().(*EdgeShape), xfA,
		this.FixtureB.GetShape().(*CircleShape), xfB)
}

//
// EdgeAndPolygonContact
//
type EdgeAndPolygonContact struct {
	Contact
}

func EdgeAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	this := new(EdgeAndPolygonContact)
	this.Contact.Init(this, fixtureA, indexA, fixtureB, indexB)
	return this
}

func (this *EdgeAndPolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollideEdgeAndPolygon(manifold, this.FixtureA.GetShape().(*EdgeShape), xfA,
		this.FixtureB.GetShape().(*PolygonShape), xfB)
}

//
// PolygonAndCircleContact
//
type PolygonAndCircleContact struct {
	Contact
}

func PolygonAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	this := new(PolygonAndCircleContact)
	this.Contact.Init(this, fixtureA, indexA, fixtureB, indexB)
	return this
}

func (this *PolygonAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollidePolygonAndCircle(manifold, this.FixtureA.GetShape().(*PolygonShape), xfA,
		this.FixtureB.GetShape().(*CircleShape), xfB)
}

//
// PolygonContact
//
type PolygonContact struct {
	Contact
}

func PolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	this := new(PolygonContact)
	this.Contact.Init(this, fixtureA, indexA, fixtureB, indexB)
	return this
}

func (this *PolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollidePolygons(manifold, this.FixtureA.GetShape().(*PolygonShape), xfA,
		this.FixtureB.GetShape().(*PolygonShape), xfB)
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
	if Contact_s_initialized == false {
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
