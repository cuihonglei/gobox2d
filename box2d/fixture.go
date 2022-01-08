package box2d

// This holds contact filtering data.
type Filter struct {

	// The collision category bits. Normally you would just set one bit.
	CategoryBits uint16

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	MaskBits uint16

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). Zero means no collision group. Non-zero group
	// filtering always wins against the mask bits.
	GroupIndex int16
}

func NewFilter() *Filter {
	this := new(Filter)
	this.CategoryBits = 0x0001
	this.MaskBits = 0xFFFF
	this.GroupIndex = 0
	return this
}

// A fixture definition is used to create a fixture. This class defines an
// abstract fixture definition. You can reuse fixture definitions safely.
type FixtureDef struct {
	// The shape, this must be set. The shape will be cloned, so you
	// can create the shape on the stack.
	Shape IShape

	// Use this to store application specific fixture data.
	UserData interface{}

	// The friction coefficient, usually in the range [0,1].
	Friction float64

	// The restitution (elasticity) usually in the range [0,1].
	Restitution float64

	// The density, usually in kg/m^2.
	Density float64

	// A sensor shape collects contact information but never generates a collision
	// response.
	IsSensor bool

	// Contact filtering data.
	Filter Filter
}

func NewFixtureDef() *FixtureDef {
	this := new(FixtureDef)
	this.Filter = *NewFilter()
	this.Friction = 0.2
	return this
}

// This proxy is used internally to connect fixtures to the broad-phase.
type FixtureProxy struct {
	AABB       AABB
	Fixture    *Fixture
	ChildIndex int32
	ProxyId    int32
}

// A fixture is used to attach a shape to a body for collision detection. A fixture
// inherits its transform from its parent. Fixtures hold additional non-geometric data
// such as friction, collision filters, etc.
// Fixtures are created via b2Body::CreateFixture.
// @warning you cannot reuse fixtures.
type Fixture struct {
	Density float64

	Next *Fixture
	Body *Body

	Shape IShape

	Friction    float64
	Restitution float64

	Proxies    []FixtureProxy
	ProxyCount int32

	Filter Filter

	IsSensor bool

	UserData interface{}
}

func NewFixture() *Fixture {
	this := new(Fixture)
	return this
}

// We need separation create/destroy functions from the constructor/destructor because
// the destructor cannot access the allocator (no destructor arguments allowed by C++).
func (this *Fixture) Create(body *Body, def *FixtureDef) {
	this.UserData = def.UserData
	this.Friction = def.Friction
	this.Restitution = def.Restitution

	this.Body = body
	this.Next = nil

	this.Filter = def.Filter

	this.IsSensor = def.IsSensor

	this.Shape = def.Shape.Clone()

	// Reserve proxy space
	childCount := this.Shape.GetChildCount()
	this.Proxies = make([]FixtureProxy, childCount, childCount)
	for i := int32(0); i < childCount; i++ {
		this.Proxies[i].Fixture = nil
		this.Proxies[i].ProxyId = BroadPhase_e_nullProxy
	}
	this.ProxyCount = 0

	this.Density = def.Density
}

func (this *Fixture) Destroy() {
	// The proxies must be destroyed before calling this.
	//Assert(this.ProxyCount == 0)

	// Free the proxy array.
	this.Proxies = nil

	// Free the child shape.
	this.Shape = nil
}

// These support body activation/deactivation.
func (this *Fixture) CreateProxies(broadPhase *BroadPhase, xf Transform) {
	// Create proxies in the broad-phase.
	this.ProxyCount = this.Shape.GetChildCount()

	for i := int32(0); i < this.ProxyCount; i++ {
		proxy := &this.Proxies[i]
		this.Shape.ComputeAABB(&proxy.AABB, xf, i)
		proxy.ProxyId = broadPhase.CreateProxy(proxy.AABB, proxy)
		proxy.Fixture = this
		proxy.ChildIndex = i
	}
}

func (this *Fixture) DestroyProxies(broadPhase *BroadPhase) {
	// Destroy proxies in the broad-phase.
	for i := int32(0); i < this.ProxyCount; i++ {
		proxy := &this.Proxies[i]
		broadPhase.DestroyProxy(proxy.ProxyId)
		proxy.ProxyId = BroadPhase_e_nullProxy
	}

	this.ProxyCount = 0
}

func (this *Fixture) Synchronize(broadPhase *BroadPhase, transform1 Transform, transform2 Transform) {
	if this.ProxyCount == 0 {
		return
	}

	for i := int32(0); i < this.ProxyCount; i++ {
		proxy := &this.Proxies[i]

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		var aabb1, aabb2 AABB
		this.Shape.ComputeAABB(&aabb1, transform1, proxy.ChildIndex)
		this.Shape.ComputeAABB(&aabb2, transform2, proxy.ChildIndex)

		proxy.AABB.Combine2(aabb1, aabb2)

		displacement := SubVV(transform2.P, transform1.P)

		broadPhase.MoveProxy(proxy.ProxyId, proxy.AABB, displacement)
	}
}

/// Get the type of the child shape. You can use this to down cast to the concrete shape.
/// @return the shape type.
func (this *Fixture) GetType() ShapeType {
	return this.Shape.GetType()
}

// Get the child shape. You can modify the child shape, however you should not change the
// number of vertices because this will crash some collision caching mechanisms.
// Manipulating the shape may lead to non-physical behavior.
func (this *Fixture) GetShape() IShape {
	return this.Shape
}

/// Set if this fixture is a sensor.
func (this *Fixture) SetSensor(sensor bool) {
	if sensor != this.IsSensor {
		this.Body.SetAwake(true)
		this.IsSensor = sensor
	}
}

/// Is this fixture a sensor (non-solid)?
/// @return the true if the shape is a sensor.
func (this *Fixture) GetSensor() bool {
	return this.IsSensor
}

// Set the contact filtering data. This will not update contacts until the next time
// step when either parent body is active and awake.
// This automatically calls Refilter.
func (this *Fixture) SetFilterData(filter Filter) {
	this.Filter = filter
	this.Refilter()
}

// Get the contact filtering data.
func (this *Fixture) GetFilterData() Filter {
	return this.Filter
}

// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
func (this *Fixture) Refilter() {
	if this.Body == nil {
		return
	}

	// Flag associated contacts for filtering.
	edge := this.Body.GetContactList()
	for edge != nil {
		contact := edge.Contact
		fixtureA := contact.GetFixtureA()
		fixtureB := contact.GetFixtureB()
		if fixtureA == this || fixtureB == this {
			contact.FlagForFiltering()
		}

		edge = edge.Next
	}

	world := this.Body.GetWorld()

	if world == nil {
		return
	}

	// Touch each proxy so that new pairs may be created
	broadPhase := world.contactManager.BroadPhase
	for i := int32(0); i < this.ProxyCount; i++ {
		broadPhase.TouchProxy(this.Proxies[i].ProxyId)
	}
}

// Get the parent body of this fixture. This is NULL if the fixture is not attached.
// @return the parent body.
func (this *Fixture) GetBody() *Body {
	return this.Body
}

// Get the next fixture in the parent body's fixture list.
// @return the next shape.
func (this *Fixture) GetNext() *Fixture {
	return this.Next
}

// Get the user data that was assigned in the fixture definition. Use this to
// store your application specific data.
func (this *Fixture) GetUserData() interface{} {
	return this.UserData
}

// Set the user data. Use this to store your application specific data.
func (this *Fixture) SetUserData(data interface{}) {
	this.UserData = data
}

// Test a point for containment in this fixture.
// @param p a point in world coordinates.
func (this *Fixture) TestPoint(p Vec2) bool {
	return this.Shape.TestPoint(this.Body.GetTransform(), p)
}

// Cast a ray against this shape.
// @param output the ray-cast results.
// @param input the ray-cast input parameters.
func (this *Fixture) RayCast(input RayCastInput, childIndex int32) (output RayCastOutput, ret bool) {
	return this.Shape.RayCast(input, this.Body.GetTransform(), childIndex)
}

// Get the mass data for this fixture. The mass data is based on the density and
// the shape. The rotational inertia is about the shape's origin. This operation
// may be expensive.
func (this *Fixture) GetMassData(massData *MassData) {
	this.Shape.ComputeMass(massData, this.Density)
}

// Set the density of this fixture. This will _not_ automatically adjust the mass
// of the body. You must call b2Body::ResetMassData to update the body's mass.
func (this *Fixture) SetDensity(density float64) {
	this.Density = density
}

// Get the density of this fixture.
func (this *Fixture) GetDensity() float64 {
	return this.Density
}

// Get the coefficient of friction.
func (this *Fixture) GetFriction() float64 {
	return this.Friction
}

// Set the coefficient of friction. This will _not_ change the friction of
// existing contacts.
func (this *Fixture) SetFriction(friction float64) {
	this.Friction = friction
}

// Get the coefficient of restitution.
func (this *Fixture) GetRestitution() float64 {
	return this.Restitution
}

// Set the coefficient of restitution. This will _not_ change the restitution of
// existing contacts.
func (this *Fixture) SetRestitution(restitution float64) {
	this.Restitution = restitution
}

// Get the fixture's AABB. This AABB may be enlarge and/or stale.
// If you need a more accurate AABB, compute it using the shape and
// the body transform.
func (this *Fixture) GetAABB(childIndex int32) AABB {
	return this.Proxies[childIndex].AABB
}

// Dump this fixture to the log file.
func (this *Fixture) Dump(bodyIndex int32) {
	Log("    b2FixtureDef fd;\n")
	Log("    fd.friction = %.15f;\n", this.Friction)
	Log("    fd.restitution = %.15f;\n", this.Restitution)
	Log("    fd.density = %.15f;\n", this.Density)
	Log("    fd.isSensor = bool(%t);\n", this.IsSensor)
	Log("    fd.filter.categoryBits = uint16(%d);\n", this.Filter.CategoryBits)
	Log("    fd.filter.maskBits = uint16(%d);\n", this.Filter.MaskBits)
	Log("    fd.filter.groupIndex = int16(%d);\n", this.Filter.GroupIndex)

	switch this.Shape.GetType() {
	case Shape_e_circle:
		{
			s := this.Shape.(*CircleShape)
			Log("    b2CircleShape shape;\n")
			Log("    shape.m_radius = %.15f;\n", s.Radius)
			Log("    shape.m_p.Set(%.15f, %.15f);\n", s.P.X, s.P.Y)
		}
	case Shape_e_edge:
		{
			s := this.Shape.(*EdgeShape)
			Log("    b2EdgeShape shape;\n")
			Log("    shape.m_radius = %.15f;\n", s.Radius)
			Log("    shape.m_vertex0.Set(%.15f, %.15f);\n", s.Vertex0.X, s.Vertex0.Y)
			Log("    shape.m_vertex1.Set(%.15f, %.15f);\n", s.Vertex1.X, s.Vertex1.Y)
			Log("    shape.m_vertex2.Set(%.15f, %.15f);\n", s.Vertex2.X, s.Vertex2.Y)
			Log("    shape.m_vertex3.Set(%.15f, %.15f);\n", s.Vertex3.X, s.Vertex3.Y)
			Log("    shape.m_hasVertex0 = bool(%t);\n", s.HasVertex0)
			Log("    shape.m_hasVertex3 = bool(%t);\n", s.HasVertex3)
		}
	case Shape_e_polygon:
		{
			s := this.Shape.(*PolygonShape)
			Log("    b2PolygonShape shape;\n")
			Log("    b2Vec2 vs[%d];\n", MaxPolygonVertices)
			for i := int32(0); i < s.VertexCount; i++ {
				Log("    vs[%d].Set(%.15f, %.15f);\n", i, s.Vertices[i].X, s.Vertices[i].Y)
			}
			Log("    shape.Set(vs, %d);\n", s.VertexCount)
		}
	case Shape_e_chain:
		{
			s := this.Shape.(*ChainShape)
			Log("    b2ChainShape shape;\n")
			Log("    b2Vec2 vs[%d];\n", len(s.Vertices))
			for i := int32(0); i < int32(len(s.Vertices)); i++ {
				Log("    vs[%d].Set(%.15f, %.15f);\n", i, s.Vertices[i].X, s.Vertices[i].Y)
			}
			Log("    shape.CreateChain(vs, %d);\n", len(s.Vertices))
			Log("    shape.m_prevVertex.Set(%.15f, %.15f);\n", s.PrevVertex.X, s.PrevVertex.Y)
			Log("    shape.m_nextVertex.Set(%.15f, %.15f);\n", s.NextVertex.X, s.NextVertex.Y)
			Log("    shape.m_hasPrevVertex = bool(%t);\n", s.HasPrevVertex)
			Log("    shape.m_hasNextVertex = bool(%t);\n", s.HasNextVertex)
		}
	default:
		return
	}

	Log("\n")
	Log("    fd.shape = &shape;\n")
	Log("\n")
	Log("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex)
}
