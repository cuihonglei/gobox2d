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

func MakeFilter() Filter {
	return Filter{
		CategoryBits: 0x0001,
		MaskBits:     0xFFFF,
		GroupIndex:   0,
	}
}

func NewFilter() *Filter {
	f := MakeFilter()
	return &f
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

func MakeFixtureDef() FixtureDef {
	return FixtureDef{
		Filter:   MakeFilter(),
		Friction: 0.2,
	}
}

func NewFixtureDef() *FixtureDef {
	fd := MakeFixtureDef()
	return &fd
}

// This proxy is used internally to connect fixtures to the broad-phase.
type FixtureProxy struct {
	AABB       AABB
	Fixture    *Fixture
	ChildIndex int
	ProxyId    int
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
	ProxyCount int

	Filter Filter

	IsSensor bool

	UserData interface{}
}

func NewFixture() *Fixture {
	f := new(Fixture)
	return f
}

// We need separation create/destroy functions from the constructor/destructor because
// the destructor cannot access the allocator (no destructor arguments allowed by C++).
func (f *Fixture) Create(body *Body, def *FixtureDef) {
	f.UserData = def.UserData
	f.Friction = def.Friction
	f.Restitution = def.Restitution

	f.Body = body
	f.Next = nil

	f.Filter = def.Filter

	f.IsSensor = def.IsSensor

	f.Shape = def.Shape.Clone()

	// Reserve proxy space
	childCount := f.Shape.GetChildCount()
	f.Proxies = make([]FixtureProxy, childCount)
	for i := 0; i < childCount; i++ {
		f.Proxies[i].Fixture = nil
		f.Proxies[i].ProxyId = BroadPhase_e_nullProxy
	}
	f.ProxyCount = 0

	f.Density = def.Density
}

func (f *Fixture) Destroy() {
	// The proxies must be destroyed before calling this.
	//Assert(this.ProxyCount == 0)

	// Free the proxy array.
	f.Proxies = nil

	// Free the child shape.
	f.Shape = nil
}

// These support body activation/deactivation.
func (f *Fixture) CreateProxies(broadPhase *BroadPhase, xf Transform) {
	// Create proxies in the broad-phase.
	f.ProxyCount = f.Shape.GetChildCount()

	for i := 0; i < f.ProxyCount; i++ {
		proxy := &f.Proxies[i]
		f.Shape.ComputeAABB(&proxy.AABB, xf, i)
		proxy.ProxyId = broadPhase.CreateProxy(proxy.AABB, proxy)
		proxy.Fixture = f
		proxy.ChildIndex = i
	}
}

func (f *Fixture) DestroyProxies(broadPhase *BroadPhase) {
	// Destroy proxies in the broad-phase.
	for i := 0; i < f.ProxyCount; i++ {
		proxy := &f.Proxies[i]
		broadPhase.DestroyProxy(proxy.ProxyId)
		proxy.ProxyId = BroadPhase_e_nullProxy
	}

	f.ProxyCount = 0
}

func (f *Fixture) Synchronize(broadPhase *BroadPhase, transform1 Transform, transform2 Transform) {
	if f.ProxyCount == 0 {
		return
	}

	for i := 0; i < f.ProxyCount; i++ {
		proxy := &f.Proxies[i]

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		var aabb1, aabb2 AABB
		f.Shape.ComputeAABB(&aabb1, transform1, proxy.ChildIndex)
		f.Shape.ComputeAABB(&aabb2, transform2, proxy.ChildIndex)

		proxy.AABB.Combine2(aabb1, aabb2)

		displacement := SubVV(transform2.P, transform1.P)

		broadPhase.MoveProxy(proxy.ProxyId, proxy.AABB, displacement)
	}
}

/// Get the type of the child shape. You can use this to down cast to the concrete shape.
/// @return the shape type.
func (f *Fixture) GetType() ShapeType {
	return f.Shape.GetType()
}

// Get the child shape. You can modify the child shape, however you should not change the
// number of vertices because this will crash some collision caching mechanisms.
// Manipulating the shape may lead to non-physical behavior.
func (f *Fixture) GetShape() IShape {
	return f.Shape
}

/// Set if this fixture is a sensor.
func (f *Fixture) SetSensor(sensor bool) {
	if sensor != f.IsSensor {
		f.Body.SetAwake(true)
		f.IsSensor = sensor
	}
}

/// Is this fixture a sensor (non-solid)?
/// @return the true if the shape is a sensor.
func (f *Fixture) GetSensor() bool {
	return f.IsSensor
}

// Set the contact filtering data. This will not update contacts until the next time
// step when either parent body is active and awake.
// This automatically calls Refilter.
func (f *Fixture) SetFilterData(filter Filter) {
	f.Filter = filter
	f.Refilter()
}

// Get the contact filtering data.
func (f *Fixture) GetFilterData() Filter {
	return f.Filter
}

// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
func (f *Fixture) Refilter() {
	if f.Body == nil {
		return
	}

	// Flag associated contacts for filtering.
	edge := f.Body.GetContactList()
	for edge != nil {
		contact := edge.Contact
		fixtureA := contact.GetFixtureA()
		fixtureB := contact.GetFixtureB()
		if fixtureA == f || fixtureB == f {
			contact.FlagForFiltering()
		}

		edge = edge.Next
	}

	world := f.Body.GetWorld()

	if world == nil {
		return
	}

	// Touch each proxy so that new pairs may be created
	broadPhase := world.contactManager.BroadPhase
	for i := 0; i < f.ProxyCount; i++ {
		broadPhase.TouchProxy(f.Proxies[i].ProxyId)
	}
}

// Get the parent body of this fixture. This is NULL if the fixture is not attached.
// @return the parent body.
func (f *Fixture) GetBody() *Body {
	return f.Body
}

// Get the next fixture in the parent body's fixture list.
// @return the next shape.
func (f *Fixture) GetNext() *Fixture {
	return f.Next
}

// Get the user data that was assigned in the fixture definition. Use this to
// store your application specific data.
func (f *Fixture) GetUserData() interface{} {
	return f.UserData
}

// Set the user data. Use this to store your application specific data.
func (f *Fixture) SetUserData(data interface{}) {
	f.UserData = data
}

// Test a point for containment in this fixture.
// @param p a point in world coordinates.
func (f *Fixture) TestPoint(p Vec2) bool {
	return f.Shape.TestPoint(f.Body.GetTransform(), p)
}

// Cast a ray against this shape.
// @param output the ray-cast results.
// @param input the ray-cast input parameters.
func (f *Fixture) RayCast(input RayCastInput, childIndex int) (output RayCastOutput, ret bool) {
	return f.Shape.RayCast(input, f.Body.GetTransform(), childIndex)
}

// Get the mass data for this fixture. The mass data is based on the density and
// the shape. The rotational inertia is about the shape's origin. This operation
// may be expensive.
func (f *Fixture) GetMassData(massData *MassData) {
	f.Shape.ComputeMass(massData, f.Density)
}

// Set the density of this fixture. This will _not_ automatically adjust the mass
// of the body. You must call b2Body::ResetMassData to update the body's mass.
func (f *Fixture) SetDensity(density float64) {
	f.Density = density
}

// Get the density of this fixture.
func (f *Fixture) GetDensity() float64 {
	return f.Density
}

// Get the coefficient of friction.
func (f *Fixture) GetFriction() float64 {
	return f.Friction
}

// Set the coefficient of friction. This will _not_ change the friction of
// existing contacts.
func (f *Fixture) SetFriction(friction float64) {
	f.Friction = friction
}

// Get the coefficient of restitution.
func (f *Fixture) GetRestitution() float64 {
	return f.Restitution
}

// Set the coefficient of restitution. This will _not_ change the restitution of
// existing contacts.
func (f *Fixture) SetRestitution(restitution float64) {
	f.Restitution = restitution
}

// Get the fixture's AABB. This AABB may be enlarge and/or stale.
// If you need a more accurate AABB, compute it using the shape and
// the body transform.
func (f *Fixture) GetAABB(childIndex int) AABB {
	return f.Proxies[childIndex].AABB
}

// Dump this fixture to the log file.
func (f *Fixture) Dump(bodyIndex int) {
	Log("    b2FixtureDef fd;\n")
	Log("    fd.friction = %.15f;\n", f.Friction)
	Log("    fd.restitution = %.15f;\n", f.Restitution)
	Log("    fd.density = %.15f;\n", f.Density)
	Log("    fd.isSensor = bool(%t);\n", f.IsSensor)
	Log("    fd.filter.categoryBits = uint16(%d);\n", f.Filter.CategoryBits)
	Log("    fd.filter.maskBits = uint16(%d);\n", f.Filter.MaskBits)
	Log("    fd.filter.groupIndex = int16(%d);\n", f.Filter.GroupIndex)

	switch f.Shape.GetType() {
	case Shape_e_circle:
		{
			s := f.Shape.(*CircleShape)
			Log("    b2CircleShape shape;\n")
			Log("    shape.m_radius = %.15f;\n", s.Radius)
			Log("    shape.m_p.Set(%.15f, %.15f);\n", s.P.X, s.P.Y)
		}
	case Shape_e_edge:
		{
			s := f.Shape.(*EdgeShape)
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
			s := f.Shape.(*PolygonShape)
			Log("    b2PolygonShape shape;\n")
			Log("    b2Vec2 vs[%d];\n", MaxPolygonVertices)
			for i := 0; i < s.VertexCount; i++ {
				Log("    vs[%d].Set(%.15f, %.15f);\n", i, s.Vertices[i].X, s.Vertices[i].Y)
			}
			Log("    shape.Set(vs, %d);\n", s.VertexCount)
		}
	case Shape_e_chain:
		{
			s := f.Shape.(*ChainShape)
			Log("    b2ChainShape shape;\n")
			Log("    b2Vec2 vs[%d];\n", len(s.Vertices))
			for i := 0; i < len(s.Vertices); i++ {
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
