package box2d

// The body type.
// static: zero mass, zero velocity, may be manually moved
// kinematic: zero mass, non-zero velocity set by user, moved by solver
// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
type BodyType byte

const (
	StaticBody BodyType = iota
	KinematicBody
	DynamicBody
)

// A body definition holds all the data needed to construct a rigid body.
// You can safely re-use body definitions. Shapes are added to a body after construction.
type BodyDef struct {
	// The body type: static, kinematic, or dynamic.
	// Note: if a dynamic body would have zero mass, the mass is set to one.
	Type BodyType

	// The world position of the body. Avoid creating bodies at the origin
	// since this can lead to many overlapping shapes.
	Position Vec2

	// The world angle of the body in radians.
	Angle float64

	// The linear velocity of the body's origin in world co-ordinates.
	LinearVelocity Vec2

	// The angular velocity of the body.
	AngularVelocity float64

	// Linear damping is use to reduce the linear velocity. The damping parameter
	// can be larger than 1.0f but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	LinearDamping float64

	// Angular damping is use to reduce the angular velocity. The damping parameter
	// can be larger than 1.0f but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	AngularDamping float64

	// Set this flag to false if this body should never fall asleep. Note that
	// this increases CPU usage.
	AllowSleep bool

	// Is this body initially awake or sleeping?
	Awake bool

	// Should this body be prevented from rotating? Useful for characters.
	FixedRotation bool

	// Is this a fast moving body that should be prevented from tunneling through
	// other moving bodies? Note that all bodies are prevented from tunneling through
	// kinematic and static bodies. This setting is only considered on dynamic bodies.
	// @warning You should use this flag sparingly since it increases processing time.
	Bullet bool

	// Does this body start out active?
	Active bool

	// Use this to store application specific body data.
	UserData interface{}

	// Scale the gravity applied to this body.
	GravityScale float64
}

func MakeBodyDef() BodyDef {
	return BodyDef{
		AllowSleep:   true,
		Awake:        true,
		Type:         StaticBody,
		Active:       true,
		GravityScale: 1.0,
	}
}

func NewBodyDef() *BodyDef {
	bd := MakeBodyDef()
	return &bd
}

// A rigid body. These are created via B2World::CreateBody.
type Body struct {
	xtype BodyType

	flags uint16

	islandIndex int

	xf    Transform // the body origin transform
	sweep Sweep     // the swept motion for CCD

	linearVelocity  Vec2
	angularVelocity float64

	force  Vec2
	torque float64

	world *World
	prev  *Body
	next  *Body

	fixtureList  *Fixture
	fixtureCount int

	jointList   *JointEdge
	contactList *ContactEdge

	mass, invMass float64

	// Rotational inertia about the center of mass.
	I, invI float64

	linearDamping  float64
	angularDamping float64
	gravityScale   float64

	sleepTime float64

	userData interface{}
}

const (
	body_e_islandFlag        uint16 = 0x0001
	body_e_awakeFlag         uint16 = 0x0002
	body_e_autoSleepFlag     uint16 = 0x0004
	body_e_bulletFlag        uint16 = 0x0008
	body_e_fixedRotationFlag uint16 = 0x0010
	body_e_activeFlag        uint16 = 0x0020
	body_e_toiFlag           uint16 = 0x0040
)

func NewBody(bd *BodyDef, world *World) *Body {
	b := new(Body)

	b.flags = 0

	if bd.Bullet {
		b.flags |= body_e_bulletFlag
	}
	if bd.FixedRotation {
		b.flags |= body_e_fixedRotationFlag
	}
	if bd.AllowSleep {
		b.flags |= body_e_autoSleepFlag
	}
	if bd.Awake {
		b.flags |= body_e_awakeFlag
	}
	if bd.Active {
		b.flags |= body_e_activeFlag
	}

	b.world = world

	b.xf.P = bd.Position
	b.xf.Q.Set(bd.Angle)

	b.sweep.LocalCenter.SetZero()
	b.sweep.C0 = b.xf.P
	b.sweep.C = b.xf.P
	b.sweep.A0 = bd.Angle
	b.sweep.A = bd.Angle
	b.sweep.Alpha0 = 0.0

	b.jointList = nil
	b.contactList = nil
	b.prev = nil
	b.next = nil

	b.linearVelocity = bd.LinearVelocity
	b.angularVelocity = bd.AngularVelocity

	b.linearDamping = bd.LinearDamping
	b.angularDamping = bd.AngularDamping
	b.gravityScale = bd.GravityScale

	b.force.SetZero()
	b.torque = 0.0

	b.sleepTime = 0.0

	b.xtype = bd.Type

	if b.xtype == DynamicBody {
		b.mass = 1.0
		b.invMass = 1.0
	} else {
		b.mass = 0.0
		b.invMass = 0.0
	}

	b.I = 0.0
	b.invI = 0.0

	b.userData = bd.UserData

	b.fixtureList = nil
	b.fixtureCount = 0

	return b
}

// Creates a fixture and attach it to this body. Use this function if you need
// to set some fixture parameters, like friction. Otherwise you can create the
// fixture directly from a shape.
// If the density is non-zero, this function automatically updates the mass of the body.
// Contacts are not created until the next time step.
// @param def the fixture definition.
// @warning This function is locked during callbacks.
func (b *Body) CreateFixture(def *FixtureDef) *Fixture {
	if b.world.IsLocked() {
		return nil
	}

	fixture := NewFixture()
	fixture.Create(b, def)

	if (b.flags & body_e_activeFlag) != 0 {
		broadPhase := b.world.contactManager.BroadPhase
		fixture.CreateProxies(broadPhase, b.xf)
	}

	fixture.Next = b.fixtureList
	b.fixtureList = fixture
	b.fixtureCount++

	fixture.Body = b

	// Adjust mass properties if needed.
	if fixture.Density > 0.0 {
		b.ResetMassData()
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	b.world.flags |= world_e_newFixture

	return fixture
}

/// Creates a fixture from a shape and attach it to this body.
/// This is a convenience function. Use b2FixtureDef if you need to set parameters
/// like friction, restitution, user data, or filtering.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// @param shape the shape to be cloned.
/// @param density the shape density (set to zero for static bodies).
/// @warning This function is locked during callbacks.
func (b *Body) CreateFixture2(shape IShape, density float64) *Fixture {
	def := MakeFixtureDef()
	def.Shape = shape
	def.Density = density

	return b.CreateFixture(&def)
}

// Destroy a fixture. This removes the fixture from the broad-phase and
// destroys all contacts associated with this fixture. This will
// automatically adjust the mass of the body if the body is dynamic and the
// fixture has positive density.
// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
// @param fixture the fixture to be removed.
// @warning This function is locked during callbacks.
func (b *Body) DestroyFixture(fixture *Fixture) {

	if fixture == nil {
		return
	}

	//Assert(this.world.IsLocked() == false)
	if b.world.IsLocked() {
		return
	}

	//Assert(fixture.Body == this)

	// Remove the fixture from this body's singly linked list.
	//Assert(this.fixtureCount > 0)
	node := &b.fixtureList
	//found := false
	for *node != nil {
		if *node == fixture {
			*node = fixture.Next
			//found = true
			break
		}

		node = &(*node).Next
	}

	// You tried to remove a shape that is not attached to this body.
	//Assert(found)

	// Destroy any contacts associated with the fixture.
	edge := b.contactList
	for edge != nil {
		c := edge.Contact
		edge = edge.Next

		fixtureA := c.GetFixtureA()
		fixtureB := c.GetFixtureB()

		if fixture == fixtureA || fixture == fixtureB {
			// This destroys the contact and removes it from
			// this body's contact list.
			b.world.contactManager.Destroy(c)
		}
	}

	//b2BlockAllocator* allocator = &m_world->m_blockAllocator;

	if (b.flags & body_e_activeFlag) != 0 {
		broadPhase := b.world.contactManager.BroadPhase
		fixture.DestroyProxies(broadPhase)
	}

	fixture.Body = nil
	fixture.Next = nil
	fixture.Destroy()
	//fixture->~b2Fixture();
	//allocator->Free(fixture, sizeof(b2Fixture));

	b.fixtureCount--

	// Reset the mass data.
	b.ResetMassData()
}

// Set the position of the body's origin and rotation.
// This breaks any contacts and wakes the other bodies.
// Manipulating a body's transform may cause non-physical behavior.
// @param position the world position of the body's local origin.
// @param angle the world rotation in radians.
func (b *Body) SetTransform(position Vec2, angle float64) {
	if b.world.IsLocked() {
		return
	}

	b.xf.Q.Set(angle)
	b.xf.P = position

	b.sweep.C = MulX(b.xf, b.sweep.LocalCenter)
	b.sweep.A = angle

	b.sweep.C0 = b.sweep.C
	b.sweep.A0 = angle

	broadPhase := b.world.contactManager.BroadPhase
	for f := b.fixtureList; f != nil; f = f.Next {
		f.Synchronize(broadPhase, b.xf, b.xf)
	}

	b.world.contactManager.FindNewContacts()
}

// Get the body transform for the body's origin.
// @return the world transform of the body's origin.
func (b *Body) GetTransform() Transform {
	return b.xf
}

// Get the world body origin position.
// @return the world position of the body's origin.
func (b *Body) GetPosition() Vec2 {
	return b.xf.P
}

// Get the angle in radians.
// @return the current world rotation angle in radians.
func (b *Body) GetAngle() float64 {
	return b.sweep.A
}

// Get the local position of the center of mass.
func (b *Body) GetWorldCenter() Vec2 {
	return b.sweep.C
}

// Get the local position of the center of mass.
func (b *Body) GetLocalCenter() Vec2 {
	return b.sweep.LocalCenter
}

// Set the linear velocity of the center of mass.
// @param v the new linear velocity of the center of mass.
func (b *Body) SetLinearVelocity(v Vec2) {
	if b.xtype == StaticBody {
		return
	}

	if DotVV(v, v) > 0.0 {
		b.SetAwake(true)
	}

	b.linearVelocity = v
}

// Get the linear velocity of the center of mass.
// @return the linear velocity of the center of mass.
func (b *Body) GetLinearVelocity() Vec2 {
	return b.linearVelocity
}

/// Set the angular velocity.
/// @param omega the new angular velocity in radians/second.
func (b *Body) SetAngularVelocity(w float64) {
	if b.xtype == StaticBody {
		return
	}

	if w*w > 0.0 {
		b.SetAwake(true)
	}

	b.angularVelocity = w
}

/// Get the angular velocity.
/// @return the angular velocity in radians/second.
func (b *Body) GetAngularVelocity() float64 {
	return b.angularVelocity
}

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
func (b *Body) ApplyForce(force, point Vec2) {
	if b.xtype != DynamicBody {
		return
	}

	if !b.IsAwake() {
		b.SetAwake(true)
	}

	b.force.Add(force)
	b.torque += CrossVV(SubVV(point, b.sweep.C), force)
}

/// Apply a force to the center of mass. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
func (b *Body) ApplyForceToCenter(force Vec2) {
	if b.xtype != DynamicBody {
		return
	}

	if !b.IsAwake() {
		b.SetAwake(true)
	}

	b.force.Add(force)
}

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// This wakes up the body.
/// @param torque about the z-axis (out of the screen), usually in N-m.
func (b *Body) ApplyTorque(torque float64) {
	if b.xtype != DynamicBody {
		return
	}

	if !b.IsAwake() {
		b.SetAwake(true)
	}

	b.torque += torque
}

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
func (b *Body) ApplyLinearImpulse(impulse Vec2, point Vec2) {
	if b.xtype != DynamicBody {
		return
	}

	if !b.IsAwake() {
		b.SetAwake(true)
	}

	b.linearVelocity.Add(MulFV(b.invMass, impulse))
	b.angularVelocity += b.invI * CrossVV(SubVV(point, b.sweep.C), impulse)
}

/// Apply an angular impulse.
/// @param impulse the angular impulse in units of kg*m*m/s
func (b *Body) ApplyAngularImpulse(impulse float64) {
	if b.xtype != DynamicBody {
		return
	}

	if !b.IsAwake() {
		b.SetAwake(true)
	}
	b.angularVelocity += b.invI * impulse
}

/// Get the total mass of the body.
/// @return the mass, usually in kilograms (kg).
func (b *Body) GetMass() float64 {
	return b.mass
}

/// Get the rotational inertia of the body about the local origin.
/// @return the rotational inertia, usually in kg-m^2.
func (b *Body) GetInertia() float64 {
	return b.I + b.mass*DotVV(b.sweep.LocalCenter, b.sweep.LocalCenter)
}

/// Get the mass data of the body.
/// @return a struct containing the mass, inertia and center of the body.
func (b *Body) GetMassData(data *MassData) {
	data.Mass = b.mass
	data.I = b.I + b.mass*DotVV(b.sweep.LocalCenter, b.sweep.LocalCenter)
	data.Center = b.sweep.LocalCenter
}

/// Set the mass properties to override the mass properties of the fixtures.
/// Note that this changes the center of mass position.
/// Note that creating or destroying fixtures can also alter the mass.
/// This function has no effect if the body isn't dynamic.
/// @param massData the mass properties.
func (b *Body) SetMassData(massData *MassData) {
	if b.world.IsLocked() {
		return
	}

	if b.xtype != DynamicBody {
		return
	}

	b.invMass = 0.0
	b.I = 0.0
	b.invI = 0.0

	b.mass = massData.Mass
	if b.mass <= 0.0 {
		b.mass = 1.0
	}

	b.invMass = 1.0 / b.mass

	if massData.I > 0.0 && (b.flags&body_e_fixedRotationFlag) == 0 {
		b.I = massData.I - b.mass*DotVV(massData.Center, massData.Center)
		b.invI = 1.0 / b.I
	}

	// Move center of mass.
	oldCenter := b.sweep.C
	b.sweep.LocalCenter = massData.Center
	b.sweep.C = MulX(b.xf, b.sweep.LocalCenter)
	b.sweep.C0 = b.sweep.C

	// Update center of mass velocity.
	b.linearVelocity.Add(CrossFV(b.angularVelocity, SubVV(b.sweep.C, oldCenter)))
}

// This resets the mass properties to the sum of the mass properties of the fixtures.
// This normally does not need to be called unless you called SetMassData to override
// the mass and you later want to reset the mass.
func (b *Body) ResetMassData() {
	// Compute mass data from shapes. Each shape has its own density.
	b.mass = 0.0
	b.invMass = 0.0
	b.I = 0.0
	b.invI = 0.0
	b.sweep.LocalCenter.SetZero()

	// Static and kinematic bodies have zero mass.
	if b.xtype == StaticBody || b.xtype == KinematicBody {
		b.sweep.C0 = b.xf.P
		b.sweep.C = b.xf.P
		b.sweep.A0 = b.sweep.A
		return
	}

	// Accumulate mass over all fixtures.
	localCenter := Vec2_zero
	for f := b.fixtureList; f != nil; f = f.Next {
		if f.Density == 0.0 {
			continue
		}

		var massData MassData
		f.GetMassData(&massData)
		b.mass += massData.Mass
		localCenter.Add(MulFV(massData.Mass, massData.Center))
		b.I += massData.I
	}

	// Compute center of mass.
	if b.mass > 0.0 {
		b.invMass = 1.0 / b.mass
		localCenter.Mul(b.invMass)
	} else {
		// Force all dynamic bodies to have a positive mass.
		b.mass = 1.0
		b.invMass = 1.0
	}

	if b.I > 0.0 && (b.flags&body_e_fixedRotationFlag) == 0 {
		// Center the inertia about the center of mass.
		b.I -= b.mass * DotVV(localCenter, localCenter)
		b.invI = 1.0 / b.I
	} else {
		b.I = 0.0
		b.invI = 0.0
	}

	// Move center of mass.
	oldCenter := b.sweep.C
	b.sweep.LocalCenter = localCenter
	b.sweep.C = MulX(b.xf, b.sweep.LocalCenter)
	b.sweep.C0 = b.sweep.C

	// Update center of mass velocity.
	b.linearVelocity.Add(CrossFV(b.angularVelocity, SubVV(b.sweep.C, oldCenter)))
}

// Get the world coordinates of a point given the local coordinates.
// @param localPoint a point on the body measured relative the the body's origin.
// @return the same point expressed in world coordinates.
func (b *Body) GetWorldPoint(localPoint Vec2) Vec2 {
	return MulX(b.xf, localPoint)
}

// Get the world coordinates of a vector given the local coordinates.
// @param localVector a vector fixed in the body.
// @return the same vector expressed in world coordinates.
func (b *Body) GetWorldVector(localVector Vec2) Vec2 {
	return MulRV(b.xf.Q, localVector)
}

// Gets a local point relative to the body's origin given a world point.
// @param a point in world coordinates.
// @return the corresponding local point relative to the body's origin.
func (b *Body) GetLocalPoint(worldPoint Vec2) Vec2 {
	return MulXT(b.xf, worldPoint)
}

// Gets a local vector given a world vector.
// @param a vector in world coordinates.
// @return the corresponding local vector.
func (b *Body) GetLocalVector(worldVector Vec2) Vec2 {
	return MulTRV(b.xf.Q, worldVector)
}

// Get the world linear velocity of a world point attached to this body.
// @param a point in world coordinates.
// @return the world velocity of a point.
func (b *Body) GetLinearVelocityFromWorldPoint(worldPoint Vec2) Vec2 {
	return AddVV(b.linearVelocity, CrossFV(b.angularVelocity, SubVV(worldPoint, b.sweep.C)))
}

// Get the world velocity of a local point.
// @param a point in local coordinates.
// @return the world velocity of a point.
func (b *Body) GetLinearVelocityFromLocalPoint(localPoint Vec2) Vec2 {
	return b.GetLinearVelocityFromWorldPoint(b.GetWorldPoint(localPoint))
}

// Get the linear damping of the body.
func (b *Body) GetLinearDamping() float64 {
	return b.linearDamping
}

// Set the linear damping of the body.
func (b *Body) SetLinearDamping(linearDamping float64) {
	b.linearDamping = linearDamping
}

// Get the angular damping of the body.
func (b *Body) GetAngularDamping() float64 {
	return b.angularDamping
}

// Set the angular damping of the body.
func (b *Body) SetAngularDamping(angularDamping float64) {
	b.angularDamping = angularDamping
}

// Get the gravity scale of the body.
func (b *Body) GetGravityScale() float64 {
	return b.gravityScale
}

// Set the gravity scale of the body.
func (b *Body) SetGravityScale(scale float64) {
	b.gravityScale = scale
}

// Set the type of this body. This may alter the mass and velocity.
func (b *Body) SetType(xtype BodyType) {
	if b.world.IsLocked() {
		return
	}

	if b.xtype == xtype {
		return
	}

	b.xtype = xtype

	b.ResetMassData()

	if b.xtype == StaticBody {
		b.linearVelocity.SetZero()
		b.angularVelocity = 0.0
		b.sweep.A0 = b.sweep.A
		b.sweep.C0 = b.sweep.C
		b.synchronizeFixtures()
	}

	b.SetAwake(true)

	b.force.SetZero()
	b.torque = 0.0

	// Since the body type changed, we need to flag contacts for filtering.
	for f := b.fixtureList; f != nil; f = f.Next {
		f.Refilter()
	}
}

// Get the type of this body.
func (b *Body) GetType() BodyType {
	return b.xtype
}

/// Should this body be treated like a bullet for continuous collision detection?
func (b *Body) SetBullet(flag bool) {
	if flag {
		b.flags |= body_e_bulletFlag
	} else {
		b.flags &= ^body_e_bulletFlag
	}
}

// Is this body treated like a bullet for continuous collision detection?
func (b *Body) IsBullet() bool {
	return (b.flags & body_e_bulletFlag) == body_e_bulletFlag
}

// You can disable sleeping on this body. If you disable sleeping, the
// body will be woken.
func (b *Body) SetSleepingAllowed(flag bool) {
	if flag {
		b.flags |= body_e_autoSleepFlag
	} else {
		b.flags &= ^body_e_autoSleepFlag
		b.SetAwake(true)
	}
}

/// Is this body allowed to sleep
func (b *Body) IsSleepingAllowed() bool {
	return (b.flags & body_e_autoSleepFlag) == body_e_autoSleepFlag
}

// Set the sleep state of the body. A sleeping body has very
// low CPU cost.
// @param flag set to true to put body to sleep, false to wake it.
func (b *Body) SetAwake(flag bool) {
	if flag {
		if (b.flags & body_e_awakeFlag) == 0 {
			b.flags |= body_e_awakeFlag
			b.sleepTime = 0.0
		}
	} else {
		b.flags &= ^body_e_awakeFlag
		b.sleepTime = 0.0
		b.linearVelocity.SetZero()
		b.angularVelocity = 0.0
		b.force.SetZero()
		b.torque = 0.0
	}
}

// Get the sleeping state of this body.
// @return true if the body is sleeping.
func (b *Body) IsAwake() bool {
	return (b.flags & body_e_awakeFlag) == body_e_awakeFlag
}

// Set the active state of the body. An inactive body is not
// simulated and cannot be collided with or woken up.
// If you pass a flag of true, all fixtures will be added to the
// broad-phase.
// If you pass a flag of false, all fixtures will be removed from
// the broad-phase and all contacts will be destroyed.
// Fixtures and joints are otherwise unaffected. You may continue
// to create/destroy fixtures and joints on inactive bodies.
// Fixtures on an inactive body are implicitly inactive and will
// not participate in collisions, ray-casts, or queries.
// Joints connected to an inactive body are implicitly inactive.
// An inactive body is still owned by a b2World object and remains
// in the body list.
func (b *Body) SetActive(flag bool) {
	if flag == b.IsActive() {
		return
	}

	if flag {
		b.flags |= body_e_activeFlag

		// Create all proxies.
		broadPhase := b.world.contactManager.BroadPhase
		for f := b.fixtureList; f != nil; f = f.Next {
			f.CreateProxies(broadPhase, b.xf)
		}

		// Contacts are created the next time step.
	} else {
		b.flags &= ^body_e_activeFlag

		// Destroy all proxies.
		broadPhase := b.world.contactManager.BroadPhase
		for f := b.fixtureList; f != nil; f = f.Next {
			f.DestroyProxies(broadPhase)
		}

		// Destroy the attached contacts.
		ce := b.contactList
		for ce != nil {
			ce0 := ce
			ce = ce.Next
			b.world.contactManager.Destroy(ce0.Contact)
		}

		b.contactList = nil
	}
}

// Get the active state of the body.
func (b *Body) IsActive() bool {
	return (b.flags & body_e_activeFlag) == body_e_activeFlag
}

// Set this body to have fixed rotation. This causes the mass
// to be reset.
func (b *Body) SetFixedRotation(flag bool) {
	if flag {
		b.flags |= body_e_fixedRotationFlag
	} else {
		b.flags &= ^body_e_fixedRotationFlag
	}

	b.ResetMassData()
}

// Does this body have fixed rotation?
func (b *Body) IsFixedRotation() bool {
	return (b.flags & body_e_fixedRotationFlag) == body_e_fixedRotationFlag
}

// Get the list of all fixtures attached to this body.
func (b *Body) GetFixtureList() *Fixture {
	return b.fixtureList
}

// Get the list of all joints attached to this body.
func (b *Body) GetJointList() *JointEdge {
	return b.jointList
}

// Get the list of all contacts attached to this body.
// @warning this list changes during the time step and you may
// miss some collisions if you don't use b2ContactListener.
func (b *Body) GetContactList() *ContactEdge {
	return b.contactList
}

// Get the next body in the world's body list.
func (b *Body) GetNext() *Body {
	return b.next
}

// Get the user data pointer that was provided in the body definition.
func (b *Body) GetUserData() interface{} {
	return b.userData
}

// Set the user data. Use this to store your application specific data.
func (b *Body) SetUserData(data interface{}) {
	b.userData = data
}

// Get the parent world of this body.
func (b *Body) GetWorld() *World {
	return b.world
}

// Dump this body to a log file
func (b *Body) Dump() {
	bodyIndex := b.islandIndex

	Log("{\n")
	Log("  b2BodyDef bd;\n")
	Log("  bd.type = b2BodyType(%d);\n", b.xtype)
	Log("  bd.position.Set(%.15f, %.15f);\n", b.xf.P.X, b.xf.P.Y)
	Log("  bd.angle = %.15f;\n", b.sweep.A)
	Log("  bd.linearVelocity.Set(%.15f, %.15f);\n", b.linearVelocity.X, b.linearVelocity.Y)
	Log("  bd.angularVelocity = %.15f;\n", b.angularVelocity)
	Log("  bd.linearDamping = %.15f;\n", b.linearDamping)
	Log("  bd.angularDamping = %.15f;\n", b.angularDamping)
	Log("  bd.allowSleep = bool(%d);\n", b.flags&body_e_autoSleepFlag)
	Log("  bd.awake = bool(%d);\n", b.flags&body_e_awakeFlag)
	Log("  bd.fixedRotation = bool(%d);\n", b.flags&body_e_fixedRotationFlag)
	Log("  bd.bullet = bool(%d);\n", b.flags&body_e_bulletFlag)
	Log("  bd.active = bool(%d);\n", b.flags&body_e_activeFlag)
	Log("  bd.gravityScale = %.15f;\n", b.gravityScale)
	Log("  bodies[%d] = world->CreateBody(&bd);\n", b.islandIndex)
	Log("\n")
	for f := b.fixtureList; f != nil; f = f.Next {
		Log("  {\n")
		f.Dump(bodyIndex)
		Log("  }\n")
	}
	Log("}\n")
}

func (b *Body) synchronizeFixtures() {
	var xf1 Transform
	xf1.Q.Set(b.sweep.A0)
	xf1.P = SubVV(b.sweep.C0, MulRV(xf1.Q, b.sweep.LocalCenter))

	broadPhase := b.world.contactManager.BroadPhase
	for f := b.fixtureList; f != nil; f = f.Next {
		f.Synchronize(broadPhase, xf1, b.xf)
	}
}

func (b *Body) synchronizeTransform() {
	b.xf.Q.Set(b.sweep.A)
	b.xf.P = SubVV(b.sweep.C, MulRV(b.xf.Q, b.sweep.LocalCenter))
}

// This is used to prevent connected bodies from colliding.
// It may lie, depending on the collideConnected flag.
func (b *Body) ShouldCollide(other *Body) bool {
	// At least one body should be dynamic.
	if b.xtype != DynamicBody && other.xtype != DynamicBody {
		return false
	}

	// Does a joint prevent collision?
	for jn := b.jointList; jn != nil; jn = jn.Next {
		if jn.Other == other {
			if !jn.Joint.GetCollideConnected() {
				return false
			}
		}
	}

	return true
}

func (b *Body) Advance(alpha float64) {
	// Advance to the new safe time. This doesn't sync the broad-phase.
	b.sweep.Advance(alpha)
	b.sweep.C = b.sweep.C0
	b.sweep.A = b.sweep.A0
	b.xf.Q.Set(b.sweep.A)
	b.xf.P = SubVV(b.sweep.C, MulRV(b.xf.Q, b.sweep.LocalCenter))
}
