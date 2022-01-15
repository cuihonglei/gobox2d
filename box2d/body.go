package box2d

// The body type.
// static: zero mass, zero velocity, may be manually moved
// kinematic: zero mass, non-zero velocity set by user, moved by solver
// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
const (
	StaticBody = iota
	KinematicBody
	DynamicBody

	// TODO_ERIN
	//BulletBody
)

type BodyType byte

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
	itype BodyType

	flags uint16

	islandIndex int32

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
	fixtureCount int32

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
	this := new(Body)

	this.flags = 0

	if bd.Bullet {
		this.flags |= body_e_bulletFlag
	}
	if bd.FixedRotation {
		this.flags |= body_e_fixedRotationFlag
	}
	if bd.AllowSleep {
		this.flags |= body_e_autoSleepFlag
	}
	if bd.Awake {
		this.flags |= body_e_awakeFlag
	}
	if bd.Active {
		this.flags |= body_e_activeFlag
	}

	this.world = world

	this.xf.P = bd.Position
	this.xf.Q.Set(bd.Angle)

	this.sweep.LocalCenter.SetZero()
	this.sweep.C0 = this.xf.P
	this.sweep.C = this.xf.P
	this.sweep.A0 = bd.Angle
	this.sweep.A = bd.Angle
	this.sweep.Alpha0 = 0.0

	this.jointList = nil
	this.contactList = nil
	this.prev = nil
	this.next = nil

	this.linearVelocity = bd.LinearVelocity
	this.angularVelocity = bd.AngularVelocity

	this.linearDamping = bd.LinearDamping
	this.angularDamping = bd.AngularDamping
	this.gravityScale = bd.GravityScale

	this.force.SetZero()
	this.torque = 0.0

	this.sleepTime = 0.0

	this.itype = bd.Type

	if this.itype == DynamicBody {
		this.mass = 1.0
		this.invMass = 1.0
	} else {
		this.mass = 0.0
		this.invMass = 0.0
	}

	this.I = 0.0
	this.invI = 0.0

	this.userData = bd.UserData

	this.fixtureList = nil
	this.fixtureCount = 0

	return this
}

// Creates a fixture and attach it to this body. Use this function if you need
// to set some fixture parameters, like friction. Otherwise you can create the
// fixture directly from a shape.
// If the density is non-zero, this function automatically updates the mass of the body.
// Contacts are not created until the next time step.
// @param def the fixture definition.
// @warning This function is locked during callbacks.
func (this *Body) CreateFixture(def *FixtureDef) *Fixture {
	if this.world.IsLocked() {
		return nil
	}

	fixture := NewFixture()
	fixture.Create(this, def)

	if (this.flags & body_e_activeFlag) != 0 {
		broadPhase := this.world.contactManager.BroadPhase
		fixture.CreateProxies(broadPhase, this.xf)
	}

	fixture.Next = this.fixtureList
	this.fixtureList = fixture
	this.fixtureCount++

	fixture.Body = this

	// Adjust mass properties if needed.
	if fixture.Density > 0.0 {
		this.ResetMassData()
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	this.world.flags |= world_e_newFixture

	return fixture
}

/// Creates a fixture from a shape and attach it to this body.
/// This is a convenience function. Use b2FixtureDef if you need to set parameters
/// like friction, restitution, user data, or filtering.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// @param shape the shape to be cloned.
/// @param density the shape density (set to zero for static bodies).
/// @warning This function is locked during callbacks.
func (this *Body) CreateFixture2(shape IShape, density float64) *Fixture {
	def := NewFixtureDef()
	def.Shape = shape
	def.Density = density

	return this.CreateFixture(def)
}

// Destroy a fixture. This removes the fixture from the broad-phase and
// destroys all contacts associated with this fixture. This will
// automatically adjust the mass of the body if the body is dynamic and the
// fixture has positive density.
// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
// @param fixture the fixture to be removed.
// @warning This function is locked during callbacks.
func (this *Body) DestroyFixture(fixture *Fixture) {
	//Assert(this.world.IsLocked() == false)
	if this.world.IsLocked() {
		return
	}

	//Assert(fixture.Body == this)

	// Remove the fixture from this body's singly linked list.
	//Assert(this.fixtureCount > 0)
	//node := &this.fixtureList
	//found := false
	//for *node != nil {
	//	if *node == fixture {
	//		*node = fixture.Next
	//		found = true
	//		break
	//	}

	//	node = &(*node).Next
	//}

	// You tried to remove a shape that is not attached to this body.
	//Assert(found)

	// Destroy any contacts associated with the fixture.
	edge := this.contactList
	for edge != nil {
		c := edge.Contact
		edge = edge.Next

		fixtureA := c.GetFixtureA()
		fixtureB := c.GetFixtureB()

		if fixture == fixtureA || fixture == fixtureB {
			// This destroys the contact and removes it from
			// this body's contact list.
			this.world.contactManager.Destroy(c)
		}
	}

	//b2BlockAllocator* allocator = &m_world->m_blockAllocator;

	if (this.flags & body_e_activeFlag) != 0 {
		broadPhase := this.world.contactManager.BroadPhase
		fixture.DestroyProxies(broadPhase)
	}

	fixture.Destroy()
	fixture.Body = nil
	fixture.Next = nil
	//fixture->~b2Fixture();
	//allocator->Free(fixture, sizeof(b2Fixture));

	this.fixtureCount--

	// Reset the mass data.
	this.ResetMassData()
}

// Set the position of the body's origin and rotation.
// This breaks any contacts and wakes the other bodies.
// Manipulating a body's transform may cause non-physical behavior.
// @param position the world position of the body's local origin.
// @param angle the world rotation in radians.
func (this *Body) SetTransform(position Vec2, angle float64) {
	if this.world.IsLocked() {
		return
	}

	this.xf.Q.Set(angle)
	this.xf.P = position

	this.sweep.C = MulX(this.xf, this.sweep.LocalCenter)
	this.sweep.A = angle

	this.sweep.C0 = this.sweep.C
	this.sweep.A0 = angle

	broadPhase := this.world.contactManager.BroadPhase
	for f := this.fixtureList; f != nil; f = f.Next {
		f.Synchronize(broadPhase, this.xf, this.xf)
	}

	this.world.contactManager.FindNewContacts()
}

// Get the body transform for the body's origin.
// @return the world transform of the body's origin.
func (this *Body) GetTransform() Transform {
	return this.xf
}

// Get the world body origin position.
// @return the world position of the body's origin.
func (this *Body) GetPosition() Vec2 {
	return this.xf.P
}

// Get the angle in radians.
// @return the current world rotation angle in radians.
func (this *Body) GetAngle() float64 {
	return this.sweep.A
}

// Get the local position of the center of mass.
func (this *Body) GetWorldCenter() Vec2 {
	return this.sweep.C
}

// Get the local position of the center of mass.
func (this *Body) GetLocalCenter() Vec2 {
	return this.sweep.LocalCenter
}

// Set the linear velocity of the center of mass.
// @param v the new linear velocity of the center of mass.
func (this *Body) SetLinearVelocity(v Vec2) {
	if this.itype == StaticBody {
		return
	}

	if DotVV(v, v) > 0.0 {
		this.SetAwake(true)
	}

	this.linearVelocity = v
}

// Get the linear velocity of the center of mass.
// @return the linear velocity of the center of mass.
func (this *Body) GetLinearVelocity() Vec2 {
	return this.linearVelocity
}

/// Set the angular velocity.
/// @param omega the new angular velocity in radians/second.
func (this *Body) SetAngularVelocity(w float64) {
	if this.itype == StaticBody {
		return
	}

	if w*w > 0.0 {
		this.SetAwake(true)
	}

	this.angularVelocity = w
}

/// Get the angular velocity.
/// @return the angular velocity in radians/second.
func (this *Body) GetAngularVelocity() float64 {
	return this.angularVelocity
}

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
func (this *Body) ApplyForce(force, point Vec2) {
	if this.itype != DynamicBody {
		return
	}

	if !this.IsAwake() {
		this.SetAwake(true)
	}

	this.force.Add(force)
	this.torque += CrossVV(SubVV(point, this.sweep.C), force)
}

/// Apply a force to the center of mass. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
func (this *Body) ApplyForceToCenter(force Vec2) {
	if this.itype != DynamicBody {
		return
	}

	if !this.IsAwake() {
		this.SetAwake(true)
	}

	this.force.Add(force)
}

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// This wakes up the body.
/// @param torque about the z-axis (out of the screen), usually in N-m.
func (this *Body) ApplyTorque(torque float64) {
	if this.itype != DynamicBody {
		return
	}

	if !this.IsAwake() {
		this.SetAwake(true)
	}

	this.torque += torque
}

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
func (this *Body) ApplyLinearImpulse(impulse Vec2, point Vec2) {
	if this.itype != DynamicBody {
		return
	}

	if !this.IsAwake() {
		this.SetAwake(true)
	}

	this.linearVelocity.Add(MulFV(this.invMass, impulse))
	this.angularVelocity += this.invI * CrossVV(SubVV(point, this.sweep.C), impulse)
}

/// Apply an angular impulse.
/// @param impulse the angular impulse in units of kg*m*m/s
func (this *Body) ApplyAngularImpulse(impulse float64) {
	if this.itype != DynamicBody {
		return
	}

	if !this.IsAwake() {
		this.SetAwake(true)
	}
	this.angularVelocity += this.invI * impulse
}

/// Get the total mass of the body.
/// @return the mass, usually in kilograms (kg).
func (this *Body) GetMass() float64 {
	return this.mass
}

/// Get the rotational inertia of the body about the local origin.
/// @return the rotational inertia, usually in kg-m^2.
func (this *Body) GetInertia() float64 {
	return this.I + this.mass*DotVV(this.sweep.LocalCenter, this.sweep.LocalCenter)
}

/// Get the mass data of the body.
/// @return a struct containing the mass, inertia and center of the body.
func (this *Body) GetMassData(data *MassData) {
	data.Mass = this.mass
	data.I = this.I + this.mass*DotVV(this.sweep.LocalCenter, this.sweep.LocalCenter)
	data.Center = this.sweep.LocalCenter
}

/// Set the mass properties to override the mass properties of the fixtures.
/// Note that this changes the center of mass position.
/// Note that creating or destroying fixtures can also alter the mass.
/// This function has no effect if the body isn't dynamic.
/// @param massData the mass properties.
func (this *Body) SetMassData(massData *MassData) {
	if this.world.IsLocked() {
		return
	}

	if this.itype != DynamicBody {
		return
	}

	this.invMass = 0.0
	this.I = 0.0
	this.invI = 0.0

	this.mass = massData.Mass
	if this.mass <= 0.0 {
		this.mass = 1.0
	}

	this.invMass = 1.0 / this.mass

	if massData.I > 0.0 && (this.flags&body_e_fixedRotationFlag) == 0 {
		this.I = massData.I - this.mass*DotVV(massData.Center, massData.Center)
		this.invI = 1.0 / this.I
	}

	// Move center of mass.
	oldCenter := this.sweep.C
	this.sweep.LocalCenter = massData.Center
	this.sweep.C = MulX(this.xf, this.sweep.LocalCenter)
	this.sweep.C0 = this.sweep.C

	// Update center of mass velocity.
	this.linearVelocity.Add(CrossFV(this.angularVelocity, SubVV(this.sweep.C, oldCenter)))
}

// This resets the mass properties to the sum of the mass properties of the fixtures.
// This normally does not need to be called unless you called SetMassData to override
// the mass and you later want to reset the mass.
func (this *Body) ResetMassData() {
	// Compute mass data from shapes. Each shape has its own density.
	this.mass = 0.0
	this.invMass = 0.0
	this.I = 0.0
	this.invI = 0.0
	this.sweep.LocalCenter.SetZero()

	// Static and kinematic bodies have zero mass.
	if this.itype == StaticBody || this.itype == KinematicBody {
		this.sweep.C0 = this.xf.P
		this.sweep.C = this.xf.P
		this.sweep.A0 = this.sweep.A
		return
	}

	// Accumulate mass over all fixtures.
	localCenter := Vec2_zero
	for f := this.fixtureList; f != nil; f = f.Next {
		if f.Density == 0.0 {
			continue
		}

		var massData MassData
		f.GetMassData(&massData)
		this.mass += massData.Mass
		localCenter.Add(MulFV(massData.Mass, massData.Center))
		this.I += massData.I
	}

	// Compute center of mass.
	if this.mass > 0.0 {
		this.invMass = 1.0 / this.mass
		localCenter.Mul(this.invMass)
	} else {
		// Force all dynamic bodies to have a positive mass.
		this.mass = 1.0
		this.invMass = 1.0
	}

	if this.I > 0.0 && (this.flags&body_e_fixedRotationFlag) == 0 {
		// Center the inertia about the center of mass.
		this.I -= this.mass * DotVV(localCenter, localCenter)
		this.invI = 1.0 / this.I
	} else {
		this.I = 0.0
		this.invI = 0.0
	}

	// Move center of mass.
	oldCenter := this.sweep.C
	this.sweep.LocalCenter = localCenter
	this.sweep.C = MulX(this.xf, this.sweep.LocalCenter)
	this.sweep.C0 = this.sweep.C

	// Update center of mass velocity.
	this.linearVelocity.Add(CrossFV(this.angularVelocity, SubVV(this.sweep.C, oldCenter)))
}

// Get the world coordinates of a point given the local coordinates.
// @param localPoint a point on the body measured relative the the body's origin.
// @return the same point expressed in world coordinates.
func (this *Body) GetWorldPoint(localPoint Vec2) Vec2 {
	return MulX(this.xf, localPoint)
}

// Get the world coordinates of a vector given the local coordinates.
// @param localVector a vector fixed in the body.
// @return the same vector expressed in world coordinates.
func (this *Body) GetWorldVector(localVector Vec2) Vec2 {
	return MulRV(this.xf.Q, localVector)
}

// Gets a local point relative to the body's origin given a world point.
// @param a point in world coordinates.
// @return the corresponding local point relative to the body's origin.
func (this *Body) GetLocalPoint(worldPoint Vec2) Vec2 {
	return MulXT(this.xf, worldPoint)
}

// Gets a local vector given a world vector.
// @param a vector in world coordinates.
// @return the corresponding local vector.
func (this *Body) GetLocalVector(worldVector Vec2) Vec2 {
	return MulTRV(this.xf.Q, worldVector)
}

// Get the world linear velocity of a world point attached to this body.
// @param a point in world coordinates.
// @return the world velocity of a point.
func (this *Body) GetLinearVelocityFromWorldPoint(worldPoint Vec2) Vec2 {
	return AddVV(this.linearVelocity, CrossFV(this.angularVelocity, SubVV(worldPoint, this.sweep.C)))
}

// Get the world velocity of a local point.
// @param a point in local coordinates.
// @return the world velocity of a point.
func (this *Body) GetLinearVelocityFromLocalPoint(localPoint Vec2) Vec2 {
	return this.GetLinearVelocityFromWorldPoint(this.GetWorldPoint(localPoint))
}

// Get the linear damping of the body.
func (this *Body) GetLinearDamping() float64 {
	return this.linearDamping
}

// Set the linear damping of the body.
func (this *Body) SetLinearDamping(linearDamping float64) {
	this.linearDamping = linearDamping
}

// Get the angular damping of the body.
func (this *Body) GetAngularDamping() float64 {
	return this.angularDamping
}

// Set the angular damping of the body.
func (this *Body) SetAngularDamping(angularDamping float64) {
	this.angularDamping = angularDamping
}

// Get the gravity scale of the body.
func (this *Body) GetGravityScale() float64 {
	return this.gravityScale
}

// Set the gravity scale of the body.
func (this *Body) SetGravityScale(scale float64) {
	this.gravityScale = scale
}

// Set the type of this body. This may alter the mass and velocity.
func (this *Body) SetType(itype BodyType) {
	if this.world.IsLocked() {
		return
	}

	if this.itype == itype {
		return
	}

	this.itype = itype

	this.ResetMassData()

	if this.itype == StaticBody {
		this.linearVelocity.SetZero()
		this.angularVelocity = 0.0
		this.sweep.A0 = this.sweep.A
		this.sweep.C0 = this.sweep.C
		this.synchronizeFixtures()
	}

	this.SetAwake(true)

	this.force.SetZero()
	this.torque = 0.0

	// Since the body type changed, we need to flag contacts for filtering.
	for f := this.fixtureList; f != nil; f = f.Next {
		f.Refilter()
	}
}

// Get the type of this body.
func (this *Body) GetType() BodyType {
	return this.itype
}

/// Should this body be treated like a bullet for continuous collision detection?
func (this *Body) SetBullet(flag bool) {
	if flag {
		this.flags |= body_e_bulletFlag
	} else {
		this.flags &= ^body_e_bulletFlag
	}
}

// Is this body treated like a bullet for continuous collision detection?
func (this *Body) IsBullet() bool {
	return (this.flags & body_e_bulletFlag) == body_e_bulletFlag
}

// You can disable sleeping on this body. If you disable sleeping, the
// body will be woken.
func (this *Body) SetSleepingAllowed(flag bool) {
	if flag {
		this.flags |= body_e_autoSleepFlag
	} else {
		this.flags &= ^body_e_autoSleepFlag
		this.SetAwake(true)
	}
}

/// Is this body allowed to sleep
func (this *Body) IsSleepingAllowed() bool {
	return (this.flags & body_e_autoSleepFlag) == body_e_autoSleepFlag
}

// Set the sleep state of the body. A sleeping body has very
// low CPU cost.
// @param flag set to true to put body to sleep, false to wake it.
func (this *Body) SetAwake(flag bool) {
	if flag {
		if (this.flags & body_e_awakeFlag) == 0 {
			this.flags |= body_e_awakeFlag
			this.sleepTime = 0.0
		}
	} else {
		this.flags &= ^body_e_awakeFlag
		this.sleepTime = 0.0
		this.linearVelocity.SetZero()
		this.angularVelocity = 0.0
		this.force.SetZero()
		this.torque = 0.0
	}
}

// Get the sleeping state of this body.
// @return true if the body is sleeping.
func (this *Body) IsAwake() bool {
	return (this.flags & body_e_awakeFlag) == body_e_awakeFlag
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
func (this *Body) SetActive(flag bool) {
	if flag == this.IsActive() {
		return
	}

	if flag {
		this.flags |= body_e_activeFlag

		// Create all proxies.
		broadPhase := this.world.contactManager.BroadPhase
		for f := this.fixtureList; f != nil; f = f.Next {
			f.CreateProxies(broadPhase, this.xf)
		}

		// Contacts are created the next time step.
	} else {
		this.flags &= ^body_e_activeFlag

		// Destroy all proxies.
		broadPhase := this.world.contactManager.BroadPhase
		for f := this.fixtureList; f != nil; f = f.Next {
			f.DestroyProxies(broadPhase)
		}

		// Destroy the attached contacts.
		ce := this.contactList
		for ce != nil {
			ce0 := ce
			ce = ce.Next
			this.world.contactManager.Destroy(ce0.Contact)
		}

		this.contactList = nil
	}
}

// Get the active state of the body.
func (this *Body) IsActive() bool {
	return (this.flags & body_e_activeFlag) == body_e_activeFlag
}

// Set this body to have fixed rotation. This causes the mass
// to be reset.
func (this *Body) SetFixedRotation(flag bool) {
	if flag {
		this.flags |= body_e_fixedRotationFlag
	} else {
		this.flags &= ^body_e_fixedRotationFlag
	}

	this.ResetMassData()
}

// Does this body have fixed rotation?
func (this *Body) IsFixedRotation() bool {
	return (this.flags & body_e_fixedRotationFlag) == body_e_fixedRotationFlag
}

// Get the list of all fixtures attached to this body.
func (this *Body) GetFixtureList() *Fixture {
	return this.fixtureList
}

// Get the list of all joints attached to this body.
func (this *Body) GetJointList() *JointEdge {
	return this.jointList
}

// Get the list of all contacts attached to this body.
// @warning this list changes during the time step and you may
// miss some collisions if you don't use b2ContactListener.
func (this *Body) GetContactList() *ContactEdge {
	return this.contactList
}

// Get the next body in the world's body list.
func (this *Body) GetNext() *Body {
	return this.next
}

// Get the user data pointer that was provided in the body definition.
func (this *Body) GetUserData() interface{} {
	return this.userData
}

// Set the user data. Use this to store your application specific data.
func (this *Body) SetUserData(data interface{}) {
	this.userData = data
}

// Get the parent world of this body.
func (this *Body) GetWorld() *World {
	return this.world
}

// Dump this body to a log file
func (this *Body) Dump() {
	bodyIndex := this.islandIndex

	Log("{\n")
	Log("  b2BodyDef bd;\n")
	Log("  bd.type = b2BodyType(%d);\n", this.itype)
	Log("  bd.position.Set(%.15f, %.15f);\n", this.xf.P.X, this.xf.P.Y)
	Log("  bd.angle = %.15f;\n", this.sweep.A)
	Log("  bd.linearVelocity.Set(%.15f, %.15f);\n", this.linearVelocity.X, this.linearVelocity.Y)
	Log("  bd.angularVelocity = %.15f;\n", this.angularVelocity)
	Log("  bd.linearDamping = %.15f;\n", this.linearDamping)
	Log("  bd.angularDamping = %.15f;\n", this.angularDamping)
	Log("  bd.allowSleep = bool(%d);\n", this.flags&body_e_autoSleepFlag)
	Log("  bd.awake = bool(%d);\n", this.flags&body_e_awakeFlag)
	Log("  bd.fixedRotation = bool(%d);\n", this.flags&body_e_fixedRotationFlag)
	Log("  bd.bullet = bool(%d);\n", this.flags&body_e_bulletFlag)
	Log("  bd.active = bool(%d);\n", this.flags&body_e_activeFlag)
	Log("  bd.gravityScale = %.15f;\n", this.gravityScale)
	Log("  bodies[%d] = world->CreateBody(&bd);\n", this.islandIndex)
	Log("\n")
	for f := this.fixtureList; f != nil; f = f.Next {
		Log("  {\n")
		f.Dump(bodyIndex)
		Log("  }\n")
	}
	Log("}\n")
}

func (this *Body) synchronizeFixtures() {
	var xf1 Transform
	xf1.Q.Set(this.sweep.A0)
	xf1.P = SubVV(this.sweep.C0, MulRV(xf1.Q, this.sweep.LocalCenter))

	broadPhase := this.world.contactManager.BroadPhase
	for f := this.fixtureList; f != nil; f = f.Next {
		f.Synchronize(broadPhase, xf1, this.xf)
	}
}

func (this *Body) synchronizeTransform() {
	this.xf.Q.Set(this.sweep.A)
	this.xf.P = SubVV(this.sweep.C, MulRV(this.xf.Q, this.sweep.LocalCenter))
}

// This is used to prevent connected bodies from colliding.
// It may lie, depending on the collideConnected flag.
func (this *Body) ShouldCollide(other *Body) bool {
	// At least one body should be dynamic.
	if this.itype != DynamicBody && other.itype != DynamicBody {
		return false
	}

	// Does a joint prevent collision?
	for jn := this.jointList; jn != nil; jn = jn.Next {
		if jn.Other == other {
			if !jn.Joint.GetCollideConnected() {
				return false
			}
		}
	}

	return true
}

func (this *Body) Advance(alpha float64) {
	// Advance to the new safe time. This doesn't sync the broad-phase.
	this.sweep.Advance(alpha)
	this.sweep.C = this.sweep.C0
	this.sweep.A = this.sweep.A0
	this.xf.Q.Set(this.sweep.A)
	this.xf.P = SubVV(this.sweep.C, MulRV(this.xf.Q, this.sweep.LocalCenter))
}
