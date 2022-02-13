package box2d

// The world class manages all physics entities, dynamic simulation,
// and asynchronous queries. The world also contains efficient memory
// management facilities.
type World struct {
	flags int

	contactManager *ContactManager

	bodyList  *Body
	jointList IJoint

	bodyCount  int
	jointCount int

	gravity    Vec2
	allowSleep bool

	destructionListener IDestructionListener
	debugDraw           IDraw

	// This is used to compute the time step ratio to
	// support a variable time step.
	inv_dt0 float64

	// These are for debugging the solver.
	warmStarting      bool
	continuousPhysics bool
	subStepping       bool

	stepComplete bool

	profile Profile
}

const (
	world_e_newFixture  = 0x0001
	world_e_locked      = 0x0002
	world_e_clearForces = 0x0004
)

func MakeWorld(gravity Vec2) World {
	return World{
		warmStarting:      true,
		continuousPhysics: true,
		stepComplete:      true,
		allowSleep:        true,
		gravity:           gravity,
		flags:             world_e_clearForces,
		contactManager:    NewContactManager(),
	}
}

func NewWorld(gravity Vec2) *World {
	w := MakeWorld(gravity)
	return &w
}

func (w *World) Destroy() {

	// Some shapes allocate using b2Alloc.
	b := w.bodyList
	for b != nil {
		bNext := b.next

		f := b.fixtureList
		for f != nil {
			fNext := f.Next
			f.ProxyCount = 0
			f.Destroy()
			f = fNext
		}

		b = bNext
	}
}

// Register a destruction listener. The listener is owned by you and must
// remain in scope.
func (w *World) SetDestructionListener(listener IDestructionListener) {
	w.destructionListener = listener
}

// Register a contact filter to provide specific control over collision.
// Otherwise the default filter is used (b2_defaultFilter). The listener is
// owned by you and must remain in scope.
func (w *World) SetContactFilter(filter IContactFilter) {
	w.contactManager.ContactFilter = filter
}

// Register a contact event listener. The listener is owned by you and must
// remain in scope.
func (w *World) SetContactListener(listener IContactListener) {
	w.contactManager.ContactListener = listener
}

// Register a routine for debug drawing. The debug draw functions are called
// inside with b2World::DrawDebugData method. The debug draw object is owned
// by you and must remain in scope.
func (w *World) SetDebugDraw(debugDraw IDraw) {
	w.debugDraw = debugDraw
}

// Create a rigid body given a definition. No reference to the definition
// is retained.
// @warning This function is locked during callbacks.
func (w *World) CreateBody(def *BodyDef) *Body {
	if w.IsLocked() {
		return nil
	}

	b := NewBody(def, w)

	// Add to world doubly linked list.
	b.prev = nil
	b.next = w.bodyList
	if w.bodyList != nil {
		w.bodyList.prev = b
	}
	w.bodyList = b
	w.bodyCount++

	return b
}

// Destroy a rigid body given a definition. No reference to the definition
// is retained. This function is locked during callbacks.
// @warning This automatically deletes all associated shapes and joints.
// @warning This function is locked during callbacks.
func (w *World) DestroyBody(b *Body) {
	if w.IsLocked() {
		return
	}

	// Delete the attached joints.
	je := b.jointList
	for je != nil {
		je0 := je
		je = je.Next

		if w.destructionListener != nil {
			w.destructionListener.SayGoodbyeJoint(je0.Joint)
		}

		w.DestroyJoint(je0.Joint)

		b.jointList = je
	}
	b.jointList = nil

	// Delete the attached contacts.
	ce := b.contactList
	for ce != nil {
		ce0 := ce
		ce = ce.Next
		w.contactManager.Destroy(ce0.Contact)
	}
	b.contactList = nil

	// Delete the attached fixtures. This destroys broad-phase proxies.
	f := b.fixtureList
	for f != nil {
		f0 := f
		f = f.Next

		if w.destructionListener != nil {
			w.destructionListener.SayGoodbyeFixture(f0)
		}

		f0.DestroyProxies(w.contactManager.BroadPhase)
		f0.Destroy()

		b.fixtureList = f
		b.fixtureCount -= 1
	}
	b.fixtureList = nil
	b.fixtureCount = 0

	// Remove world body list.
	if b.prev != nil {
		b.prev.next = b.next
	}

	if b.next != nil {
		b.next.prev = b.prev
	}

	if b == w.bodyList {
		w.bodyList = b.next
	}

	w.bodyCount--
}

// Create a joint to constrain bodies together. No reference to the definition
// is retained. This may cause the connected bodies to cease colliding.
// @warning This function is locked during callbacks.
func (w *World) CreateJoint(def IJointDef) IJoint {
	if w.IsLocked() {
		return nil
	}

	j := Joint_Create(def)

	// Connect to the world list.
	j.SetPrev(nil)
	j.SetNext(w.jointList)
	if w.jointList != nil {
		w.jointList.SetPrev(j)
	}
	w.jointList = j
	w.jointCount++

	// Connect to the bodies' doubly linked lists.
	j.GetEdgeA().Joint = j
	j.GetEdgeA().Other = j.GetBodyB()
	j.GetEdgeA().Prev = nil
	j.GetEdgeA().Next = j.GetBodyA().jointList
	if j.GetBodyA().jointList != nil {
		j.GetBodyA().jointList.Prev = j.GetEdgeA()
	}
	j.GetBodyA().jointList = j.GetEdgeA()

	j.GetEdgeB().Joint = j
	j.GetEdgeB().Other = j.GetBodyA()
	j.GetEdgeB().Prev = nil
	j.GetEdgeB().Next = j.GetBodyB().jointList
	if j.GetBodyB().jointList != nil {
		j.GetBodyB().jointList.Prev = j.GetEdgeB()
	}
	j.GetBodyB().jointList = j.GetEdgeB()

	bodyA := def.GetBodyA()
	bodyB := def.GetBodyB()

	// If the joint prevents collisions, then flag any contacts for filtering.
	if !def.GetCollideConnected() {
		edge := bodyB.GetContactList()
		for edge != nil {
			if edge.Other == bodyA {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.Contact.FlagForFiltering()
			}

			edge = edge.Next
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j
}

// Destroy a joint. This may cause the connected bodies to begin colliding.
// @warning This function is locked during callbacks.
func (w *World) DestroyJoint(j IJoint) {
	if w.IsLocked() {
		return
	}

	collideConnected := j.GetCollideConnected()

	// Remove from the doubly linked list.
	if j.GetPrev() != nil {
		j.GetPrev().SetNext(j.GetNext())
	}

	if j.GetNext() != nil {
		j.GetNext().SetPrev(j.GetPrev())
	}

	if j == w.jointList {
		w.jointList = j.GetNext()
	}

	// Disconnect from island graph.
	bodyA := j.GetBodyA()
	bodyB := j.GetBodyB()

	// Wake up connected bodies.
	bodyA.SetAwake(true)
	bodyB.SetAwake(true)

	// Remove from body 1.
	if j.GetEdgeA().Prev != nil {
		j.GetEdgeA().Prev.Next = j.GetEdgeA().Next
	}

	if j.GetEdgeA().Next != nil {
		j.GetEdgeA().Next.Prev = j.GetEdgeA().Prev
	}

	if j.GetEdgeA() == bodyA.jointList {
		bodyA.jointList = j.GetEdgeA().Next
	}

	j.GetEdgeA().Prev = nil
	j.GetEdgeA().Next = nil

	// Remove from body 2
	if j.GetEdgeB().Prev != nil {
		j.GetEdgeB().Prev.Next = j.GetEdgeB().Next
	}

	if j.GetEdgeB().Next != nil {
		j.GetEdgeB().Next.Prev = j.GetEdgeB().Prev
	}

	if j.GetEdgeB() == bodyB.jointList {
		bodyB.jointList = j.GetEdgeB().Next
	}

	j.GetEdgeB().Prev = nil
	j.GetEdgeB().Next = nil

	w.jointCount--

	// If the joint prevents collisions, then flag any contacts for filtering.
	if !collideConnected {
		edge := bodyB.GetContactList()
		for edge != nil {
			if edge.Other == bodyA {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.Contact.FlagForFiltering()
			}

			edge = edge.Next
		}
	}
}

// Take a time step. This performs collision detection, integration,
// and constraint solution.
// @param timeStep the amount of time to simulate, this should not vary.
// @param velocityIterations for the velocity constraint solver.
// @param positionIterations for the position constraint solver.
func (w *World) Step(dt float64, velocityIterations int, positionIterations int) {
	stepTimer := MakeTimer()

	// If new fixtures were added, we need to find the new contacts.
	if (w.flags & world_e_newFixture) != 0 {
		w.contactManager.FindNewContacts()
		w.flags &= ^world_e_newFixture
	}

	w.flags |= world_e_locked

	var step timeStep
	step.dt = dt
	step.velocityIterations = velocityIterations
	step.positionIterations = positionIterations
	if dt > 0.0 {
		step.inv_dt = 1.0 / dt
	} else {
		step.inv_dt = 0.0
	}

	step.dtRatio = w.inv_dt0 * dt

	step.warmStarting = w.warmStarting

	// Update contacts. This is where some contacts are destroyed.
	{
		timer := MakeTimer()
		w.contactManager.Collide()
		w.profile.collide = timer.GetMilliseconds()
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if w.stepComplete && step.dt > 0.0 {
		timer := MakeTimer()
		w.solve(&step)
		w.profile.solve = timer.GetMilliseconds()
	}

	// Handle TOI events.
	if w.continuousPhysics && step.dt > 0.0 {
		timer := MakeTimer()
		w.solveTOI(&step)
		w.profile.solveTOI = timer.GetMilliseconds()
	}

	if step.dt > 0.0 {
		w.inv_dt0 = step.inv_dt
	}

	if (w.flags & world_e_clearForces) != 0 {
		w.ClearForces()
	}

	w.flags &= ^world_e_locked

	w.profile.step = stepTimer.GetMilliseconds()
}

// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
// a fixed sized time step under a variable frame-rate.
// When you perform sub-stepping you will disable auto clearing of forces and instead call
// ClearForces after all sub-steps are complete in one pass of your game loop.
// @see SetAutoClearForces
func (w *World) ClearForces() {
	for body := w.bodyList; body != nil; body = body.GetNext() {
		body.force.SetZero()
		body.torque = 0.0
	}
}

func (w *World) DrawShape(fixture *Fixture, xf Transform, color Color) {
	switch fixture.GetType() {
	case Shape_e_circle:
		circle := fixture.GetShape().(*CircleShape)

		center := MulX(xf, circle.P)
		radius := circle.Radius
		axis := MulRV(xf.Q, Vec2{1.0, 0.0})

		w.debugDraw.DrawSolidCircle(center, radius, axis, color)
	case Shape_e_edge:
		edge := fixture.GetShape().(*EdgeShape)
		v1 := MulX(xf, edge.Vertex1)
		v2 := MulX(xf, edge.Vertex2)
		w.debugDraw.DrawSegment(v1, v2, color)
	case Shape_e_chain:
		chain := fixture.GetShape().(*ChainShape)
		count := chain.Count
		vertices := chain.Vertices

		v1 := MulX(xf, vertices[0])
		for i := 0; i < count; i++ {
			v2 := MulX(xf, vertices[i])
			w.debugDraw.DrawSegment(v1, v2, color)
			w.debugDraw.DrawCircle(v1, 0.05, color)
			v1 = v2
		}
	case Shape_e_polygon:
		poly := fixture.GetShape().(*PolygonShape)
		vertexCount := poly.VertexCount
		var vertices [MaxPolygonVertices]Vec2

		for i := 0; i < vertexCount; i++ {
			vertices[i] = MulX(xf, poly.Vertices[i])
		}

		w.debugDraw.DrawSolidPolygon(vertices[:vertexCount], color)
	default:
	}
}

func (w *World) drawJoint(joint IJoint) {
	bodyA := joint.GetBodyA()
	bodyB := joint.GetBodyB()
	xf1 := bodyA.GetTransform()
	xf2 := bodyB.GetTransform()
	x1 := xf1.P
	x2 := xf2.P
	p1 := joint.GetAnchorA()
	p2 := joint.GetAnchorB()

	color := MakeColor(0.5, 0.8, 0.8)

	switch joint.GetType() {
	case Joint_e_distanceJoint:
		w.debugDraw.DrawSegment(p1, p2, color)
	case Joint_e_pulleyJoint:
		pulley := joint.(*PulleyJoint)
		s1 := pulley.GetGroundAnchorA()
		s2 := pulley.GetGroundAnchorB()
		w.debugDraw.DrawSegment(s1, p1, color)
		w.debugDraw.DrawSegment(s2, p2, color)
		w.debugDraw.DrawSegment(s1, s2, color)
	case Joint_e_mouseJoint:
		// don't draw this
	default:
		w.debugDraw.DrawSegment(x1, p1, color)
		w.debugDraw.DrawSegment(p1, p2, color)
		w.debugDraw.DrawSegment(x2, p2, color)
	}
}

// Call this to draw shapes and other debug draw data. This is intentionally non-const.
func (w *World) DebugDraw() {
	if w.debugDraw == nil {
		return
	}

	flags := w.debugDraw.GetFlags()

	if (flags & Draw_e_shapeBit) != 0 {
		for b := w.bodyList; b != nil; b = b.GetNext() {
			xf := b.GetTransform()
			for f := b.GetFixtureList(); f != nil; f = f.GetNext() {
				if !b.IsActive() {
					w.DrawShape(f, xf, MakeColor(0.5, 0.5, 0.3))
				} else if b.GetType() == StaticBody {
					w.DrawShape(f, xf, MakeColor(0.5, 0.9, 0.5))
				} else if b.GetType() == KinematicBody {
					w.DrawShape(f, xf, MakeColor(0.5, 0.5, 0.9))
				} else if !b.IsAwake() {
					w.DrawShape(f, xf, MakeColor(0.6, 0.6, 0.6))
				} else {
					w.DrawShape(f, xf, MakeColor(0.9, 0.7, 0.7))
				}
			}
		}
	}

	if (flags & Draw_e_jointBit) != 0 {
		for j := w.jointList; j != nil; j = j.GetNext() {
			w.drawJoint(j)
		}
	}

	if (flags & Draw_e_pairBit) != 0 {
		//color := Color{0.3, 0.9, 0.9}
		for c := w.contactManager.ContactList; c != nil; c = c.GetNext() {

		}
	}

	if (flags & Draw_e_aabbBit) != 0 {
		color := MakeColor(0.9, 0.3, 0.9)
		bp := w.contactManager.BroadPhase

		for b := w.bodyList; b != nil; b = b.GetNext() {
			if !b.IsActive() {
				continue
			}

			for f := b.GetFixtureList(); f != nil; f = f.GetNext() {
				for i := 0; i < f.ProxyCount; i++ {
					proxy := &f.Proxies[i]
					aabb := bp.GetFatAABB(proxy.ProxyId)
					var vs [4]Vec2
					vs[0].Set(aabb.LowerBound.X, aabb.LowerBound.Y)
					vs[1].Set(aabb.UpperBound.X, aabb.LowerBound.Y)
					vs[2].Set(aabb.UpperBound.X, aabb.UpperBound.Y)
					vs[3].Set(aabb.LowerBound.X, aabb.UpperBound.Y)

					w.debugDraw.DrawPolygon(vs[:], color)
				}
			}
		}
	}

	if (flags & Draw_e_centerOfMassBit) != 0 {
		for b := w.bodyList; b != nil; b = b.GetNext() {
			xf := b.GetTransform()
			xf.P = b.GetWorldCenter()
			w.debugDraw.DrawTransform(xf)
		}
	}
}

/// Query the world for all fixtures that potentially overlap the
/// provided AABB.
/// @param callback a user implemented callback function.
/// @param aabb the query box.
type WorldQueryAABBWrapper struct {
	broadPhase *BroadPhase
	callback   IQueryCallback
}

func (w *WorldQueryAABBWrapper) QueryCallback(proxyId int) bool {
	proxy := w.broadPhase.GetUserData(proxyId).(*FixtureProxy)
	return w.callback.ReportFixture(proxy.Fixture)
}

func (w *World) QueryAABB(callback IQueryCallback, aabb AABB) {
	var wrapper WorldQueryAABBWrapper
	wrapper.broadPhase = w.contactManager.BroadPhase
	wrapper.callback = callback
	w.contactManager.BroadPhase.Query(wrapper.QueryCallback, aabb)
}

/// Query the world for all fixtures that potentially overlap the
/// provided transformed shape.
/// @param callback a user implemented callback class.
/// @param aabb the query box.
type WorldQueryShapeWrapper struct {
	broadPhase *BroadPhase
	callback   func(*Fixture) bool
	shape      IShape
	transform  *Transform
}

func (w *WorldQueryShapeWrapper) QueryCallback(proxyId int) bool {
	fixture := w.broadPhase.GetUserData(proxyId).(*FixtureProxy).Fixture
	if TestOverlap(w.shape, 0, fixture.GetShape(), 0, *w.transform, fixture.GetBody().GetTransform()) {
		return w.callback(fixture)
	}
	return true
}

func (w *World) QueryShape(callback func(*Fixture) bool, shape IShape, transform *Transform) {
	if transform == nil {
		transform = new(Transform)
		transform.SetIdentity()
	}
	var wrapper WorldQueryShapeWrapper
	wrapper.broadPhase = w.contactManager.BroadPhase
	wrapper.callback = callback
	wrapper.shape = shape
	wrapper.transform = transform
	var aabb AABB
	shape.ComputeAABB(&aabb, *transform, 0)
	w.contactManager.BroadPhase.Query(wrapper.QueryCallback, aabb)
}

/// Ray-cast the world for all fixtures in the path of the ray. Your callback
/// controls whether you get the closest point, any point, or n-points.
/// The ray-cast ignores shapes that contain the starting point.
/// @param callback a user implemented callback function.
/// @param point1 the ray starting point
/// @param point2 the ray ending point
type WorldRayCastWrapper struct {
	broadPhase *BroadPhase
	callback   IRayCastCallback
}

func (ww *WorldRayCastWrapper) RayCastCallback(input RayCastInput, proxyId int) float64 {
	proxy := ww.broadPhase.GetUserData(proxyId).(*FixtureProxy)
	fixture := proxy.Fixture
	index := proxy.ChildIndex
	output, hit := fixture.RayCast(input, index)

	if hit {
		fraction := output.Fraction
		point := AddVV(MulFV(1.0-fraction, input.P1), MulFV(fraction, input.P2))
		return ww.callback.ReportFixture(fixture, point, output.Normal, fraction)
	}

	return input.MaxFraction
}

func (w *World) RayCast(callback IRayCastCallback, point1 Vec2, point2 Vec2) {
	var wrapper WorldRayCastWrapper
	wrapper.broadPhase = w.contactManager.BroadPhase
	wrapper.callback = callback
	var input RayCastInput
	input.MaxFraction = 1.0
	input.P1 = point1
	input.P2 = point2
	w.contactManager.BroadPhase.RayCast(wrapper.RayCastCallback, input)
}

/// Get the world body list. With the returned body, use b2Body::GetNext to get
/// the next body in the world list. A NULL body indicates the end of the list.
/// @return the head of the world body list.
func (w *World) GetBodyList() *Body {
	return w.bodyList
}

/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
/// the next joint in the world list. A NULL joint indicates the end of the list.
/// @return the head of the world joint list.
func (w *World) GetJointList() IJoint {
	return w.jointList
}

/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
/// the next contact in the world list. A NULL contact indicates the end of the list.
/// @return the head of the world contact list.
/// @warning contacts are created and destroyed in the middle of a time step.
/// Use b2ContactListener to avoid missing contacts.
func (w *World) GetContactList() IContact {
	return w.contactManager.ContactList
}

/// Enable/disable sleep.
func (w *World) SetAllowSleeping(flag bool) {
	if flag == w.allowSleep {
		return
	}

	w.allowSleep = flag
	if !w.allowSleep {
		for b := w.bodyList; b != nil; b = b.next {
			b.SetAwake(true)
		}
	}
}
func (w *World) GetAllowSleeping() bool {
	return w.allowSleep
}

/// Enable/disable warm starting. For testing.
func (w *World) SetWarmStarting(flag bool) {
	w.warmStarting = flag
}

func (w *World) GetWarmStarting() bool {
	return w.warmStarting
}

/// Enable/disable continuous physics. For testing.
func (w *World) SetContinuousPhysics(flag bool) {
	w.continuousPhysics = flag
}

func (w *World) GetContinuousPhysics() bool {
	return w.continuousPhysics
}

/// Enable/disable single stepped continuous physics. For testing.
func (w *World) SetSubStepping(flag bool) {
	w.subStepping = flag
}

func (w *World) GetSubStepping() bool {
	return w.subStepping
}

/// Get the number of broad-phase proxies.
func (w *World) GetProxyCount() int {
	return w.contactManager.BroadPhase.GetProxyCount()
}

/// Get the number of bodies.
func (w *World) GetBodyCount() int {
	return w.bodyCount
}

/// Get the number of joints.
func (w *World) GetJointCount() int {
	return w.jointCount
}

/// Get the number of contacts (each may have 0 or more contact points).
func (w *World) GetContactCount() int {
	return w.contactManager.ContactCount
}

/// Get the height of the dynamic tree.
func (w *World) GetTreeHeight() int {
	return w.contactManager.BroadPhase.GetTreeHeight()
}

/// Get the balance of the dynamic tree.
func (w *World) GetTreeBalance() int {
	return w.contactManager.BroadPhase.GetTreeBalance()
}

/// Get the quality metric of the dynamic tree. The smaller the better.
/// The minimum is 1.
func (w *World) GetTreeQuality() float64 {
	return w.contactManager.BroadPhase.GetTreeQuality()
}

/// Change the global gravity vector.
func (w *World) SetGravity(gravity Vec2) {
	w.gravity = gravity
}

/// Get the global gravity vector.
func (w *World) GetGravity() Vec2 {
	return w.gravity
}

// Is the world locked (in the middle of a time step).
func (w *World) IsLocked() bool {
	return (w.flags & world_e_locked) == world_e_locked
}

/// Set flag to control automatic clearing of forces after each time step.
func (w *World) SetAutoClearForces(flag bool) {
	if flag {
		w.flags |= world_e_clearForces
	} else {
		w.flags &= ^world_e_clearForces
	}
}

/// Get the flag that controls automatic clearing of forces after each time step.
func (w *World) GetAutoClearForces() bool {
	return (w.flags & world_e_clearForces) == world_e_clearForces
}

/// Get the contact manager for testing.
func (w *World) GetContactManager() *ContactManager {
	return w.contactManager
}

/// Get the current profile.
func (w *World) GetProfile() *Profile {
	return &w.profile
}

func (w *World) solve(step *timeStep) {
	w.profile.solveInit = 0.0
	w.profile.solveVelocity = 0.0
	w.profile.solvePosition = 0.0

	// Size the island for the worst case.
	island := NewIsland(w.bodyCount, w.contactManager.ContactCount,
		w.jointCount, w.contactManager.ContactListener)

	// Clear all the island flags.
	for b := w.bodyList; b != nil; b = b.next {
		b.flags &= ^body_e_islandFlag
	}
	for c := w.contactManager.ContactList; c != nil; c = c.GetNext() {
		c.SetFlags(c.GetFlags() & ^Contact_e_islandFlag)
	}
	for j := w.jointList; j != nil; j = j.GetNext() {
		j.SetIsland(false)
	}

	// Build and simulate all awake islands.
	stackSize := w.bodyCount
	//b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
	stack := make([]*Body, stackSize)
	for seed := w.bodyList; seed != nil; seed = seed.next {
		if (seed.flags & body_e_islandFlag) != 0 {
			continue
		}

		if !seed.IsAwake() || !seed.IsActive() {
			continue
		}

		// The seed can be dynamic or kinematic.
		if seed.GetType() == StaticBody {
			continue
		}

		// Reset island and stack.
		island.Clear()
		stackCount := 0
		stack[stackCount] = seed
		stackCount++
		seed.flags |= body_e_islandFlag

		// Perform a depth first search (DFS) on the constraint graph.
		for stackCount > 0 {
			// Grab the next body off the stack and add it to the island.
			stackCount--
			b := stack[stackCount]
			island.AddBody(b)

			// Make sure the body is awake.
			b.SetAwake(true)

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if b.GetType() == StaticBody {
				continue
			}

			// Search all contacts connected to this body.
			for ce := b.contactList; ce != nil; ce = ce.Next {
				contact := ce.Contact

				// Has this contact already been added to an island?
				if (contact.GetFlags() & Contact_e_islandFlag) != 0 {
					continue
				}

				// Is this contact solid and touching?
				if !contact.IsEnabled() || !contact.IsTouching() {
					continue
				}

				// Skip sensors.
				sensorA := contact.GetFixtureA().IsSensor
				sensorB := contact.GetFixtureB().IsSensor
				if sensorA || sensorB {
					continue
				}

				island.AddContact(contact)
				contact.SetFlags(contact.GetFlags() | Contact_e_islandFlag)

				other := ce.Other

				// Was the other body already added to this island?
				if (other.flags & body_e_islandFlag) != 0 {
					continue
				}

				stack[stackCount] = other
				stackCount++
				other.flags |= body_e_islandFlag
			}

			// Search all joints connect to this body.
			for je := b.jointList; je != nil; je = je.Next {
				if je.Joint.GetIsland() {
					continue
				}

				other := je.Other

				// Don't simulate joints connected to inactive bodies.
				if !other.IsActive() {
					continue
				}

				island.AddJoint(je.Joint)
				je.Joint.SetIsland(true)

				if (other.flags & body_e_islandFlag) != 0 {
					continue
				}

				stack[stackCount] = other
				stackCount++
				other.flags |= body_e_islandFlag
			}
		}

		var profile Profile
		island.Solve(&profile, step, w.gravity, w.allowSleep)
		w.profile.solveInit += profile.solveInit
		w.profile.solveVelocity += profile.solveVelocity
		w.profile.solvePosition += profile.solvePosition

		// Post solve cleanup.
		for i := 0; i < island.BodyCount; i++ {
			// Allow static bodies to participate in other islands.
			b := island.Bodies[i]
			if b.GetType() == StaticBody {
				b.flags &= ^body_e_islandFlag
			}
		}
	}

	//m_stackAllocator.Free(stack);

	{
		timer := MakeTimer()
		// Synchronize fixtures, check for out of range bodies.
		for b := w.bodyList; b != nil; b = b.GetNext() {
			// If a body was not in an island then it did not move.
			if (b.flags & body_e_islandFlag) == 0 {
				continue
			}

			if b.GetType() == StaticBody {
				continue
			}

			// Update fixtures (for broad-phase).
			b.synchronizeFixtures()
		}

		// Look for new contacts.
		w.contactManager.FindNewContacts()
		w.profile.broadphase = timer.GetMilliseconds()
	}
}

func (w *World) solveTOI(step *timeStep) {
	island := NewIsland(2*MaxTOIContacts, MaxTOIContacts, 0, w.contactManager.ContactListener)

	if w.stepComplete {
		for b := w.bodyList; b != nil; b = b.next {
			b.flags &= ^body_e_islandFlag
			b.sweep.Alpha0 = 0.0
		}

		for c := w.contactManager.ContactList; c != nil; c = c.GetNext() {
			// Invalidate TOI
			c.SetFlags(c.GetFlags() & ^(Contact_e_toiFlag | Contact_e_islandFlag))
			c.SetToiCount(0)
			c.SetToi(1.0)
		}
	}

	// Find TOI events and solve them.
	for {
		// Find the first TOI.
		var minContact IContact = nil
		minAlpha := 1.0

		for c := w.contactManager.ContactList; c != nil; c = c.GetNext() {
			// Is this contact disabled?
			if !c.IsEnabled() {
				continue
			}

			// Prevent excessive sub-stepping.
			if c.GetToiCount() > MaxSubSteps {
				continue
			}

			alpha := 1.0
			if (c.GetFlags() & Contact_e_toiFlag) != 0 {
				// This contact has a valid cached TOI.
				alpha = c.GetToi()
			} else {
				fA := c.GetFixtureA()
				fB := c.GetFixtureB()

				// Is there a sensor?
				if fA.IsSensor || fB.IsSensor {
					continue
				}

				bA := fA.GetBody()
				bB := fB.GetBody()

				typeA := bA.xtype
				typeB := bB.xtype

				activeA := bA.IsAwake() && typeA != StaticBody
				activeB := bB.IsAwake() && typeB != StaticBody

				// Is at least one body active (awake and dynamic or kinematic)?
				if !activeA && !activeB {
					continue
				}

				collideA := bA.IsBullet() || typeA != DynamicBody
				collideB := bB.IsBullet() || typeB != DynamicBody

				// Are these two non-bullet dynamic bodies?
				if !collideA && !collideB {
					continue
				}

				// Compute the TOI for this contact.
				// Put the sweeps onto the same time interval.
				alpha0 := bA.sweep.Alpha0

				if bA.sweep.Alpha0 < bB.sweep.Alpha0 {
					alpha0 = bB.sweep.Alpha0
					bA.sweep.Advance(alpha0)
				} else if bB.sweep.Alpha0 < bA.sweep.Alpha0 {
					alpha0 = bA.sweep.Alpha0
					bB.sweep.Advance(alpha0)
				}

				indexA := c.GetChildIndexA()
				indexB := c.GetChildIndexB()

				// Compute the time of impact in interval [0, minTOI]
				var input TOIInput
				input.ProxyA.Set(fA.GetShape(), indexA)
				input.ProxyB.Set(fB.GetShape(), indexB)
				input.SweepA = bA.sweep
				input.SweepB = bB.sweep
				input.TMax = 1.0

				var output TOIOutput
				TimeOfImpact(&output, &input)

				// Beta is the fraction of the remaining portion of the .
				beta := output.T
				if output.State == TOIOutput_e_touching {
					alpha = MinF(alpha0+(1.0-alpha0)*beta, 1.0)
				} else {
					alpha = 1.0
				}

				c.SetToi(alpha)
				c.SetFlags(c.GetFlags() | Contact_e_toiFlag)
			}

			if alpha < minAlpha {
				// This is the minimum TOI found so far.
				minContact = c
				minAlpha = alpha
			}
		}

		if minContact == nil || 1.0-10.0*Epsilon < minAlpha {
			// No more TOI events. Done!
			w.stepComplete = true
			break
		}

		// Advance the bodies to the TOI.
		fA := minContact.GetFixtureA()
		fB := minContact.GetFixtureB()
		bA := fA.GetBody()
		bB := fB.GetBody()

		backup1 := bA.sweep
		backup2 := bB.sweep

		bA.Advance(minAlpha)
		bB.Advance(minAlpha)

		// The TOI contact likely has some new contact points.
		minContact.Update(w.contactManager.ContactListener)
		minContact.SetFlags(minContact.GetFlags() & ^Contact_e_toiFlag)
		minContact.SetToiCount(minContact.GetToiCount() + 1)

		// Is the contact solid?
		if !minContact.IsEnabled() || !minContact.IsTouching() {
			// Restore the sweeps.
			minContact.SetEnabled(false)
			bA.sweep = backup1
			bB.sweep = backup2
			bA.synchronizeTransform()
			bB.synchronizeTransform()
			continue
		}

		bA.SetAwake(true)
		bB.SetAwake(true)

		// Build the island
		island.Clear()
		island.AddBody(bA)
		island.AddBody(bB)
		island.AddContact(minContact)

		bA.flags |= body_e_islandFlag
		bB.flags |= body_e_islandFlag
		minContact.SetFlags(minContact.GetFlags() | Contact_e_islandFlag)

		// Get contacts on bodyA and bodyB.
		bodies := [2]*Body{bA, bB}
		for i := 0; i < 2; i++ {
			body := bodies[i]
			if body.xtype == DynamicBody {
				for ce := body.contactList; ce != nil; ce = ce.Next {
					if island.BodyCount == island.BodyCapacity {
						break
					}

					if island.ContactCount == island.ContactCapacity {
						break
					}

					contact := ce.Contact

					// Has this contact already been added to the island?
					if (contact.GetFlags() & Contact_e_islandFlag) != 0 {
						continue
					}

					// Only add static, kinematic, or bullet bodies.
					other := ce.Other
					if other.xtype == DynamicBody && !body.IsBullet() && !other.IsBullet() {
						continue
					}

					// Skip sensors.
					sensorA := contact.GetFixtureA().IsSensor
					sensorB := contact.GetFixtureB().IsSensor
					if sensorA || sensorB {
						continue
					}

					// Tentatively advance the body to the TOI.
					backup := other.sweep
					if (other.flags & body_e_islandFlag) == 0 {
						other.Advance(minAlpha)
					}

					// Update the contact points
					contact.Update(w.contactManager.ContactListener)

					// Was the contact disabled by the user?
					if !contact.IsEnabled() {
						other.sweep = backup
						other.synchronizeTransform()
						continue
					}

					// Are there contact points?
					if !contact.IsTouching() {
						other.sweep = backup
						other.synchronizeTransform()
						continue
					}

					// Add the contact to the island
					contact.SetFlags(contact.GetFlags() | Contact_e_islandFlag)
					island.AddContact(contact)

					// Has the other body already been added to the island?
					if (other.flags & body_e_islandFlag) != 0 {
						continue
					}

					// Add the other body to the island.
					other.flags |= body_e_islandFlag

					if other.xtype != StaticBody {
						other.SetAwake(true)
					}

					island.AddBody(other)
				}
			}
		}

		var subStep timeStep
		subStep.dt = (1.0 - minAlpha) * step.dt
		subStep.inv_dt = 1.0 / subStep.dt
		subStep.dtRatio = 1.0
		subStep.positionIterations = 20
		subStep.velocityIterations = step.velocityIterations
		subStep.warmStarting = false
		island.SolveTOI(&subStep, bA.islandIndex, bB.islandIndex)

		// Reset island flags and synchronize broad-phase proxies.
		for i := 0; i < island.BodyCount; i++ {
			body := island.Bodies[i]
			body.flags &= ^body_e_islandFlag

			if body.xtype != DynamicBody {
				continue
			}

			body.synchronizeFixtures()

			// Invalidate all contact TOIs on this displaced body.
			for ce := body.contactList; ce != nil; ce = ce.Next {
				ce.Contact.SetFlags(ce.Contact.GetFlags() & ^(Contact_e_toiFlag | Contact_e_islandFlag))
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		w.contactManager.FindNewContacts()

		if w.subStepping {
			w.stepComplete = false
			break
		}
	}
}

/// Dump the world into the log file.
/// @warning this should be called outside of a time step.
func (w *World) Dump() {
	if (w.flags & world_e_locked) == world_e_locked {
		return
	}

	Log("b2Vec2 g(%.15f, %.15f);\n", w.gravity.X, w.gravity.Y)
	Log("m_world->SetGravity(g);\n")

	Log("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", w.bodyCount)
	Log("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", w.jointCount)
	i := 0
	for b := w.bodyList; b != nil; b = b.next {
		b.islandIndex = i
		b.Dump()
		i++
	}

	i = 0
	for j := w.jointList; j != nil; j = j.GetNext() {
		j.SetIndex(i)
		i++
	}

	// First pass on joints, skip gear joints.
	for j := w.jointList; j != nil; j = j.GetNext() {
		if j.GetType() == Joint_e_gearJoint {
			continue
		}

		Log("{\n")
		j.Dump()
		Log("}\n")
	}

	// Second pass on joints, only gear joints.
	for j := w.jointList; j != nil; j = j.GetNext() {
		if j.GetType() != Joint_e_gearJoint {
			continue
		}

		Log("{\n")
		j.Dump()
		Log("}\n")
	}

	Log("b2Free(joints);\n")
	Log("b2Free(bodies);\n")
	Log("joints = nil;\n")
	Log("bodies = nil;\n")
}
