package box2d

type JointType int

const (
	Joint_e_unknownJoint = iota
	Joint_e_revoluteJoint
	Joint_e_prismaticJoint
	Joint_e_distanceJoint
	Joint_e_pulleyJoint
	Joint_e_mouseJoint
	Joint_e_gearJoint
	Joint_e_wheelJoint
	Joint_e_weldJoint
	Joint_e_frictionJoint
	Joint_e_ropeJoint
)

type LimitState int

const (
	LimitState_e_inactiveLimit = iota
	LimitState_e_atLowerLimit
	LimitState_e_atUpperLimit
	LimitState_e_equalLimits
)

type Jacobian struct {
	Linear   Vec2
	AngularA float64
	AngularB float64
}

// A joint edge is used to connect bodies and joints together
// in a joint graph where each body is a node and each joint
// is an edge. A joint edge belongs to a doubly linked list
// maintained in each attached body. Each joint has two joint
// nodes, one for each attached body.
type JointEdge struct {
	Other *Body      // provides quick access to the other body attached.
	Joint IJoint     // the joint
	Prev  *JointEdge // the previous joint edge in the body's joint list
	Next  *JointEdge // the next joint edge in the body's joint list
}

type IJointDef interface {
	GetType() JointType

	GetBodyA() *Body
	GetBodyB() *Body

	GetCollideConnected() bool
}

// Joint definitions are used to construct joints.
type JointDef struct {
	// The joint type is set automatically for concrete joint types.
	Type JointType

	// Use this to attach application specific data to your joints.
	UserData interface{}

	// The first attached body.
	BodyA *Body

	// The second attached body.
	BodyB *Body

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool
}

func (this *JointDef) GetType() JointType {
	return this.Type
}

func (this *JointDef) GetBodyA() *Body {
	return this.BodyA
}

func (this *JointDef) GetBodyB() *Body {
	return this.BodyB
}

func (this *JointDef) GetCollideConnected() bool {
	return this.CollideConnected
}

type IJoint interface {

	// Get the type of the concrete joint.
	GetType() JointType

	//
	GetBodyA() *Body
	GetBodyB() *Body

	// Get the anchor point on bodyA in world coordinates.
	GetAnchorA() Vec2

	// Get the anchor point on bodyB in world coordinates.
	GetAnchorB() Vec2

	// Get the reaction force on bodyB at the joint anchor in Newtons.
	GetReactionForce(inv_dt float64) Vec2

	// Get the reaction torque on bodyB in N*m.
	GetReactionTorque(inv_dt float64) float64

	// Get the next joint the world joint list.
	GetNext() IJoint
	SetNext(IJoint)

	GetPrev() IJoint
	SetPrev(IJoint)

	GetEdgeA() *JointEdge
	GetEdgeB() *JointEdge

	// Get the user data pointer.
	GetUserData() interface{}

	// Set the user data pointer.
	SetUserData(data interface{})

	// Short-cut function to determine if either body is inactive.
	IsActive() bool

	// Get collide connected.
	// Note: modifying the collide connect flag won't work correctly because
	// the flag is only checked when fixture AABBs begin to overlap.
	GetCollideConnected() bool

	//
	GetIsland() bool
	SetIsland(flag bool)

	//
	GetIndex() int
	SetIndex(int)

	InitVelocityConstraints(data *solverData)
	SolveVelocityConstraints(data *solverData)
	SolvePositionConstraints(data *solverData) bool

	// Dump this joint to the log file.
	Dump()
}

// The base joint class. Joints are used to constraint two bodies together in
// various fashions. Some joints also feature limits and motors.
type Joint struct {
	Type  JointType
	Prev  IJoint
	Next  IJoint
	EdgeA JointEdge
	EdgeB JointEdge
	BodyA *Body
	BodyB *Body

	Index int

	IslandFlag       bool
	CollideConnected bool

	UserData interface{}
}

func (this *Joint) GetType() JointType {
	return this.Type
}

/// Get the first body attached to this joint.
func (this *Joint) GetBodyA() *Body {
	return this.BodyA
}

/// Get the second body attached to this joint.
func (this *Joint) GetBodyB() *Body {
	return this.BodyB
}

// Get the next joint the world joint list.
func (this *Joint) GetNext() IJoint {
	return this.Next
}

func (this *Joint) SetNext(next IJoint) {
	this.Next = next
}

func (this *Joint) GetPrev() IJoint {
	return this.Prev
}

func (this *Joint) SetPrev(prev IJoint) {
	this.Prev = prev
}

func (this *Joint) GetEdgeA() *JointEdge {
	return &this.EdgeA
}

func (this *Joint) GetEdgeB() *JointEdge {
	return &this.EdgeB
}

// Get the user data pointer.
func (this *Joint) GetUserData() interface{} {
	return this.UserData
}

// Set the user data pointer.
func (this *Joint) SetUserData(data interface{}) {
	this.UserData = data
}

// Short-cut function to determine if either body is inactive.
func (this *Joint) IsActive() bool {
	return this.BodyA.IsActive() && this.BodyB.IsActive()
}

// Get collide connected.
// Note: modifying the collide connect flag won't work correctly because
// the flag is only checked when fixture AABBs begin to overlap.
func (this *Joint) GetCollideConnected() bool {
	return this.CollideConnected
}

func (this *Joint) GetIsland() bool {
	return this.IslandFlag
}

func (this *Joint) SetIsland(flag bool) {
	this.IslandFlag = flag
}

func (this *Joint) GetIndex() int {
	return this.Index
}

func (this *Joint) SetIndex(index int) {
	this.Index = index
}

func Joint_Create(def IJointDef) IJoint {
	var joint IJoint

	switch def.GetType() {
	case Joint_e_distanceJoint:
		joint = NewDistanceJoint(def.(*DistanceJointDef))
	case Joint_e_mouseJoint:
		joint = NewMouseJoint(def.(*MouseJointDef))
	case Joint_e_prismaticJoint:
		joint = NewPrismaticJoint(def.(*PrismaticJointDef))
	case Joint_e_revoluteJoint:
		joint = NewRevoluteJoint(def.(*RevoluteJointDef))
	case Joint_e_pulleyJoint:
		joint = NewPulleyJoint(def.(*PulleyJointDef))
	case Joint_e_gearJoint:
		joint = NewGearJoint(def.(*GearJointDef))
	case Joint_e_wheelJoint:
		joint = NewWheelJoint(def.(*WheelJointDef))
	case Joint_e_weldJoint:
		joint = NewWeldJoint(def.(*WeldJointDef))
	case Joint_e_frictionJoint:
		joint = NewFrictionJoint(def.(*FrictionJointDef))
	case Joint_e_ropeJoint:
		joint = NewRopeJoint(def.(*RopeJointDef))
	}

	return joint
}

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
type DistanceJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The natural length between the anchor points.
	Length float64

	/// The mass-spring-damper frequency in Hertz. A value of 0
	/// disables softness.
	FrequencyHz float64

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	DampingRatio float64
}

func NewDistanceJointDef() *DistanceJointDef {
	this := new(DistanceJointDef)
	this.Type = Joint_e_distanceJoint
	this.LocalAnchorA.Set(0.0, 0.0)
	this.LocalAnchorB.Set(0.0, 0.0)
	this.Length = 1.0
	this.FrequencyHz = 0.0
	this.DampingRatio = 0.0
	return this
}

// Initialize the bodies, anchors, and length using the world
// anchors.
func (this *DistanceJointDef) Initialize(bodyA *Body, bodyB *Body, anchorA Vec2, anchorB Vec2) {
	this.BodyA = bodyA
	this.BodyB = bodyB
	this.LocalAnchorA = bodyA.GetLocalPoint(anchorA)
	this.LocalAnchorB = bodyB.GetLocalPoint(anchorB)
	d := SubVV(anchorB, anchorA)
	this.Length = d.Length()
}

// A distance joint constrains two points on two bodies
// to remain at a fixed distance from each other. You can view
// this as a massless, rigid rod.
type DistanceJoint struct {
	Joint

	FrequencyHz  float64
	DampingRatio float64
	Bias         float64

	// Solver shared
	LocalAnchorA Vec2
	LocalAnchorB Vec2
	Gamma        float64
	Impulse      float64
	Length       float64

	// Solver temp
	IndexA       int
	IndexB       int
	U            Vec2
	RA           Vec2
	RB           Vec2
	LocalCenterA Vec2
	LocalCenterB Vec2
	InvMassA     float64
	InvMassB     float64
	InvIA        float64
	InvIB        float64
	Mass         float64
}

func NewDistanceJoint(def *DistanceJointDef) *DistanceJoint {
	this := new(DistanceJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.LocalAnchorA = def.LocalAnchorA
	this.LocalAnchorB = def.LocalAnchorB
	this.Length = def.Length
	this.FrequencyHz = def.FrequencyHz
	this.DampingRatio = def.DampingRatio
	return this
}

func (this *DistanceJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *DistanceJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

/// Get the reaction force given the inverse time step.
/// Unit is N.
func (this *DistanceJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt*this.Impulse, this.U)
}

/// Get the reaction torque given the inverse time step.
/// Unit is N*m. This is always zero for a distance joint.
func (this *DistanceJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

/// The local anchor point relative to bodyA's origin.
func (this *DistanceJoint) GetLocalAnchorA() Vec2 {
	return this.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (this *DistanceJoint) GetLocalAnchorB() Vec2 {
	return this.LocalAnchorB
}

/// Set/get the natural length.
/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
func (this *DistanceJoint) SetLength(length float64) {
	this.Length = length
}

func (this *DistanceJoint) GetLength() float64 {
	return this.Length
}

/// Set/get frequency in Hz.
func (this *DistanceJoint) SetFrequency(hz float64) {
	this.FrequencyHz = hz
}

func (this *DistanceJoint) GetFrequency() float64 {
	return this.FrequencyHz
}

/// Set/get damping ratio.
func (this *DistanceJoint) SetDampingRatio(ratio float64) {
	this.DampingRatio = ratio
}

func (this *DistanceJoint) GetDampingRatio() float64 {
	return this.DampingRatio
}

func (this *DistanceJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterA = this.BodyA.sweep.LocalCenter
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassA = this.BodyA.invMass
	this.InvMassB = this.BodyB.invMass
	this.InvIA = this.BodyA.invI
	this.InvIB = this.BodyB.invI

	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qA := NewRot(aA)
	qB := NewRot(aB)

	this.RA = MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	this.RB = MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))
	this.U = SubVV(AddVV(cB, this.RB), AddVV(cA, this.RA))

	// Handle singularity.
	length := this.U.Length()
	if length > LinearSlop {
		this.U.Mul(1.0 / length)
	} else {
		this.U.Set(0.0, 0.0)
	}

	crAu := CrossVV(this.RA, this.U)
	crBu := CrossVV(this.RB, this.U)
	invMass := this.InvMassA + this.InvIA*crAu*crAu + this.InvMassB + this.InvIB*crBu*crBu

	// Compute the effective mass matrix.
	this.Mass = 0.0
	if invMass != 0.0 {
		this.Mass = 1.0 / invMass
	}

	if this.FrequencyHz > 0.0 {
		C := length - this.Length

		// Frequency
		omega := 2.0 * Pi * this.FrequencyHz

		// Damping coefficient
		d := 2.0 * this.Mass * this.DampingRatio * omega

		// Spring stiffness
		k := this.Mass * omega * omega

		// magic formulas
		h := data.step.dt
		this.Gamma = h * (d + h*k)
		this.Gamma = 0.0
		if this.Gamma != 0.0 {
			this.Gamma = 1.0 / this.Gamma
		}
		this.Bias = C * h * k * this.Gamma

		invMass += this.Gamma
		this.Mass = 0.0
		if invMass != 0.0 {
			this.Mass = 1.0 / invMass
		}
	} else {
		this.Gamma = 0.0
		this.Bias = 0.0
	}

	if data.step.warmStarting {
		// Scale the impulse to support a variable time step.
		this.Impulse *= data.step.dtRatio

		P := MulFV(this.Impulse, this.U)
		vA.Sub(MulFV(this.InvMassA, P))
		wA -= this.InvIA * CrossVV(this.RA, P)
		vB.Add(MulFV(this.InvMassB, P))
		wB += this.InvIB * CrossVV(this.RB, P)
	} else {
		this.Impulse = 0.0
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *DistanceJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	// Cdot = dot(u, v + cross(w, r))
	vpA := AddVV(vA, CrossFV(wA, this.RA))
	vpB := AddVV(vB, CrossFV(wB, this.RB))
	Cdot := DotVV(this.U, SubVV(vpB, vpA))

	impulse := -this.Mass * (Cdot + this.Bias + this.Gamma*this.Impulse)
	this.Impulse += impulse

	P := MulFV(impulse, this.U)
	vA.Sub(MulFV(this.InvMassA, P))
	wA -= this.InvIA * CrossVV(this.RA, P)
	vB.Add(MulFV(this.InvMassB, P))
	wB += this.InvIB * CrossVV(this.RB, P)

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *DistanceJoint) SolvePositionConstraints(data *solverData) bool {
	if this.FrequencyHz > 0.0 {
		// There is no position correction for soft distance constraints.
		return true
	}

	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a

	qA := NewRot(aA)
	qB := NewRot(aB)

	rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))
	u := SubVV(AddVV(cB, rB), AddVV(cA, rA))

	length := u.Normalize()
	C := length - this.Length
	C = ClampF(C, -MaxLinearCorrection, MaxLinearCorrection)

	impulse := -this.Mass * C
	P := MulFV(impulse, u)

	cA.Sub(MulFV(this.InvMassA, P))
	aA -= this.InvIA * CrossVV(rA, P)
	cB.Add(MulFV(this.InvMassB, P))
	aB += this.InvIB * CrossVV(rB, P)

	data.positions[this.IndexA].c = cA
	data.positions[this.IndexA].a = aA
	data.positions[this.IndexB].c = cB
	data.positions[this.IndexB].a = aB

	return AbsF(C) < LinearSlop
}

/// Dump joint to dmLog
func (this *DistanceJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	Log("  b2DistanceJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", this.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this.LocalAnchorA.X, this.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this.LocalAnchorB.X, this.LocalAnchorB.Y)
	Log("  jd.length = %.15lef;\n", this.Length)
	Log("  jd.frequencyHz = %.15lef;\n", this.FrequencyHz)
	Log("  jd.dampingRatio = %.15lef;\n", this.DampingRatio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
type MouseJointDef struct {
	JointDef

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	Target Vec2

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	MaxForce float64

	/// The response speed.
	FrequencyHz float64

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	DampingRatio float64
}

func NewMouseJointDef() *MouseJointDef {
	this := new(MouseJointDef)
	this.Type = Joint_e_mouseJoint
	this.Target.Set(0.0, 0.0)
	this.MaxForce = 0.0
	this.FrequencyHz = 5.0
	this.DampingRatio = 0.7
	return this
}

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
type MouseJoint struct {
	Joint

	LocalAnchorB Vec2
	TargetA      Vec2
	FrequencyHz  float64
	DampingRatio float64
	Beta         float64

	// Solver shared
	Impulse  Vec2
	MaxForce float64
	Gamma    float64

	// Solver temp
	IndexA       int
	IndexB       int
	RB           Vec2
	LocalCenterB Vec2
	InvMassB     float64
	InvIB        float64
	Mass         Mat22
	C            Vec2
}

func NewMouseJoint(def *MouseJointDef) *MouseJoint {
	this := new(MouseJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.TargetA = def.Target
	this.LocalAnchorB = MulXT(this.BodyB.GetTransform(), this.TargetA)
	this.MaxForce = def.MaxForce
	this.Impulse.SetZero()
	this.FrequencyHz = def.FrequencyHz
	this.DampingRatio = def.DampingRatio
	return this
}

/// Implements b2Joint.
func (this *MouseJoint) GetAnchorA() Vec2 {
	return this.TargetA
}

/// Implements b2Joint.
func (this *MouseJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

/// Implements b2Joint.
func (this *MouseJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt, this.Impulse)
}

/// Implements b2Joint.
func (this *MouseJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * 0.0
}

/// Use this to update the target point.
func (this *MouseJoint) SetTarget(target Vec2) {
	if !this.BodyB.IsAwake() {
		this.BodyB.SetAwake(true)
	}
	this.TargetA = target
}

func (this *MouseJoint) GetTarget() Vec2 {
	return this.TargetA
}

/// Set/get the maximum force in Newtons.
func (this *MouseJoint) SetMaxForce(force float64) {
	this.MaxForce = force
}

func (this *MouseJoint) GetMaxForce() float64 {
	return this.MaxForce
}

/// Set/get the frequency in Hertz.
func (this *MouseJoint) SetFrequency(hz float64) {
	this.FrequencyHz = hz
}

func (this *MouseJoint) GetFrequency() float64 {
	return this.FrequencyHz
}

/// Set/get the damping ratio (dimensionless).
func (this *MouseJoint) SetDampingRatio(ratio float64) {
	this.DampingRatio = ratio
}

func (this *MouseJoint) GetDampingRatio() float64 {
	return this.DampingRatio
}

/// The mouse joint does not support dumping.
func (this *MouseJoint) Dump() {
	Log("Mouse joint dumping is not supported.\n")
}

func (this *MouseJoint) InitVelocityConstraints(data *solverData) {
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassB = this.BodyB.invMass
	this.InvIB = this.BodyB.invI

	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qB := NewRot(aB)

	mass := this.BodyB.GetMass()

	// Frequency
	omega := 2.0 * Pi * this.FrequencyHz

	// Damping coefficient
	d := 2.0 * mass * this.DampingRatio * omega

	// Spring stiffness
	k := mass * (omega * omega)

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	h := data.step.dt
	//b2Assert(d + h * k > b2_epsilon);
	this.Gamma = h * (d + h*k)
	if this.Gamma != 0.0 {
		this.Gamma = 1.0 / this.Gamma
	}
	this.Beta = h * k * this.Gamma

	// Compute the effective mass matrix.
	this.RB = MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	var K Mat22
	K.Ex.X = this.InvMassB + this.InvIB*this.RB.Y*this.RB.Y + this.Gamma
	K.Ex.Y = -this.InvIB * this.RB.X * this.RB.Y
	K.Ey.X = K.Ex.Y
	K.Ey.Y = this.InvMassB + this.InvIB*this.RB.X*this.RB.X + this.Gamma

	this.Mass = K.GetInverse()

	this.C = SubVV(AddVV(cB, this.RB), this.TargetA)
	this.C.Mul(this.Beta)

	// Cheat with some damping
	wB *= 0.98

	if data.step.warmStarting {
		this.Impulse.Mul(data.step.dtRatio)
		vB.Add(MulFV(this.InvMassB, this.Impulse))
		wB += this.InvIB * CrossVV(this.RB, this.Impulse)
	} else {
		this.Impulse.SetZero()
	}

	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *MouseJoint) SolveVelocityConstraints(data *solverData) {
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	// Cdot = v + cross(w, r)
	Cdot := AddVV(vB, CrossFV(wB, this.RB))
	tmpv := AddVV(AddVV(Cdot, this.C), MulFV(this.Gamma, this.Impulse))
	impulse := MulMV(this.Mass, tmpv.Minus())

	oldImpulse := this.Impulse
	this.Impulse.Add(impulse)
	maxImpulse := data.step.dt * this.MaxForce
	if this.Impulse.LengthSquared() > maxImpulse*maxImpulse {
		this.Impulse.Mul(maxImpulse / this.Impulse.Length())
	}
	impulse = SubVV(this.Impulse, oldImpulse)

	vB.Add(MulFV(this.InvMassB, impulse))
	wB += this.InvIB * CrossVV(this.RB, impulse)

	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *MouseJoint) SolvePositionConstraints(data *solverData) bool {
	return true
}

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
type PrismaticJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The local translation unit axis in bodyA.
	LocalAxisA Vec2

	/// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	ReferenceAngle float64

	/// Enable/disable the joint limit.
	EnableLimit bool

	/// The lower translation limit, usually in meters.
	LowerTranslation float64

	/// The upper translation limit, usually in meters.
	UpperTranslation float64

	/// Enable/disable the joint motor.
	EnableMotor bool

	/// The maximum motor torque, usually in N-m.
	MaxMotorForce float64

	/// The desired motor speed in radians per second.
	MotorSpeed float64
}

func NewPrismaticJointDef() *PrismaticJointDef {
	this := new(PrismaticJointDef)
	this.Type = Joint_e_prismaticJoint
	this.LocalAnchorA.SetZero()
	this.LocalAnchorB.SetZero()
	this.LocalAxisA.Set(1.0, 0.0)
	this.ReferenceAngle = 0.0
	this.EnableLimit = false
	this.LowerTranslation = 0.0
	this.UpperTranslation = 0.0
	this.EnableMotor = false
	this.MaxMotorForce = 0.0
	this.MotorSpeed = 0.0
	return this
}

// Initialize the bodies, anchors, axis, and reference angle using the world
// anchor and unit world axis.
func (this *PrismaticJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2, axis Vec2) {
	this.BodyA = bodyA
	this.BodyB = bodyB
	this.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	this.LocalAnchorB = bodyB.GetLocalPoint(anchor)
	this.LocalAxisA = bodyA.GetLocalVector(axis)
	this.ReferenceAngle = bodyB.GetAngle() - bodyA.GetAngle()
}

// A prismatic joint. This joint provides one degree of freedom: translation
// along an axis fixed in bodyA. Relative rotation is prevented. You can
// use a joint limit to restrict the range of motion and a joint motor to
// drive the motion or to model joint friction.
type PrismaticJoint struct {
	Joint

	// Solver shared
	LocalAnchorA     Vec2
	LocalAnchorB     Vec2
	LocalXAxisA      Vec2
	LocalYAxisA      Vec2
	ReferenceAngle   float64
	Impulse          Vec3
	MotorImpulse     float64
	LowerTranslation float64
	UpperTranslation float64
	MaxMotorForce    float64
	MotorSpeed       float64
	EnableLimit      bool
	EnableMotor      bool
	LimitState       LimitState

	// Solver temp
	IndexA       int
	IndexB       int
	LocalCenterA Vec2
	LocalCenterB Vec2
	InvMassA     float64
	InvMassB     float64
	InvIA        float64
	InvIB        float64
	Axis, Perp   Vec2
	S1, S2       float64
	A1, A2       float64
	K            Mat33
	MotorMass    float64
}

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

func NewPrismaticJoint(def *PrismaticJointDef) *PrismaticJoint {
	this := new(PrismaticJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.LocalAnchorA = def.LocalAnchorA
	this.LocalAnchorB = def.LocalAnchorB
	this.LocalXAxisA = def.LocalAxisA
	this.LocalXAxisA.Normalize()
	this.LocalYAxisA = CrossFV(1.0, this.LocalXAxisA)
	this.ReferenceAngle = def.ReferenceAngle

	this.Impulse.SetZero()
	this.MotorMass = 0.0
	this.MotorImpulse = 0.0

	this.LowerTranslation = def.LowerTranslation
	this.UpperTranslation = def.UpperTranslation
	this.MaxMotorForce = def.MaxMotorForce
	this.MotorSpeed = def.MotorSpeed
	this.EnableLimit = def.EnableLimit
	this.EnableMotor = def.EnableMotor
	this.LimitState = LimitState_e_inactiveLimit

	this.Axis.SetZero()
	this.Perp.SetZero()
	return this
}

func (this *PrismaticJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *PrismaticJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

func (this *PrismaticJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt, AddVV(MulFV(this.Impulse.X, this.Perp), MulFV(this.MotorImpulse+this.Impulse.Z, this.Axis)))
}

func (this *PrismaticJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * this.Impulse.Y
}

/// The local anchor point relative to bodyA's origin.
func (this *PrismaticJoint) GetLocalAnchorA() Vec2 {
	return this.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (this *PrismaticJoint) GetLocalAnchorB() Vec2 {
	return this.LocalAnchorB
}

/// The local joint axis relative to bodyA.
func (this *PrismaticJoint) GetLocalAxisA() Vec2 {
	return this.LocalXAxisA
}

/// Get the reference angle.
func (this *PrismaticJoint) GetReferenceAngle() float64 {
	return this.ReferenceAngle
}

/// Get the current joint translation, usually in meters.
func (this *PrismaticJoint) GetJointTranslation() float64 {
	pA := this.BodyA.GetWorldPoint(this.LocalAnchorA)
	pB := this.BodyB.GetWorldPoint(this.LocalAnchorB)
	d := SubVV(pB, pA)
	axis := this.BodyA.GetWorldVector(this.LocalXAxisA)

	translation := DotVV(d, axis)
	return translation
}

/// Get the current joint translation speed, usually in meters per second.
func (this *PrismaticJoint) GetJointSpeed() float64 {
	bA := this.BodyA
	bB := this.BodyB

	rA := MulRV(bA.xf.Q, SubVV(this.LocalAnchorA, bA.sweep.LocalCenter))
	rB := MulRV(bB.xf.Q, SubVV(this.LocalAnchorB, bB.sweep.LocalCenter))
	p1 := AddVV(bA.sweep.C, rA)
	p2 := AddVV(bB.sweep.C, rB)
	d := SubVV(p2, p1)
	axis := MulRV(bA.xf.Q, this.LocalXAxisA)

	vA := bA.linearVelocity
	vB := bB.linearVelocity
	wA := bA.angularVelocity
	wB := bB.angularVelocity

	speed := DotVV(d, CrossFV(wA, axis)) + DotVV(axis, SubVV(AddVV(vB, CrossFV(wB, rB)), AddVV(vA, CrossFV(wA, rA))))
	return speed
}

/// Is the joint limit enabled?
func (this *PrismaticJoint) IsLimitEnabled() bool {
	return this.EnableLimit
}

/// Enable/disable the joint limit.
func (this *PrismaticJoint) SetEnableLimit(flag bool) {
	if flag != this.EnableLimit {
		this.BodyA.SetAwake(true)
		this.BodyB.SetAwake(true)
		this.EnableLimit = flag
		this.Impulse.Z = 0.0
	}
}

/// Get the lower joint limit, usually in meters.
func (this *PrismaticJoint) GetLowerLimit() float64 {
	return this.LowerTranslation
}

/// Get the upper joint limit, usually in meters.
func (this *PrismaticJoint) GetUpperLimit() float64 {
	return this.UpperTranslation
}

/// Set the joint limits, usually in meters.
func (this *PrismaticJoint) SetLimits(lower float64, upper float64) {
	//b2Assert(lower <= upper);
	if lower != this.LowerTranslation || upper != this.UpperTranslation {
		this.BodyA.SetAwake(true)
		this.BodyB.SetAwake(true)
		this.LowerTranslation = lower
		this.UpperTranslation = upper
		this.Impulse.Z = 0.0
	}
}

/// Is the joint motor enabled?
func (this *PrismaticJoint) IsMotorEnabled() bool {
	return this.EnableMotor
}

/// Enable/disable the joint motor.
func (this *PrismaticJoint) SetEnableMotor(flag bool) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.EnableMotor = flag
}

/// Set the motor speed, usually in meters per second.
func (this *PrismaticJoint) SetMotorSpeed(speed float64) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.MotorSpeed = speed
}

/// Get the motor speed, usually in meters per second.
func (this *PrismaticJoint) GetMotorSpeed() float64 {
	return this.MotorSpeed
}

/// Set the maximum motor force, usually in N.
func (this *PrismaticJoint) SetMaxMotorForce(force float64) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.MaxMotorForce = force
}

func (this *PrismaticJoint) GetMaxMotorForce() float64 {
	return this.MaxMotorForce
}

/// Get the current motor force given the inverse time step, usually in N.
func (this *PrismaticJoint) GetMotorForce(inv_dt float64) float64 {
	return inv_dt * this.MotorImpulse
}

func (this *PrismaticJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterA = this.BodyA.sweep.LocalCenter
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassA = this.BodyA.invMass
	this.InvMassB = this.BodyB.invMass
	this.InvIA = this.BodyA.invI
	this.InvIB = this.BodyB.invI

	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qA := NewRot(aA)
	qB := NewRot(aB)

	// Compute the effective masses.
	rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))
	d := AddVV(SubVV(cB, cA), SubVV(rB, rA))

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	// Compute motor Jacobian and effective mass.
	{
		this.Axis = MulRV(qA, this.LocalXAxisA)
		this.A1 = CrossVV(AddVV(d, rA), this.Axis)
		this.A2 = CrossVV(rB, this.Axis)

		this.MotorMass = mA + mB + iA*this.A1*this.A1 + iB*this.A2*this.A2
		if this.MotorMass > 0.0 {
			this.MotorMass = 1.0 / this.MotorMass
		}
	}

	// Prismatic constraint.
	{
		this.Perp = MulRV(qA, this.LocalYAxisA)

		this.S1 = CrossVV(AddVV(d, rA), this.Perp)
		this.S2 = CrossVV(rB, this.Perp)

		k11 := mA + mB + iA*this.S1*this.S1 + iB*this.S2*this.S2
		k12 := iA*this.S1 + iB*this.S2
		k13 := iA*this.S1*this.A1 + iB*this.S2*this.A2
		k22 := iA + iB
		if k22 == 0.0 {
			// For bodies with fixed rotation.
			k22 = 1.0
		}
		k23 := iA*this.A1 + iB*this.A2
		k33 := mA + mB + iA*this.A1*this.A1 + iB*this.A2*this.A2

		this.K.Ex.Set(k11, k12, k13)
		this.K.Ey.Set(k12, k22, k23)
		this.K.Ez.Set(k13, k23, k33)
	}

	// Compute motor and limit terms.
	if this.EnableLimit {
		jointTranslation := DotVV(this.Axis, d)
		if AbsF(this.UpperTranslation-this.LowerTranslation) < 2.0*LinearSlop {
			this.LimitState = LimitState_e_equalLimits
		} else if jointTranslation <= this.LowerTranslation {
			if this.LimitState != LimitState_e_atLowerLimit {
				this.LimitState = LimitState_e_atLowerLimit
				this.Impulse.Z = 0.0
			}
		} else if jointTranslation >= this.UpperTranslation {
			if this.LimitState != LimitState_e_atUpperLimit {
				this.LimitState = LimitState_e_atUpperLimit
				this.Impulse.Z = 0.0
			}
		} else {
			this.LimitState = LimitState_e_inactiveLimit
			this.Impulse.Z = 0.0
		}
	} else {
		this.LimitState = LimitState_e_inactiveLimit
		this.Impulse.Z = 0.0
	}

	if !this.EnableMotor {
		this.MotorImpulse = 0.0
	}

	if data.step.warmStarting {
		// Account for variable time step.
		this.Impulse.Mul(data.step.dtRatio)
		this.MotorImpulse *= data.step.dtRatio

		P := AddVV(MulFV(this.Impulse.X, this.Perp), MulFV(this.MotorImpulse+this.Impulse.Z, this.Axis))
		LA := this.Impulse.X*this.S1 + this.Impulse.Y + (this.MotorImpulse+this.Impulse.Z)*this.A1
		LB := this.Impulse.X*this.S2 + this.Impulse.Y + (this.MotorImpulse+this.Impulse.Z)*this.A2

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	} else {
		this.Impulse.SetZero()
		this.MotorImpulse = 0.0
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *PrismaticJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	// Solve linear motor constraint.
	if this.EnableMotor && this.LimitState != LimitState_e_equalLimits {
		Cdot := DotVV(this.Axis, SubVV(vB, vA)) + this.A2*wB - this.A1*wA
		impulse := this.MotorMass * (this.MotorSpeed - Cdot)
		oldImpulse := this.MotorImpulse
		maxImpulse := data.step.dt * this.MaxMotorForce
		this.MotorImpulse = ClampF(this.MotorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = this.MotorImpulse - oldImpulse

		P := MulFV(impulse, this.Axis)
		LA := impulse * this.A1
		LB := impulse * this.A2

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	}

	var Cdot1 Vec2
	Cdot1.X = DotVV(this.Perp, SubVV(vB, vA)) + this.S2*wB - this.S1*wA
	Cdot1.Y = wB - wA

	if this.EnableLimit && this.LimitState != LimitState_e_inactiveLimit {
		// Solve prismatic and limit constraint in block form.
		var Cdot2 float64
		Cdot2 = DotVV(this.Axis, SubVV(vB, vA)) + this.A2*wB - this.A1*wA
		Cdot := Vec3{Cdot1.X, Cdot1.Y, Cdot2}

		f1 := this.Impulse
		df := this.K.Solve33(Cdot.Minus())
		this.Impulse.Add(df)

		if this.LimitState == LimitState_e_atLowerLimit {
			this.Impulse.Z = MaxF(this.Impulse.Z, 0.0)
		} else if this.LimitState == LimitState_e_atUpperLimit {
			this.Impulse.Z = MinF(this.Impulse.Z, 0.0)
		}

		// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
		b := SubVV(Cdot1.Minus(), MulFV((this.Impulse.Z-f1.Z), Vec2{this.K.Ez.X, this.K.Ez.Y}))
		f2r := AddVV(this.K.Solve22(b), Vec2{f1.X, f1.Y})
		this.Impulse.X = f2r.X
		this.Impulse.Y = f2r.Y

		df = SubV3V3(this.Impulse, f1)

		P := AddVV(MulFV(df.X, this.Perp), MulFV(df.Z, this.Axis))
		LA := df.X*this.S1 + df.Y + df.Z*this.A1
		LB := df.X*this.S2 + df.Y + df.Z*this.A2

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	} else {
		// Limit is inactive, just solve the prismatic constraint in block form.
		df := this.K.Solve22(Cdot1.Minus())
		this.Impulse.X += df.X
		this.Impulse.Y += df.Y

		P := MulFV(df.X, this.Perp)
		LA := df.X*this.S1 + df.Y
		LB := df.X*this.S2 + df.Y

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB

		//Cdot10 := Cdot1

		Cdot1.X = DotVV(this.Perp, SubVV(vB, vA)) + this.S2*wB - this.S1*wA
		Cdot1.Y = wB - wA

		if AbsF(Cdot1.X) > 0.01 || AbsF(Cdot1.Y) > 0.01 {
			//test := MulM3V(this.K, df)
			Cdot1.X += 0.0
		}
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *PrismaticJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a

	qA := NewRot(aA)
	qB := NewRot(aB)

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	// Compute fresh Jacobians
	rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))
	d := SubVV(AddVV(cB, rB), AddVV(cA, rA))

	axis := MulRV(qA, this.LocalXAxisA)
	a1 := CrossVV(AddVV(d, rA), axis)
	a2 := CrossVV(rB, axis)
	perp := MulRV(qA, this.LocalYAxisA)

	s1 := CrossVV(AddVV(d, rA), perp)
	s2 := CrossVV(rB, perp)

	var impulse Vec3
	var C1 Vec2
	C1.X = DotVV(perp, d)
	C1.Y = aB - aA - this.ReferenceAngle

	linearError := AbsF(C1.X)
	angularError := AbsF(C1.Y)

	active := false
	C2 := 0.0
	if this.EnableLimit {
		translation := DotVV(axis, d)
		if AbsF(this.UpperTranslation-this.LowerTranslation) < 2.0*LinearSlop {
			// Prevent large angular corrections
			C2 = ClampF(translation, -MaxLinearCorrection, MaxLinearCorrection)
			linearError = MaxF(linearError, AbsF(translation))
			active = true
		} else if translation <= this.LowerTranslation {
			// Prevent large linear corrections and allow some slop.
			C2 = ClampF(translation-this.LowerTranslation+LinearSlop, -MaxLinearCorrection, 0.0)
			linearError = MaxF(linearError, this.LowerTranslation-translation)
			active = true
		} else if translation >= this.UpperTranslation {
			// Prevent large linear corrections and allow some slop.
			C2 = ClampF(translation-this.UpperTranslation-LinearSlop, 0.0, MaxLinearCorrection)
			linearError = MaxF(linearError, translation-this.UpperTranslation)
			active = true
		}
	}

	if active {
		k11 := mA + mB + iA*s1*s1 + iB*s2*s2
		k12 := iA*s1 + iB*s2
		k13 := iA*s1*a1 + iB*s2*a2
		k22 := iA + iB
		if k22 == 0.0 {
			// For fixed rotation
			k22 = 1.0
		}
		k23 := iA*a1 + iB*a2
		k33 := mA + mB + iA*a1*a1 + iB*a2*a2

		var K Mat33
		K.Ex.Set(k11, k12, k13)
		K.Ey.Set(k12, k22, k23)
		K.Ez.Set(k13, k23, k33)

		var C Vec3
		C.X = C1.X
		C.Y = C1.Y
		C.Z = C2

		impulse = K.Solve33(C.Minus())
	} else {
		k11 := mA + mB + iA*s1*s1 + iB*s2*s2
		k12 := iA*s1 + iB*s2
		k22 := iA + iB
		if k22 == 0.0 {
			k22 = 1.0
		}

		var K Mat22
		K.Ex.Set(k11, k12)
		K.Ey.Set(k12, k22)

		impulse1 := K.Solve(C1.Minus())
		impulse.X = impulse1.X
		impulse.Y = impulse1.Y
		impulse.Z = 0.0
	}

	P := AddVV(MulFV(impulse.X, perp), MulFV(impulse.Z, axis))
	LA := impulse.X*s1 + impulse.Y + impulse.Z*a1
	LB := impulse.X*s2 + impulse.Y + impulse.Z*a2

	cA.Sub(MulFV(mA, P))
	aA -= iA * LA
	cB.Add(MulFV(mB, P))
	aB += iB * LB

	data.positions[this.IndexA].c = cA
	data.positions[this.IndexA].a = aA
	data.positions[this.IndexB].c = cB
	data.positions[this.IndexB].a = aB

	return linearError <= LinearSlop && angularError <= AngularSlop
}

/// Dump to b2Log
func (this *PrismaticJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	Log("  b2PrismaticJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%d);\n", this.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this.LocalAnchorA.X, this.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this.LocalAnchorB.X, this.LocalAnchorB.Y)
	Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", this.LocalXAxisA.X, this.LocalXAxisA.Y)
	Log("  jd.referenceAngle = %.15lef;\n", this.ReferenceAngle)
	Log("  jd.enableLimit = bool(%d);\n", this.EnableLimit)
	Log("  jd.lowerTranslation = %.15lef;\n", this.LowerTranslation)
	Log("  jd.upperTranslation = %.15lef;\n", this.UpperTranslation)
	Log("  jd.enableMotor = bool(%d);\n", this.EnableMotor)
	Log("  jd.motorSpeed = %.15lef;\n", this.MotorSpeed)
	Log("  jd.maxMotorForce = %.15lef;\n", this.MaxMotorForce)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
type RevoluteJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	ReferenceAngle float64

	/// A flag to enable joint limits.
	EnableLimit bool

	/// The lower angle for the joint limit (radians).
	LowerAngle float64

	/// The upper angle for the joint limit (radians).
	UpperAngle float64

	/// A flag to enable the joint motor.
	EnableMotor bool

	/// The desired motor speed. Usually in radians per second.
	MotorSpeed float64

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	MaxMotorTorque float64
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2
func NewRevoluteJointDef() *RevoluteJointDef {
	this := new(RevoluteJointDef)
	this.Type = Joint_e_revoluteJoint
	return this
}

/// Initialize the bodies, anchors, and reference angle using a world
/// anchor point.
func (this *RevoluteJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2) {
	this.BodyA = bodyA
	this.BodyB = bodyB
	this.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	this.LocalAnchorB = bodyB.GetLocalPoint(anchor)
	this.ReferenceAngle = bodyB.GetAngle() - bodyA.GetAngle()
}

/// A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
type RevoluteJoint struct {
	Joint

	// Solver shared
	LocalAnchorA Vec2
	LocalAnchorB Vec2
	Impulse      Vec3
	MotorImpulse float64

	EnableMotor    bool
	MaxMotorTorque float64
	MotorSpeed     float64

	EnableLimit    bool
	ReferenceAngle float64
	LowerAngle     float64
	UpperAngle     float64

	// Solver temp
	IndexA       int
	IndexB       int
	RA           Vec2
	RB           Vec2
	LocalCenterA Vec2
	LocalCenterB Vec2
	InvMassA     float64
	InvMassB     float64
	InvIA        float64
	InvIB        float64
	Mass         Mat33   // effective mass for point-to-point constraint.
	MotorMass    float64 // effective mass for motor/limit angular constraint.
	LimitState   LimitState
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2
func NewRevoluteJoint(def *RevoluteJointDef) *RevoluteJoint {
	this := new(RevoluteJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.LocalAnchorA = def.LocalAnchorA
	this.LocalAnchorB = def.LocalAnchorB
	this.ReferenceAngle = def.ReferenceAngle

	this.Impulse.SetZero()
	this.MotorImpulse = 0.0

	this.LowerAngle = def.LowerAngle
	this.UpperAngle = def.UpperAngle
	this.MaxMotorTorque = def.MaxMotorTorque
	this.MotorSpeed = def.MotorSpeed
	this.EnableLimit = def.EnableLimit
	this.EnableMotor = def.EnableMotor
	this.LimitState = LimitState_e_inactiveLimit
	return this
}

func (this *RevoluteJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *RevoluteJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

/// The local anchor point relative to bodyA's origin.
func (this *RevoluteJoint) GetLocalAnchorA() Vec2 {
	return this.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (this *RevoluteJoint) GetLocalAnchorB() Vec2 {
	return this.LocalAnchorB
}

/// Get the reference angle.
func (this *RevoluteJoint) GetReferenceAngle() float64 {
	return this.ReferenceAngle
}

/// Get the current joint angle in radians.
func (this *RevoluteJoint) GetJointAngle() float64 {
	bA := this.BodyA
	bB := this.BodyB
	return bB.sweep.A - bA.sweep.A - this.ReferenceAngle
}

/// Get the current joint angle speed in radians per second.
func (this *RevoluteJoint) GetJointSpeed() float64 {
	bA := this.BodyA
	bB := this.BodyB
	return bB.angularVelocity - bA.angularVelocity
}

/// Is the joint limit enabled?
func (this *RevoluteJoint) IsLimitEnabled() bool {
	return this.EnableLimit
}

/// Enable/disable the joint limit.
func (this *RevoluteJoint) SetEnableLimit(flag bool) {
	if flag != this.EnableLimit {
		this.BodyA.SetAwake(true)
		this.BodyB.SetAwake(true)
		this.EnableLimit = flag
		this.Impulse.Z = 0.0
	}
}

/// Get the lower joint limit in radians.
func (this *RevoluteJoint) GetLowerLimit() float64 {
	return this.LowerAngle
}

/// Get the upper joint limit in radians.
func (this *RevoluteJoint) GetUpperLimit() float64 {
	return this.UpperAngle
}

/// Set the joint limits in radians.
func (this *RevoluteJoint) SetLimits(lower float64, upper float64) {
	//b2Assert(lower <= upper);
	if lower != this.LowerAngle || upper != this.UpperAngle {
		this.BodyA.SetAwake(true)
		this.BodyB.SetAwake(true)
		this.Impulse.Z = 0.0
		this.LowerAngle = lower
		this.UpperAngle = upper
	}
}

/// Is the joint motor enabled?
func (this *RevoluteJoint) IsMotorEnabled() bool {
	return this.EnableMotor
}

/// Enable/disable the joint motor.
func (this *RevoluteJoint) SetEnableMotor(flag bool) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.EnableMotor = flag
}

/// Set the motor speed in radians per second.
func (this *RevoluteJoint) SetMotorSpeed(speed float64) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.MotorSpeed = speed
}

/// Get the motor speed in radians per second.
func (this *RevoluteJoint) GetMotorSpeed() float64 {
	return this.MotorSpeed
}

/// Set the maximum motor torque, usually in N-m.
func (this *RevoluteJoint) SetMaxMotorTorque(torque float64) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.MaxMotorTorque = torque
}

func (this *RevoluteJoint) GetMaxMotorTorque() float64 {
	return this.MaxMotorTorque
}

/// Get the reaction force given the inverse time step.
/// Unit is N.
func (this *RevoluteJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := Vec2{this.Impulse.X, this.Impulse.Y}
	return MulFV(inv_dt, P)
}

/// Get the reaction torque due to the joint limit given the inverse time step.
/// Unit is N*m.
func (this *RevoluteJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * this.Impulse.Z
}

/// Get the current motor torque given the inverse time step.
/// Unit is N*m.
func (this *RevoluteJoint) GetMotorTorque(inv_dt float64) float64 {
	return inv_dt * this.MotorImpulse
}

func (this *RevoluteJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterA = this.BodyA.sweep.LocalCenter
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassA = this.BodyA.invMass
	this.InvMassB = this.BodyB.invMass
	this.InvIA = this.BodyA.invI
	this.InvIB = this.BodyB.invI

	//cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	//cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qA := NewRot(aA)
	qB := NewRot(aB)

	this.RA = MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	this.RB = MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	fixedRotation := (iA+iB == 0.0)

	this.Mass.Ex.X = mA + mB + this.RA.Y*this.RA.Y*iA + this.RB.Y*this.RB.Y*iB
	this.Mass.Ey.X = -this.RA.Y*this.RA.X*iA - this.RB.Y*this.RB.X*iB
	this.Mass.Ez.X = -this.RA.Y*iA - this.RB.Y*iB
	this.Mass.Ex.Y = this.Mass.Ey.X
	this.Mass.Ey.Y = mA + mB + this.RA.X*this.RA.X*iA + this.RB.X*this.RB.X*iB
	this.Mass.Ez.Y = this.RA.X*iA + this.RB.X*iB
	this.Mass.Ex.Z = this.Mass.Ez.X
	this.Mass.Ey.Z = this.Mass.Ez.Y
	this.Mass.Ez.Z = iA + iB

	this.MotorMass = iA + iB
	if this.MotorMass > 0.0 {
		this.MotorMass = 1.0 / this.MotorMass
	}

	if !this.EnableMotor || fixedRotation {
		this.MotorImpulse = 0.0
	}

	if this.EnableLimit && !fixedRotation {
		jointAngle := aB - aA - this.ReferenceAngle
		if AbsF(this.UpperAngle-this.LowerAngle) < 2.0*AngularSlop {
			this.LimitState = LimitState_e_equalLimits
		} else if jointAngle <= this.LowerAngle {
			if this.LimitState != LimitState_e_atLowerLimit {
				this.Impulse.Z = 0.0
			}
			this.LimitState = LimitState_e_atLowerLimit
		} else if jointAngle >= this.UpperAngle {
			if this.LimitState != LimitState_e_atUpperLimit {
				this.Impulse.Z = 0.0
			}
			this.LimitState = LimitState_e_atUpperLimit
		} else {
			this.LimitState = LimitState_e_inactiveLimit
			this.Impulse.Z = 0.0
		}
	} else {
		this.LimitState = LimitState_e_inactiveLimit
	}

	if data.step.warmStarting {
		// Scale impulses to support a variable time step.
		this.Impulse.Mul(data.step.dtRatio)
		this.MotorImpulse *= data.step.dtRatio

		P := Vec2{this.Impulse.X, this.Impulse.Y}

		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(this.RA, P) + this.MotorImpulse + this.Impulse.Z)

		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(this.RB, P) + this.MotorImpulse + this.Impulse.Z)
	} else {
		this.Impulse.SetZero()
		this.MotorImpulse = 0.0
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *RevoluteJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	fixedRotation := (iA+iB == 0.0)

	// Solve motor constraint.
	if this.EnableMotor && this.LimitState != LimitState_e_equalLimits && !fixedRotation {
		Cdot := wB - wA - this.MotorSpeed
		impulse := -this.MotorMass * Cdot
		oldImpulse := this.MotorImpulse
		maxImpulse := data.step.dt * this.MaxMotorTorque
		this.MotorImpulse = ClampF(this.MotorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = this.MotorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve limit constraint.
	if this.EnableLimit && this.LimitState != LimitState_e_inactiveLimit && !fixedRotation {
		Cdot1 := SubVV(AddVV(vB, CrossFV(wB, this.RB)), AddVV(vA, CrossFV(wA, this.RA)))
		Cdot2 := wB - wA
		Cdot := Vec3{Cdot1.X, Cdot1.Y, Cdot2}

		impulse := this.Mass.Solve33(Cdot)
		impulse = impulse.Minus()

		if this.LimitState == LimitState_e_equalLimits {
			this.Impulse.Add(impulse)
		} else if this.LimitState == LimitState_e_atLowerLimit {
			newImpulse := this.Impulse.Z + impulse.Z
			if newImpulse < 0.0 {
				rhs := AddVV(Cdot1.Minus(), MulFV(this.Impulse.Z, Vec2{this.Mass.Ez.X, this.Mass.Ez.Y}))
				reduced := this.Mass.Solve22(rhs)
				impulse.X = reduced.X
				impulse.Y = reduced.Y
				impulse.Z = -this.Impulse.Z
				this.Impulse.X += reduced.X
				this.Impulse.Y += reduced.Y
				this.Impulse.Z = 0.0
			} else {
				this.Impulse.Add(impulse)
			}
		} else if this.LimitState == LimitState_e_atUpperLimit {
			newImpulse := this.Impulse.Z + impulse.Z
			if newImpulse > 0.0 {
				rhs := AddVV(Cdot1.Minus(), MulFV(this.Impulse.Z, Vec2{this.Mass.Ez.X, this.Mass.Ez.Y}))
				reduced := this.Mass.Solve22(rhs)
				impulse.X = reduced.X
				impulse.Y = reduced.Y
				impulse.Z = -this.Impulse.Z
				this.Impulse.X += reduced.X
				this.Impulse.Y += reduced.Y
				this.Impulse.Z = 0.0
			} else {
				this.Impulse.Add(impulse)
			}
		}

		P := Vec2{impulse.X, impulse.Y}

		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(this.RA, P) + impulse.Z)

		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(this.RB, P) + impulse.Z)
	} else {
		// Solve point-to-point constraint
		Cdot := SubVV(AddVV(vB, CrossFV(wB, this.RB)), AddVV(vA, CrossFV(wA, this.RA)))
		impulse := this.Mass.Solve22(Cdot.Minus())

		this.Impulse.X += impulse.X
		this.Impulse.Y += impulse.Y

		vA.Sub(MulFV(mA, impulse))
		wA -= iA * CrossVV(this.RA, impulse)

		vB.Add(MulFV(mB, impulse))
		wB += iB * CrossVV(this.RB, impulse)
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *RevoluteJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a

	qA := NewRot(aA)
	qB := NewRot(aB)

	angularError := 0.0
	positionError := 0.0

	fixedRotation := (this.InvIA+this.InvIB == 0.0)

	// Solve angular limit constraint.
	if this.EnableLimit && this.LimitState != LimitState_e_inactiveLimit && !fixedRotation {
		angle := aB - aA - this.ReferenceAngle
		limitImpulse := 0.0

		if this.LimitState == LimitState_e_equalLimits {
			// Prevent large angular corrections
			C := ClampF(angle-this.LowerAngle, -MaxAngularCorrection, MaxAngularCorrection)
			limitImpulse = -this.MotorMass * C
			angularError = AbsF(C)
		} else if this.LimitState == LimitState_e_atLowerLimit {
			C := angle - this.LowerAngle
			angularError = -C

			// Prevent large angular corrections and allow some slop.
			C = ClampF(C+AngularSlop, -MaxAngularCorrection, 0.0)
			limitImpulse = -this.MotorMass * C
		} else if this.LimitState == LimitState_e_atUpperLimit {
			C := angle - this.UpperAngle
			angularError = C

			// Prevent large angular corrections and allow some slop.
			C = ClampF(C-AngularSlop, 0.0, MaxAngularCorrection)
			limitImpulse = -this.MotorMass * C
		}

		aA -= this.InvIA * limitImpulse
		aB += this.InvIB * limitImpulse
	}

	// Solve point-to-point constraint.
	{
		qA.Set(aA)
		qB.Set(aB)
		rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
		rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))

		C := SubVV(AddVV(cB, rB), AddVV(cA, rA))
		positionError = C.Length()

		mA := this.InvMassA
		mB := this.InvMassB
		iA := this.InvIA
		iB := this.InvIB

		var K Mat22
		K.Ex.X = mA + mB + iA*rA.Y*rA.Y + iB*rB.Y*rB.Y
		K.Ex.Y = -iA*rA.X*rA.Y - iB*rB.X*rB.Y
		K.Ey.X = K.Ex.Y
		K.Ey.Y = mA + mB + iA*rA.X*rA.X + iB*rB.X*rB.X

		impulse := K.Solve(C)
		impulse = impulse.Minus()

		cA.Sub(MulFV(mA, impulse))
		aA -= iA * CrossVV(rA, impulse)

		cB.Add(MulFV(mB, impulse))
		aB += iB * CrossVV(rB, impulse)
	}

	data.positions[this.IndexA].c = cA
	data.positions[this.IndexA].a = aA
	data.positions[this.IndexB].c = cB
	data.positions[this.IndexB].a = aB

	return positionError <= LinearSlop && angularError <= AngularSlop
}

/// Dump to b2Log.
func (this *RevoluteJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	Log("  b2RevoluteJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%d);\n", this.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this.LocalAnchorA.X, this.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this.LocalAnchorB.X, this.LocalAnchorB.Y)
	Log("  jd.referenceAngle = %.15lef;\n", this.ReferenceAngle)
	Log("  jd.enableLimit = bool(%d);\n", this.EnableLimit)
	Log("  jd.lowerAngle = %.15lef;\n", this.LowerAngle)
	Log("  jd.upperAngle = %.15lef;\n", this.UpperAngle)
	Log("  jd.enableMotor = bool(%d);\n", this.EnableMotor)
	Log("  jd.motorSpeed = %.15lef;\n", this.MotorSpeed)
	Log("  jd.maxMotorTorque = %.15lef;\n", this.MaxMotorTorque)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}

const minPulleyLength float64 = 2.0

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
type PulleyJointDef struct {
	JointDef

	/// The first ground anchor in world coordinates. This point never moves.
	GroundAnchorA Vec2

	/// The second ground anchor in world coordinates. This point never moves.
	GroundAnchorB Vec2

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The a reference length for the segment attached to bodyA.
	LengthA float64

	/// The a reference length for the segment attached to bodyB.
	LengthB float64

	/// The pulley ratio, used to simulate a block-and-tackle.
	Ratio float64
}

func NewPulleyJointDef() *PulleyJointDef {
	this := new(PulleyJointDef)
	this.Type = Joint_e_pulleyJoint
	this.GroundAnchorA.Set(-1.0, 1.0)
	this.GroundAnchorB.Set(1.0, 1.0)
	this.LocalAnchorA.Set(-1.0, 0.0)
	this.LocalAnchorB.Set(1.0, 0.0)
	this.LengthA = 0.0
	this.LengthB = 0.0
	this.Ratio = 1.0
	this.CollideConnected = true
	return this
}

/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
func (this *PulleyJointDef) Initialize(bodyA *Body, bodyB *Body, groundAnchorA Vec2,
	groundAnchorB Vec2, anchorA Vec2, anchorB Vec2, ratio float64) {
	this.BodyA = bodyA
	this.BodyB = bodyB
	this.GroundAnchorA = groundAnchorA
	this.GroundAnchorB = groundAnchorB
	this.LocalAnchorA = bodyA.GetLocalPoint(anchorA)
	this.LocalAnchorB = bodyB.GetLocalPoint(anchorB)
	dA := SubVV(anchorA, groundAnchorA)
	this.LengthA = dA.Length()
	dB := SubVV(anchorB, groundAnchorB)
	this.LengthB = dB.Length()
	this.Ratio = ratio
	//b2Assert(ratio > b2_epsilon);
}

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
type PulleyJoint struct {
	Joint

	GroundAnchorA Vec2
	GroundAnchorB Vec2
	LengthA       float64
	LengthB       float64

	// Solver shared
	LocalAnchorA Vec2
	LocalAnchorB Vec2
	Constant     float64
	Ratio        float64
	Impulse      float64

	// Solver temp
	IndexA       int
	IndexB       int
	UA           Vec2
	UB           Vec2
	RA           Vec2
	RB           Vec2
	LocalCenterA Vec2
	LocalCenterB Vec2
	InvMassA     float64
	InvMassB     float64
	InvIA        float64
	InvIB        float64
	Mass         float64
}

func NewPulleyJoint(def *PulleyJointDef) *PulleyJoint {
	this := new(PulleyJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.GroundAnchorA = def.GroundAnchorA
	this.GroundAnchorB = def.GroundAnchorB
	this.LocalAnchorA = def.LocalAnchorA
	this.LocalAnchorB = def.LocalAnchorB

	this.LengthA = def.LengthA
	this.LengthB = def.LengthB

	//b2Assert(def->ratio != 0.0f);
	this.Ratio = def.Ratio

	this.Constant = def.LengthA + this.Ratio*def.LengthB

	this.Impulse = 0.0
	return this
}

func (this *PulleyJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *PulleyJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

func (this *PulleyJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := MulFV(this.Impulse, this.UB)
	return MulFV(inv_dt, P)
}

func (this *PulleyJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

/// Get the first ground anchor.
func (this *PulleyJoint) GetGroundAnchorA() Vec2 {
	return this.GroundAnchorA
}

/// Get the second ground anchor.
func (this *PulleyJoint) GetGroundAnchorB() Vec2 {
	return this.GroundAnchorB
}

/// Get the current length of the segment attached to bodyA.
func (this *PulleyJoint) GetLengthA() float64 {
	p := this.BodyA.GetWorldPoint(this.LocalAnchorA)
	s := this.GroundAnchorA
	d := SubVV(p, s)
	return d.Length()
}

/// Get the current length of the segment attached to bodyB.
func (this *PulleyJoint) GetLengthB() float64 {
	p := this.BodyB.GetWorldPoint(this.LocalAnchorB)
	s := this.GroundAnchorB
	d := SubVV(p, s)
	return d.Length()
}

/// Get the pulley ratio.
func (this *PulleyJoint) GetRatio() float64 {
	return this.Ratio
}

func (this *PulleyJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterA = this.BodyA.sweep.LocalCenter
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassA = this.BodyA.invMass
	this.InvMassB = this.BodyB.invMass
	this.InvIA = this.BodyA.invI
	this.InvIB = this.BodyB.invI

	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qA := NewRot(aA)
	qB := NewRot(aB)

	this.RA = MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	this.RB = MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))

	// Get the pulley axes.
	this.UA = SubVV(AddVV(cA, this.RA), this.GroundAnchorA)
	this.UB = SubVV(AddVV(cB, this.RB), this.GroundAnchorB)

	lengthA := this.UA.Length()
	lengthB := this.UB.Length()

	if lengthA > 10.0*LinearSlop {
		this.UA.Mul(1.0 / lengthA)
	} else {
		this.UA.SetZero()
	}

	if lengthB > 10.0*LinearSlop {
		this.UB.Mul(1.0 / lengthB)
	} else {
		this.UB.SetZero()
	}

	// Compute effective mass.
	ruA := CrossVV(this.RA, this.UA)
	ruB := CrossVV(this.RB, this.UB)

	mA := this.InvMassA + this.InvIA*ruA*ruA
	mB := this.InvMassB + this.InvIB*ruB*ruB

	this.Mass = mA + this.Ratio*this.Ratio*mB

	if this.Mass > 0.0 {
		this.Mass = 1.0 / this.Mass
	}

	if data.step.warmStarting {
		// Scale impulses to support variable time steps.
		this.Impulse *= data.step.dtRatio

		// Warm starting.
		PA := MulFV(-this.Impulse, this.UA)
		PB := MulFV((-this.Ratio * this.Impulse), this.UB)

		vA.Add(MulFV(this.InvMassA, PA))
		wA += this.InvIA * CrossVV(this.RA, PA)
		vB.Add(MulFV(this.InvMassB, PB))
		wB += this.InvIB * CrossVV(this.RB, PB)
	} else {
		this.Impulse = 0.0
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *PulleyJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	vpA := AddVV(vA, CrossFV(wA, this.RA))
	vpB := AddVV(vB, CrossFV(wB, this.RB))

	Cdot := -DotVV(this.UA, vpA) - this.Ratio*DotVV(this.UB, vpB)
	impulse := -this.Mass * Cdot
	this.Impulse += impulse

	PA := MulFV(-impulse, this.UA)
	PB := MulFV(-this.Ratio*impulse, this.UB)
	vA.Add(MulFV(this.InvMassA, PA))
	wA += this.InvIA * CrossVV(this.RA, PA)
	vB.Add(MulFV(this.InvMassB, PB))
	wB += this.InvIB * CrossVV(this.RB, PB)

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *PulleyJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a

	qA := NewRot(aA)
	qB := NewRot(aB)

	rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))

	// Get the pulley axes.
	uA := SubVV(AddVV(cA, rA), this.GroundAnchorA)
	uB := SubVV(AddVV(cB, rB), this.GroundAnchorB)

	lengthA := uA.Length()
	lengthB := uB.Length()

	if lengthA > 10.0*LinearSlop {
		uA.Mul(1.0 / lengthA)
	} else {
		uA.SetZero()
	}

	if lengthB > 10.0*LinearSlop {
		uB.Mul(1.0 / lengthB)
	} else {
		uB.SetZero()
	}

	// Compute effective mass.
	ruA := CrossVV(rA, uA)
	ruB := CrossVV(rB, uB)

	mA := this.InvMassA + this.InvIA*ruA*ruA
	mB := this.InvMassB + this.InvIB*ruB*ruB

	mass := mA + this.Ratio*this.Ratio*mB

	if mass > 0.0 {
		mass = 1.0 / mass
	}

	C := this.Constant - lengthA - this.Ratio*lengthB
	linearError := AbsF(C)

	impulse := -mass * C

	PA := MulFV(-impulse, uA)
	PB := MulFV(-this.Ratio*impulse, uB)

	cA.Add(MulFV(this.InvMassA, PA))
	aA += this.InvIA * CrossVV(rA, PA)
	cB.Add(MulFV(this.InvMassB, PB))
	aB += this.InvIB * CrossVV(rB, PB)

	data.positions[this.IndexA].c = cA
	data.positions[this.IndexA].a = aA
	data.positions[this.IndexB].c = cB
	data.positions[this.IndexB].a = aB

	return linearError < LinearSlop
}

/// Dump joint to dmLog
func (this *PulleyJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	Log("  b2PulleyJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%d);\n", this.CollideConnected)
	Log("  jd.groundAnchorA.Set(%.15lef, %.15lef);\n", this.GroundAnchorA.X, this.GroundAnchorA.Y)
	Log("  jd.groundAnchorB.Set(%.15lef, %.15lef);\n", this.GroundAnchorB.X, this.GroundAnchorB.Y)
	Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this.LocalAnchorA.X, this.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this.LocalAnchorB.X, this.LocalAnchorB.Y)
	Log("  jd.lengthA = %.15lef;\n", this.LengthA)
	Log("  jd.lengthB = %.15lef;\n", this.LengthB)
	Log("  jd.ratio = %.15lef;\n", this.Ratio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}

/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
type GearJointDef struct {
	JointDef

	/// The first revolute/prismatic joint attached to the gear joint.
	Joint1 IJoint

	/// The second revolute/prismatic joint attached to the gear joint.
	Joint2 IJoint

	/// The gear ratio.
	/// @see b2GearJoint for explanation.
	Ratio float64
}

func NewGearJointDef() *GearJointDef {
	this := new(GearJointDef)
	this.Type = Joint_e_gearJoint
	this.Joint1 = nil
	this.Joint2 = nil
	this.Ratio = 1.0
	return this
}

/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning You have to manually destroy the gear joint if joint1 or joint2
/// is destroyed.
type GearJoint struct {
	Joint

	Joint1 IJoint
	Joint2 IJoint

	TypeA JointType
	TypeB JointType

	// Body A is connected to body C
	// Body B is connected to body D
	BodyC *Body
	BodyD *Body

	// Solver shared
	LocalAnchorA Vec2
	LocalAnchorB Vec2
	LocalAnchorC Vec2
	LocalAnchorD Vec2

	LocalAxisC Vec2
	LocalAxisD Vec2

	ReferenceAngleA float64
	ReferenceAngleB float64

	Constant float64
	Ratio    float64

	Impulse float64

	// Solver temp
	IndexA, IndexB, IndexC, IndexD int
	LcA, LcB, LcC, LcD             Vec2
	MA, MB, MC, MD                 float64
	IA, IB, IC, ID                 float64
	JvAC, JvBD                     Vec2
	JwA, JwB, JwC, JwD             float64
	Mass                           float64
}

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

func NewGearJoint(def *GearJointDef) *GearJoint {
	this := new(GearJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.Joint1 = def.Joint1
	this.Joint2 = def.Joint2

	this.TypeA = this.Joint1.GetType()
	this.TypeB = this.Joint2.GetType()

	//b2Assert(m_typeA == e_revoluteJoint || m_typeA == e_prismaticJoint);
	//b2Assert(m_typeB == e_revoluteJoint || m_typeB == e_prismaticJoint);

	var coordinateA, coordinateB float64

	// TODO_ERIN there might be some problem with the joint edges in b2Joint.

	this.BodyC = this.Joint1.GetBodyA()
	this.BodyA = this.Joint1.GetBodyB()

	// Get geometry of joint1
	xfA := this.BodyA.xf
	aA := this.BodyA.sweep.A
	xfC := this.BodyC.xf
	aC := this.BodyC.sweep.A

	if this.TypeA == Joint_e_revoluteJoint {
		revolute := def.Joint1.(*RevoluteJoint)
		this.LocalAnchorC = revolute.LocalAnchorA
		this.LocalAnchorA = revolute.LocalAnchorB
		this.ReferenceAngleA = revolute.ReferenceAngle
		this.LocalAxisC.SetZero()

		coordinateA = aA - aC - this.ReferenceAngleA
	} else {
		prismatic := def.Joint1.(*PrismaticJoint)
		this.LocalAnchorC = prismatic.LocalAnchorA
		this.LocalAnchorA = prismatic.LocalAnchorB
		this.ReferenceAngleA = prismatic.ReferenceAngle
		this.LocalAxisC = prismatic.LocalXAxisA

		pC := this.LocalAnchorC
		pA := MulTRV(xfC.Q, AddVV(MulRV(xfA.Q, this.LocalAnchorA), SubVV(xfA.P, xfC.P)))
		coordinateA = DotVV(SubVV(pA, pC), this.LocalAxisC)
	}

	this.BodyD = this.Joint2.GetBodyA()
	this.BodyB = this.Joint2.GetBodyB()

	// Get geometry of joint2
	xfB := this.BodyB.xf
	aB := this.BodyB.sweep.A
	xfD := this.BodyD.xf
	aD := this.BodyD.sweep.A

	if this.TypeB == Joint_e_revoluteJoint {
		revolute := def.Joint2.(*RevoluteJoint)
		this.LocalAnchorD = revolute.LocalAnchorA
		this.LocalAnchorB = revolute.LocalAnchorB
		this.ReferenceAngleB = revolute.ReferenceAngle
		this.LocalAxisD.SetZero()

		coordinateB = aB - aD - this.ReferenceAngleB
	} else {
		prismatic := def.Joint2.(*PrismaticJoint)
		this.LocalAnchorD = prismatic.LocalAnchorA
		this.LocalAnchorB = prismatic.LocalAnchorB
		this.ReferenceAngleB = prismatic.ReferenceAngle
		this.LocalAxisD = prismatic.LocalXAxisA

		pD := this.LocalAnchorD
		pB := MulTRV(xfD.Q, AddVV(MulRV(xfB.Q, this.LocalAnchorB), AddVV(xfB.P, xfD.P)))
		coordinateB = DotVV(SubVV(pB, pD), this.LocalAxisD)
	}

	this.Ratio = def.Ratio

	this.Constant = coordinateA + this.Ratio*coordinateB

	this.Impulse = 0.0
	return this
}

func (this *GearJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *GearJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

func (this *GearJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := MulFV(this.Impulse, this.JvAC)
	return MulFV(inv_dt, P)
}

func (this *GearJoint) GetReactionTorque(inv_dt float64) float64 {
	L := this.Impulse * this.JwA
	return inv_dt * L
}

/// Get the first joint.
func (this *GearJoint) GetJoint1() IJoint {
	return this.Joint1
}

/// Get the second joint.
func (this *GearJoint) GetJoint2() IJoint {
	return this.Joint2
}

/// Set/Get the gear ratio.
func (this *GearJoint) SetRatio(ratio float64) {
	this.Ratio = ratio
}

func (this *GearJoint) GetRatio() float64 {
	return this.Ratio
}

func (this *GearJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.IndexC = this.BodyC.islandIndex
	this.IndexD = this.BodyD.islandIndex
	this.LcA = this.BodyA.sweep.LocalCenter
	this.LcB = this.BodyB.sweep.LocalCenter
	this.LcC = this.BodyC.sweep.LocalCenter
	this.LcD = this.BodyD.sweep.LocalCenter
	this.MA = this.BodyA.invMass
	this.MB = this.BodyB.invMass
	this.MC = this.BodyC.invMass
	this.MD = this.BodyD.invMass
	this.IA = this.BodyA.invI
	this.IB = this.BodyB.invI
	this.IC = this.BodyC.invI
	this.ID = this.BodyD.invI

	//cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	//cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	//cC := data.positions[this.IndexC].c
	aC := data.positions[this.IndexC].a
	vC := data.velocities[this.IndexC].v
	wC := data.velocities[this.IndexC].w

	//cD := data.positions[this.IndexD].c
	aD := data.positions[this.IndexD].a
	vD := data.velocities[this.IndexD].v
	wD := data.velocities[this.IndexD].w

	qA, qB, qC, qD := NewRot(aA), NewRot(aB), NewRot(aC), NewRot(aD)

	this.Mass = 0.0

	if this.TypeA == Joint_e_revoluteJoint {
		this.JvAC.SetZero()
		this.JwA = 1.0
		this.JwC = 1.0
		this.Mass += this.IA + this.IC
	} else {
		u := MulRV(qC, this.LocalAxisC)
		rC := MulRV(qC, SubVV(this.LocalAnchorC, this.LcC))
		rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LcA))
		this.JvAC = u
		this.JwC = CrossVV(rC, u)
		this.JwA = CrossVV(rA, u)
		this.Mass += this.MC + this.MA + this.IC*this.JwC*this.JwC + this.IA*this.JwA*this.JwA
	}

	if this.TypeB == Joint_e_revoluteJoint {
		this.JvBD.SetZero()
		this.JwB = this.Ratio
		this.JwD = this.Ratio
		this.Mass += this.Ratio * this.Ratio * (this.IB + this.ID)
	} else {
		u := MulRV(qD, this.LocalAxisD)
		rD := MulRV(qD, SubVV(this.LocalAnchorD, this.LcD))
		rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LcB))
		this.JvBD = MulFV(this.Ratio, u)
		this.JwD = this.Ratio * CrossVV(rD, u)
		this.JwB = this.Ratio * CrossVV(rB, u)
		this.Mass += this.Ratio*this.Ratio*(this.MD+this.MB) + this.ID*this.JwD*this.JwD + this.IB*this.JwB*this.JwB
	}

	// Compute effective mass.
	this.Mass = 0.0
	if this.Mass > 0.0 {
		this.Mass = 1.0 / this.Mass
	}

	if data.step.warmStarting {
		vA.Add(MulFV(this.MA*this.Impulse, this.JvAC))
		wA += this.IA * this.Impulse * this.JwA
		vB.Add(MulFV(this.MB*this.Impulse, this.JvBD))
		wB += this.IB * this.Impulse * this.JwB
		vC.Sub(MulFV(this.MC*this.Impulse, this.JvAC))
		wC -= this.IC * this.Impulse * this.JwC
		vD.Sub(MulFV(this.MD*this.Impulse, this.JvBD))
		wD -= this.ID * this.Impulse * this.JwD
	} else {
		this.Impulse = 0.0
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
	data.velocities[this.IndexC].v = vC
	data.velocities[this.IndexC].w = wC
	data.velocities[this.IndexD].v = vD
	data.velocities[this.IndexD].w = wD
}

func (this *GearJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w
	vC := data.velocities[this.IndexC].v
	wC := data.velocities[this.IndexC].w
	vD := data.velocities[this.IndexD].v
	wD := data.velocities[this.IndexD].w

	Cdot := DotVV(this.JvAC, SubVV(vA, vC)) + DotVV(this.JvBD, SubVV(vB, vD))
	Cdot += (this.JwA*wA - this.JwC*wC) + (this.JwB*wB - this.JwD*wD)

	impulse := -this.Mass * Cdot
	this.Impulse += impulse

	vA.Add(MulFV(this.MA*impulse, this.JvAC))
	wA += this.IA * impulse * this.JwA
	vB.Add(MulFV(this.MB*impulse, this.JvBD))
	wB += this.IB * impulse * this.JwB
	vC.Sub(MulFV(this.MC*impulse, this.JvAC))
	wC -= this.IC * impulse * this.JwC
	vD.Sub(MulFV(this.MD*impulse, this.JvBD))
	wD -= this.ID * impulse * this.JwD

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
	data.velocities[this.IndexC].v = vC
	data.velocities[this.IndexC].w = wC
	data.velocities[this.IndexD].v = vD
	data.velocities[this.IndexD].w = wD
}

func (this *GearJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	cC := data.positions[this.IndexC].c
	aC := data.positions[this.IndexC].a
	cD := data.positions[this.IndexD].c
	aD := data.positions[this.IndexD].a

	qA, qB, qC, qD := NewRot(aA), NewRot(aB), NewRot(aC), NewRot(aD)

	linearError := 0.0

	var coordinateA, coordinateB float64

	var JvAC, JvBD Vec2
	var JwA, JwB, JwC, JwD float64
	mass := 0.0

	if this.TypeA == Joint_e_revoluteJoint {
		JvAC.SetZero()
		JwA = 1.0
		JwC = 1.0
		mass += this.IA + this.IC

		coordinateA = aA - aC - this.ReferenceAngleA
	} else {
		u := MulRV(qC, this.LocalAxisC)
		rC := MulRV(qC, SubVV(this.LocalAnchorC, this.LcC))
		rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LcA))
		JvAC = u
		JwC = CrossVV(rC, u)
		JwA = CrossVV(rA, u)
		mass += this.MC + this.MA + this.IC*JwC*JwC + this.IA*JwA*JwA

		pC := SubVV(this.LocalAnchorC, this.LcC)
		pA := MulTRV(qC, AddVV(rA, SubVV(cA, cC)))
		coordinateA = DotVV(SubVV(pA, pC), this.LocalAxisC)
	}

	if this.TypeB == Joint_e_revoluteJoint {
		JvBD.SetZero()
		JwB = this.Ratio
		JwD = this.Ratio
		mass += this.Ratio * this.Ratio * (this.IB + this.ID)

		coordinateB = aB - aD - this.ReferenceAngleB
	} else {
		u := MulRV(qD, this.LocalAxisD)
		rD := MulRV(qD, SubVV(this.LocalAnchorD, this.LcD))
		rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LcB))
		JvBD = MulFV(this.Ratio, u)
		JwD = this.Ratio * CrossVV(rD, u)
		JwB = this.Ratio * CrossVV(rB, u)
		mass += this.Ratio*this.Ratio*(this.MD+this.MB) + this.ID*JwD*JwD + this.IB*JwB*JwB

		pD := SubVV(this.LocalAnchorD, this.LcD)
		pB := MulTRV(qD, AddVV(rB, SubVV(cB, cD)))
		coordinateB = DotVV(SubVV(pB, pD), this.LocalAxisD)
	}

	C := (coordinateA + this.Ratio*coordinateB) - this.Constant

	impulse := 0.0
	if mass > 0.0 {
		impulse = -C / mass
	}

	cA.Add(MulFV(this.MA*impulse, JvAC))
	aA += this.IA * impulse * JwA
	cB.Add(MulFV(this.MB*impulse, JvBD))
	aB += this.IB * impulse * JwB
	cC.Sub(MulFV(this.MC*impulse, JvAC))
	aC -= this.IC * impulse * JwC
	cD.Sub(MulFV(this.MD*impulse, JvBD))
	aD -= this.ID * impulse * JwD

	data.positions[this.IndexA].c = cA
	data.positions[this.IndexA].a = aA
	data.positions[this.IndexB].c = cB
	data.positions[this.IndexB].a = aB
	data.positions[this.IndexC].c = cC
	data.positions[this.IndexC].a = aC
	data.positions[this.IndexD].c = cD
	data.positions[this.IndexD].a = aD

	// TODO_ERIN not implemented
	return linearError < LinearSlop
}

/// Dump joint to dmLog
func (this *GearJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	index1 := this.Joint1.GetIndex()
	index2 := this.Joint2.GetIndex()

	Log("  b2GearJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%d);\n", this.CollideConnected)
	Log("  jd.joint1 = joints[%d];\n", index1)
	Log("  jd.joint2 = joints[%d];\n", index2)
	Log("  jd.ratio = %.15lef;\n", this.Ratio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
type WheelJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The local translation axis in bodyA.
	LocalAxisA Vec2

	/// Enable/disable the joint motor.
	EnableMotor bool

	/// The maximum motor torque, usually in N-m.
	MaxMotorTorque float64

	/// The desired motor speed in radians per second.
	MotorSpeed float64

	/// Suspension frequency, zero indicates no suspension
	FrequencyHz float64

	/// Suspension damping ratio, one indicates critical damping
	DampingRatio float64
}

func NewWheelJointDef() *WheelJointDef {
	this := new(WheelJointDef)
	this.Type = Joint_e_wheelJoint
	this.LocalAnchorA.SetZero()
	this.LocalAnchorB.SetZero()
	this.LocalAxisA.Set(1.0, 0.0)
	this.EnableMotor = false
	this.MaxMotorTorque = 0.0
	this.MotorSpeed = 0.0
	this.FrequencyHz = 2.0
	this.DampingRatio = 0.7
	return this
}

/// Initialize the bodies, anchors, axis, and reference angle using the world
/// anchor and world axis.
func (this *WheelJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2, axis Vec2) {
	this.BodyA = bodyA
	this.BodyB = bodyB
	this.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	this.LocalAnchorB = bodyB.GetLocalPoint(anchor)
	this.LocalAxisA = bodyA.GetLocalVector(axis)
}

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. You can use a
/// joint limit to restrict the range of motion and a joint motor to drive
/// the rotation or to model rotational friction.
/// This joint is designed for vehicle suspensions.
type WheelJoint struct {
	Joint

	FrequencyHz  float64
	DampingRatio float64

	// Solver shared
	LocalAnchorA Vec2
	LocalAnchorB Vec2
	LocalXAxisA  Vec2
	LocalYAxisA  Vec2

	Impulse       float64
	MotorImpulse  float64
	SpringImpulse float64

	MaxMotorTorque float64
	MotorSpeed     float64
	EnableMotor    bool

	// Solver temp
	IndexA       int
	IndexB       int
	LocalCenterA Vec2
	LocalCenterB Vec2
	InvMassA     float64
	InvMassB     float64
	InvIA        float64
	InvIB        float64

	Ax, Ay   Vec2
	SAx, SBx float64
	SAy, SBy float64

	Mass       float64
	MotorMass  float64
	SpringMass float64

	Bias  float64
	Gamma float64
}

func NewWheelJoint(def *WheelJointDef) *WheelJoint {
	this := new(WheelJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.LocalAnchorA = def.LocalAnchorA
	this.LocalAnchorB = def.LocalAnchorB
	this.LocalXAxisA = def.LocalAxisA
	this.LocalYAxisA = CrossFV(1.0, this.LocalXAxisA)

	this.Mass = 0.0
	this.Impulse = 0.0
	this.MotorMass = 0.0
	this.MotorImpulse = 0.0
	this.SpringMass = 0.0
	this.SpringImpulse = 0.0

	this.MaxMotorTorque = def.MaxMotorTorque
	this.MotorSpeed = def.MotorSpeed
	this.EnableMotor = def.EnableMotor

	this.FrequencyHz = def.FrequencyHz
	this.DampingRatio = def.DampingRatio

	this.Bias = 0.0
	this.Gamma = 0.0

	this.Ax.SetZero()
	this.Ay.SetZero()
	return this
}

func (this *WheelJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *WheelJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

func (this *WheelJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt, AddVV(MulFV(this.Impulse, this.Ay), MulFV(this.SpringImpulse, this.Ax)))
}

func (this *WheelJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * this.MotorImpulse
}

/// The local anchor point relative to bodyA's origin.
func (this *WheelJoint) GetLocalAnchorA() Vec2 {
	return this.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (this *WheelJoint) GetLocalAnchorB() Vec2 {
	return this.LocalAnchorB
}

/// The local joint axis relative to bodyA.
func (this *WheelJoint) GetLocalAxisA() Vec2 {
	return this.LocalXAxisA
}

/// Get the current joint translation, usually in meters.
func (this *WheelJoint) GetJointTranslation() float64 {
	bA := this.BodyA
	bB := this.BodyB

	pA := bA.GetWorldPoint(this.LocalAnchorA)
	pB := bB.GetWorldPoint(this.LocalAnchorB)
	d := SubVV(pB, pA)
	axis := bA.GetWorldVector(this.LocalXAxisA)

	translation := DotVV(d, axis)
	return translation
}

/// Get the current joint translation speed, usually in meters per second.
func (this *WheelJoint) GetJointSpeed() float64 {
	wA := this.BodyA.angularVelocity
	wB := this.BodyB.angularVelocity
	return wB - wA
}

/// Is the joint motor enabled?
func (this *WheelJoint) IsMotorEnabled() bool {
	return this.EnableMotor
}

/// Enable/disable the joint motor.
func (this *WheelJoint) SetEnableMotor(flag bool) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.EnableMotor = flag
}

/// Set the motor speed, usually in radians per second.
func (this *WheelJoint) SetMotorSpeed(speed float64) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.MotorSpeed = speed
}

/// Get the motor speed, usually in radians per second.
func (this *WheelJoint) GetMotorSpeed() float64 {
	return this.MotorSpeed
}

/// Set/Get the maximum motor force, usually in N-m.
func (this *WheelJoint) SetMaxMotorTorque(torque float64) {
	this.BodyA.SetAwake(true)
	this.BodyB.SetAwake(true)
	this.MaxMotorTorque = torque
}

func (this *WheelJoint) GetMaxMotorTorque() float64 {
	return this.MaxMotorTorque
}

/// Get the current motor torque given the inverse time step, usually in N-m.
func (this *WheelJoint) GetMotorTorque(inv_dt float64) float64 {
	return inv_dt * this.MotorImpulse
}

/// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
func (this *WheelJoint) SetSpringFrequencyHz(hz float64) {
	this.FrequencyHz = hz
}

func (this *WheelJoint) GetSpringFrequencyHz() float64 {
	return this.FrequencyHz
}

/// Set/Get the spring damping ratio
func (this *WheelJoint) SetSpringDampingRatio(ratio float64) {
	this.DampingRatio = ratio
}

func (this *WheelJoint) GetSpringDampingRatio() float64 {
	return this.DampingRatio
}

func (this *WheelJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterA = this.BodyA.sweep.LocalCenter
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassA = this.BodyA.invMass
	this.InvMassB = this.BodyB.invMass
	this.InvIA = this.BodyA.invI
	this.InvIB = this.BodyB.invI

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qA, qB := NewRot(aA), NewRot(aB)

	// Compute the effective masses.
	rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))
	d := SubVV(AddVV(cB, rB), AddVV(cA, rA))

	// Point to line constraint
	{
		this.Ay = MulRV(qA, this.LocalYAxisA)
		this.SAy = CrossVV(AddVV(d, rA), this.Ay)
		this.SBy = CrossVV(rB, this.Ay)

		this.Mass = mA + mB + iA*this.SAy*this.SAy + iB*this.SBy*this.SBy

		if this.Mass > 0.0 {
			this.Mass = 1.0 / this.Mass
		}
	}

	// Spring constraint
	this.SpringMass = 0.0
	this.Bias = 0.0
	this.Gamma = 0.0
	if this.FrequencyHz > 0.0 {
		this.Ax = MulRV(qA, this.LocalXAxisA)
		this.SAx = CrossVV(AddVV(d, rA), this.Ax)
		this.SBx = CrossVV(rB, this.Ax)

		invMass := mA + mB + iA*this.SAx*this.SAx + iB*this.SBx*this.SBx

		if invMass > 0.0 {
			this.SpringMass = 1.0 / invMass

			C := DotVV(d, this.Ax)

			// Frequency
			omega := 2.0 * Pi * this.FrequencyHz

			// Damping coefficient
			d := 2.0 * this.SpringMass * this.DampingRatio * omega

			// Spring stiffness
			k := this.SpringMass * omega * omega

			// magic formulas
			h := data.step.dt
			this.Gamma = h * (d + h*k)
			if this.Gamma > 0.0 {
				this.Gamma = 1.0 / this.Gamma
			}

			this.Bias = C * h * k * this.Gamma

			this.SpringMass = invMass + this.Gamma
			if this.SpringMass > 0.0 {
				this.SpringMass = 1.0 / this.SpringMass
			}
		}
	} else {
		this.SpringImpulse = 0.0
	}

	// Rotational motor
	if this.EnableMotor {
		this.MotorMass = iA + iB
		if this.MotorMass > 0.0 {
			this.MotorMass = 1.0 / this.MotorMass
		}
	} else {
		this.MotorMass = 0.0
		this.MotorImpulse = 0.0
	}

	if data.step.warmStarting {
		// Account for variable time step.
		this.Impulse *= data.step.dtRatio
		this.SpringImpulse *= data.step.dtRatio
		this.MotorImpulse *= data.step.dtRatio

		P := AddVV(MulFV(this.Impulse, this.Ay), MulFV(this.SpringImpulse, this.Ax))
		LA := this.Impulse*this.SAy + this.SpringImpulse*this.SAx + this.MotorImpulse
		LB := this.Impulse*this.SBy + this.SpringImpulse*this.SBx + this.MotorImpulse

		vA.Sub(MulFV(this.InvMassA, P))
		wA -= this.InvIA * LA

		vB.Add(MulFV(this.InvMassB, P))
		wB += this.InvIB * LB
	} else {
		this.Impulse = 0.0
		this.SpringImpulse = 0.0
		this.MotorImpulse = 0.0
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *WheelJoint) SolveVelocityConstraints(data *solverData) {
	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	// Solve spring constraint
	{
		Cdot := DotVV(this.Ax, SubVV(vB, vA)) + this.SBx*wB - this.SAx*wA
		impulse := -this.SpringMass * (Cdot + this.Bias + this.Gamma*this.SpringImpulse)
		this.SpringImpulse += impulse

		P := MulFV(impulse, this.Ax)
		LA := impulse * this.SAx
		LB := impulse * this.SBx

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	}

	// Solve rotational motor constraint
	{
		Cdot := wB - wA - this.MotorSpeed
		impulse := -this.MotorMass * Cdot

		oldImpulse := this.MotorImpulse
		maxImpulse := data.step.dt * this.MaxMotorTorque
		this.MotorImpulse = ClampF(this.MotorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = this.MotorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve point to line constraint
	{
		Cdot := DotVV(this.Ay, SubVV(vB, vA)) + this.SBy*wB - this.SAy*wA
		impulse := -this.Mass * Cdot
		this.Impulse += impulse

		P := MulFV(impulse, this.Ay)
		LA := impulse * this.SAy
		LB := impulse * this.SBy

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *WheelJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a

	qA, qB := NewRot(aA), NewRot(aB)

	rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))
	d := AddVV(SubVV(cB, cA), SubVV(rB, rA))

	ay := MulRV(qA, this.LocalYAxisA)

	sAy := CrossVV(AddVV(d, rA), ay)
	sBy := CrossVV(rB, ay)

	C := DotVV(d, ay)

	k := this.InvMassA + this.InvMassB + this.InvIA*this.SAy*this.SAy + this.InvIB*this.SBy*this.SBy

	var impulse float64
	if k != 0.0 {
		impulse = -C / k
	} else {
		impulse = 0.0
	}

	P := MulFV(impulse, ay)
	LA := impulse * sAy
	LB := impulse * sBy

	cA.Sub(MulFV(this.InvMassA, P))
	aA -= this.InvIA * LA
	cB.Add(MulFV(this.InvMassB, P))
	aB += this.InvIB * LB

	data.positions[this.IndexA].c = cA
	data.positions[this.IndexA].a = aA
	data.positions[this.IndexB].c = cB
	data.positions[this.IndexB].a = aB

	return AbsF(C) <= LinearSlop
}

/// Dump to b2Log
func (this *WheelJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	Log("  b2WheelJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%d);\n", this.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this.LocalAnchorA.X, this.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this.LocalAnchorB.X, this.LocalAnchorB.Y)
	Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", this.LocalXAxisA.X, this.LocalXAxisA.Y)
	Log("  jd.enableMotor = bool(%d);\n", this.EnableMotor)
	Log("  jd.motorSpeed = %.15lef;\n", this.MotorSpeed)
	Log("  jd.maxMotorTorque = %.15lef;\n", this.MaxMotorTorque)
	Log("  jd.frequencyHz = %.15lef;\n", this.FrequencyHz)
	Log("  jd.dampingRatio = %.15lef;\n", this.DampingRatio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
type WeldJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	ReferenceAngle float64

	/// The mass-spring-damper frequency in Hertz. Rotation only.
	/// Disable softness with a value of 0.
	FrequencyHz float64

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	DampingRatio float64
}

func NewWeldJointDef() *WeldJointDef {
	this := new(WeldJointDef)
	this.Type = Joint_e_weldJoint
	this.LocalAnchorA.Set(0.0, 0.0)
	this.LocalAnchorB.Set(0.0, 0.0)
	this.ReferenceAngle = 0.0
	this.FrequencyHz = 0.0
	this.DampingRatio = 0.0
	return this
}

/// Initialize the bodies, anchors, and reference angle using a world
/// anchor point.
func (this *WeldJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2) {
	this.BodyA = bodyA
	this.BodyB = bodyB
	this.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	this.LocalAnchorB = bodyB.GetLocalPoint(anchor)
	this.ReferenceAngle = bodyB.GetAngle() - bodyA.GetAngle()
}

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
type WeldJoint struct {
	Joint

	FrequencyHz  float64
	DampingRatio float64
	Bias         float64

	// Solver shared
	LocalAnchorA   Vec2
	LocalAnchorB   Vec2
	ReferenceAngle float64
	Gamma          float64
	Impulse        Vec3

	// Solver temp
	IndexA       int
	IndexB       int
	RA           Vec2
	RB           Vec2
	LocalCenterA Vec2
	LocalCenterB Vec2
	InvMassA     float64
	InvMassB     float64
	InvIA        float64
	InvIB        float64
	Mass         Mat33
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

func NewWeldJoint(def *WeldJointDef) *WeldJoint {
	this := new(WeldJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.LocalAnchorA = def.LocalAnchorA
	this.LocalAnchorB = def.LocalAnchorB
	this.ReferenceAngle = def.ReferenceAngle
	this.FrequencyHz = def.FrequencyHz
	this.DampingRatio = def.DampingRatio

	this.Impulse.SetZero()
	return this
}

func (this *WeldJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *WeldJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

func (this *WeldJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := Vec2{this.Impulse.X, this.Impulse.Y}
	return MulFV(inv_dt, P)
}

func (this *WeldJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * this.Impulse.Z
}

/// The local anchor point relative to bodyA's origin.
func (this *WeldJoint) GetLocalAnchorA() Vec2 {
	return this.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (this *WeldJoint) GetLocalAnchorB() Vec2 {
	return this.LocalAnchorB
}

/// Get the reference angle.
func (this *WeldJoint) GetReferenceAngle() float64 {
	return this.ReferenceAngle
}

/// Set/get frequency in Hz.
func (this *WeldJoint) SetFrequency(hz float64) {
	this.FrequencyHz = hz
}

func (this *WeldJoint) GetFrequency() float64 {
	return this.FrequencyHz
}

/// Set/get damping ratio.
func (this *WeldJoint) SetDampingRatio(ratio float64) {
	this.DampingRatio = ratio
}

func (this *WeldJoint) GetDampingRatio() float64 {
	return this.DampingRatio
}

func (this *WeldJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterA = this.BodyA.sweep.LocalCenter
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassA = this.BodyA.invMass
	this.InvMassB = this.BodyB.invMass
	this.InvIA = this.BodyA.invI
	this.InvIB = this.BodyB.invI

	//cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	//cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qA, qB := NewRot(aA), NewRot(aB)

	this.RA = MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	this.RB = MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	var K Mat33
	K.Ex.X = mA + mB + this.RA.Y*this.RA.Y*iA + this.RB.Y*this.RB.Y*iB
	K.Ex.X = mA + mB + this.RA.Y*this.RA.Y*iA + this.RB.Y*this.RB.Y*iB
	K.Ey.X = -this.RA.Y*this.RA.X*iA - this.RB.Y*this.RB.X*iB
	K.Ez.X = -this.RA.Y*iA - this.RB.Y*iB
	K.Ex.Y = K.Ey.X
	K.Ey.Y = mA + mB + this.RA.X*this.RA.X*iA + this.RB.X*this.RB.X*iB
	K.Ez.Y = this.RA.X*iA + this.RB.X*iB
	K.Ex.Z = K.Ez.X
	K.Ey.Z = K.Ez.Y
	K.Ez.Z = iA + iB

	if this.FrequencyHz > 0.0 {
		K.GetInverse22(&this.Mass)

		invM := iA + iB
		m := 0.0
		if invM > 0.0 {
			m = 1.0 / invM
		}

		C := aB - aA - this.ReferenceAngle

		// Frequency
		omega := 2.0 * Pi * this.FrequencyHz

		// Damping coefficient
		d := 2.0 * m * this.DampingRatio * omega

		// Spring stiffness
		k := m * omega * omega

		// magic formulas
		h := data.step.dt
		this.Gamma = h * (d + h*k)
		this.Gamma = 0.0
		if this.Gamma != 0.0 {
			this.Gamma = 1.0 / this.Gamma
		}
		this.Bias = C * h * k * this.Gamma

		invM += this.Gamma
		this.Mass.Ez.Z = 0.0
		if invM != 0.0 {
			invM = 1.0 / invM
		}
	} else {
		K.GetSymInverse33(&this.Mass)
		this.Gamma = 0.0
		this.Bias = 0.0
	}

	if data.step.warmStarting {
		// Scale impulses to support a variable time step.
		this.Impulse.Mul(data.step.dtRatio)

		P := Vec2{this.Impulse.X, this.Impulse.Y}

		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(this.RA, P) + this.Impulse.Z)

		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(this.RB, P) + this.Impulse.Z)
	} else {
		this.Impulse.SetZero()
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *WeldJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	if this.FrequencyHz > 0.0 {
		Cdot2 := wB - wA

		impulse2 := -this.Mass.Ez.Z * (Cdot2 + this.Bias + this.Gamma*this.Impulse.Z)
		this.Impulse.Z += impulse2

		wA -= iA * impulse2
		wB += iB * impulse2

		Cdot1 := SubVV(AddVV(vB, CrossFV(wB, this.RB)), AddVV(vA, CrossFV(wA, this.RA)))

		impulse1 := MulM3V(this.Mass, Cdot1)
		impulse1 = impulse1.Minus()
		this.Impulse.X += impulse1.X
		this.Impulse.Y += impulse1.Y

		P := impulse1

		vA.Sub(MulFV(mA, P))
		wA -= iA * CrossVV(this.RA, P)

		vB.Add(MulFV(mB, P))
		wB += iB * CrossVV(this.RB, P)
	} else {
		Cdot1 := SubVV(AddVV(vB, CrossFV(wB, this.RB)), AddVV(vA, CrossFV(wA, this.RA)))
		Cdot2 := wB - wA
		Cdot := Vec3{Cdot1.X, Cdot1.Y, Cdot2}

		impulse := MulM3V3(this.Mass, Cdot)
		impulse = impulse.Minus()
		this.Impulse.Add(impulse)

		P := Vec2{impulse.X, impulse.Y}

		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(this.RA, P) + impulse.Z)

		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(this.RB, P) + impulse.Z)
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *WeldJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a

	qA, qB := NewRot(aA), NewRot(aB)

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))

	var positionError, angularError float64

	var K Mat33
	K.Ex.X = mA + mB + rA.Y*rA.Y*iA + rB.Y*rB.Y*iB
	K.Ey.X = -rA.Y*rA.X*iA - rB.Y*rB.X*iB
	K.Ez.X = -rA.Y*iA - rB.Y*iB
	K.Ex.Y = K.Ey.X
	K.Ey.Y = mA + mB + rA.X*rA.X*iA + rB.X*rB.X*iB
	K.Ez.Y = rA.X*iA + rB.X*iB
	K.Ex.Y = K.Ez.X
	K.Ey.Y = K.Ez.Y
	K.Ez.Y = iA + iB

	if this.FrequencyHz > 0.0 {
		C1 := SubVV(AddVV(cB, rB), AddVV(cA, rA))

		positionError = C1.Length()
		angularError = 0.0

		P := K.Solve22(C1)
		P = P.Minus()

		cA.Sub(MulFV(mA, P))
		aA -= iA * CrossVV(rA, P)

		cB.Add(MulFV(mB, P))
		aB += iB * CrossVV(rB, P)
	} else {
		C1 := SubVV(AddVV(cB, rB), AddVV(cA, rA))
		C2 := aB - aA - this.ReferenceAngle

		positionError = C1.Length()
		angularError = AbsF(C2)

		C := Vec3{C1.X, C1.Y, C2}

		impulse := K.Solve33(C)
		impulse = impulse.Minus()
		P := Vec2{impulse.X, impulse.Y}

		cA.Sub(MulFV(mA, P))
		aA -= iA * (CrossVV(rA, P) + impulse.Z)

		cB.Add(MulFV(mB, P))
		aB += iB * (CrossVV(rB, P) + impulse.Z)
	}

	data.positions[this.IndexA].c = cA
	data.positions[this.IndexA].a = aA
	data.positions[this.IndexB].c = cB
	data.positions[this.IndexB].a = aB

	return positionError <= LinearSlop && angularError <= AngularSlop
}

/// Dump to b2Log
func (this *WeldJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	Log("  b2WeldJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%d);\n", this.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this.LocalAnchorA.X, this.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this.LocalAnchorB.X, this.LocalAnchorB.Y)
	Log("  jd.referenceAngle = %.15lef;\n", this.ReferenceAngle)
	Log("  jd.frequencyHz = %.15lef;\n", this.FrequencyHz)
	Log("  jd.dampingRatio = %.15lef;\n", this.DampingRatio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}

/// Friction joint definition.
type FrictionJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The maximum friction force in N.
	MaxForce float64

	/// The maximum friction torque in N-m.
	MaxTorque float64
}

func NewFrictionJointDef() *FrictionJointDef {
	this := new(FrictionJointDef)
	this.Type = Joint_e_frictionJoint
	this.LocalAnchorA.SetZero()
	this.LocalAnchorB.SetZero()
	this.MaxForce = 0.0
	this.MaxTorque = 0.0
	return this
}

/// Initialize the bodies, anchors, axis, and reference angle using the world
/// anchor and world axis.
func (this *FrictionJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2) {
	this.BodyA = bodyA
	this.BodyB = bodyB
	this.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	this.LocalAnchorB = bodyB.GetLocalPoint(anchor)
}

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
type FrictionJoint struct {
	Joint

	LocalAnchorA Vec2
	LocalAnchorB Vec2

	// Solver shared
	LinearImpulse  Vec2
	AngularImpulse float64
	MaxForce       float64
	MaxTorque      float64

	// Solver temp
	IndexA       int
	IndexB       int
	RA           Vec2
	RB           Vec2
	LocalCenterA Vec2
	LocalCenterB Vec2
	InvMassA     float64
	InvMassB     float64
	InvIA        float64
	InvIB        float64
	LinearMass   Mat22
	AngularMass  float64
}

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

func NewFrictionJoint(def *FrictionJointDef) *FrictionJoint {
	this := new(FrictionJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.LocalAnchorA = def.LocalAnchorA
	this.LocalAnchorB = def.LocalAnchorB

	this.LinearImpulse.SetZero()
	this.AngularImpulse = 0.0

	this.MaxForce = def.MaxForce
	this.MaxTorque = def.MaxTorque
	return this
}

func (this *FrictionJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *FrictionJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

func (this *FrictionJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt, this.LinearImpulse)
}

func (this *FrictionJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * this.AngularImpulse
}

/// The local anchor point relative to bodyA's origin.
func (this *FrictionJoint) GetLocalAnchorA() Vec2 {
	return this.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (this *FrictionJoint) GetLocalAnchorB() Vec2 {
	return this.LocalAnchorB
}

/// Set the maximum friction force in N.
func (this *FrictionJoint) SetMaxForce(force float64) {
	this.MaxForce = force
}

/// Get the maximum friction force in N.
func (this *FrictionJoint) GetMaxForce() float64 {
	return this.MaxForce
}

/// Set the maximum friction torque in N*m.
func (this *FrictionJoint) SetMaxTorque(torque float64) {
	this.MaxTorque = torque
}

/// Get the maximum friction torque in N*m.
func (this *FrictionJoint) GetMaxTorque() float64 {
	return this.MaxTorque
}

func (this *FrictionJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterA = this.BodyA.sweep.LocalCenter
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassA = this.BodyA.invMass
	this.InvMassB = this.BodyB.invMass
	this.InvIA = this.BodyA.invI
	this.InvIB = this.BodyB.invI

	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qA, qB := NewRot(aA), NewRot(aB)

	// Compute the effective mass matrix.
	this.RA = MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	this.RB = MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	var K Mat22
	K.Ex.X = mA + mB + iA*this.RA.Y*this.RA.Y + iB*this.RB.Y*this.RB.Y
	K.Ex.Y = -iA*this.RA.X*this.RA.Y - iB*this.RB.X*this.RB.Y
	K.Ey.X = K.Ex.Y
	K.Ey.Y = mA + mB + iA*this.RA.X*this.RA.X + iB*this.RB.X*this.RB.X

	this.LinearMass = K.GetInverse()

	this.AngularMass = iA + iB
	if this.AngularMass > 0.0 {
		this.AngularMass = 1.0 / this.AngularMass
	}

	if data.step.warmStarting {
		// Scale impulses to support a variable time step.
		this.LinearImpulse.Mul(data.step.dtRatio)
		this.AngularImpulse *= data.step.dtRatio

		P := Vec2{this.LinearImpulse.X, this.LinearImpulse.Y}
		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(this.RA, P) + this.AngularImpulse)
		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(this.RB, P) + this.AngularImpulse)
	} else {
		this.LinearImpulse.SetZero()
		this.AngularImpulse = 0.0
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *FrictionJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	mA := this.InvMassA
	mB := this.InvMassB
	iA := this.InvIA
	iB := this.InvIB

	h := data.step.dt

	// Solve angular friction
	{
		Cdot := wB - wA
		impulse := -this.AngularMass * Cdot

		oldImpulse := this.AngularImpulse
		maxImpulse := h * this.MaxTorque
		this.AngularImpulse = ClampF(this.AngularImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = this.AngularImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve linear friction
	{
		Cdot := SubVV(AddVV(vB, CrossFV(wB, this.RB)), AddVV(vA, CrossFV(wA, this.RA)))

		impulse := MulMV(this.LinearMass, Cdot)
		impulse = impulse.Minus()
		oldImpulse := this.LinearImpulse
		this.LinearImpulse.Add(impulse)

		maxImpulse := h * this.MaxForce

		if this.LinearImpulse.LengthSquared() > maxImpulse*maxImpulse {
			this.LinearImpulse.Normalize()
			this.LinearImpulse.Mul(maxImpulse)
		}

		impulse = SubVV(this.LinearImpulse, oldImpulse)

		vA.Sub(MulFV(mA, impulse))
		wA -= iA * CrossVV(this.RA, impulse)

		vB.Add(MulFV(mB, impulse))
		wB += iB * CrossVV(this.RB, impulse)
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *FrictionJoint) SolvePositionConstraints(data *solverData) bool {
	return true
}

/// Dump joint to b2Log
func (this *FrictionJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	Log("  b2FrictionJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%d);\n", this.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this.LocalAnchorA.X, this.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this.LocalAnchorB.X, this.LocalAnchorB.Y)
	Log("  jd.maxForce = %.15lef;\n", this.MaxForce)
	Log("  jd.maxTorque = %.15lef;\n", this.MaxTorque)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in b2JointDef.
type RopeJointDef struct {
	JointDef

	/// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	/// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	/// The maximum length of the rope.
	/// Warning: this must be larger than b2_linearSlop or
	/// the joint will have no effect.
	MaxLength float64
}

func NewRopeJointDef() *RopeJointDef {
	this := new(RopeJointDef)
	this.Type = Joint_e_ropeJoint
	this.LocalAnchorA.Set(-1.0, 0.0)
	this.LocalAnchorB.Set(1.0, 0.0)
	this.MaxLength = 0.0
	return this
}

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so I chose not to implement it
/// that way. See b2DistanceJoint if you want to dynamically
/// control length.
type RopeJoint struct {
	Joint

	// Solver shared
	LocalAnchorA Vec2
	LocalAnchorB Vec2
	MaxLength    float64
	Length       float64
	Impulse      float64

	// Solver temp
	IndexA       int
	IndexB       int
	U            Vec2
	RA           Vec2
	RB           Vec2
	LocalCenterA Vec2
	LocalCenterB Vec2
	InvMassA     float64
	InvMassB     float64
	InvIA        float64
	InvIB        float64
	Mass         float64
	State        LimitState
}

// Limit:
// C = norm(pB - pA) - L
// u = (pB - pA) / norm(pB - pA)
// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
// J = [-u -cross(rA, u) u cross(rB, u)]
// K = J * invM * JT
//   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

func NewRopeJoint(def *RopeJointDef) *RopeJoint {
	this := new(RopeJoint)
	this.Type = def.Type
	this.BodyA = def.BodyA
	this.BodyB = def.BodyB
	this.CollideConnected = def.CollideConnected
	this.UserData = def.UserData

	this.LocalAnchorA = def.LocalAnchorA
	this.LocalAnchorB = def.LocalAnchorB

	this.MaxLength = def.MaxLength

	this.Mass = 0.0
	this.Impulse = 0.0
	this.State = LimitState_e_inactiveLimit
	this.Length = 0.0
	return this
}

func (this *RopeJoint) GetAnchorA() Vec2 {
	return this.BodyA.GetWorldPoint(this.LocalAnchorA)
}

func (this *RopeJoint) GetAnchorB() Vec2 {
	return this.BodyB.GetWorldPoint(this.LocalAnchorB)
}

func (this *RopeJoint) GetReactionForce(inv_dt float64) Vec2 {
	F := MulFV((inv_dt * this.Impulse), this.U)
	return F
}

func (this *RopeJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

/// The local anchor point relative to bodyA's origin.
func (this *RopeJoint) GetLocalAnchorA() Vec2 {
	return this.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (this *RopeJoint) GetLocalAnchorB() Vec2 {
	return this.LocalAnchorB
}

/// Set/Get the maximum length of the rope.
func (this *RopeJoint) SetMaxLength(length float64) {
	this.MaxLength = length
}

func (this *RopeJoint) GetMaxLength() float64 {
	return this.MaxLength
}

func (this *RopeJoint) GetLimitState() LimitState {
	return this.State
}

func (this *RopeJoint) InitVelocityConstraints(data *solverData) {
	this.IndexA = this.BodyA.islandIndex
	this.IndexB = this.BodyB.islandIndex
	this.LocalCenterA = this.BodyA.sweep.LocalCenter
	this.LocalCenterB = this.BodyB.sweep.LocalCenter
	this.InvMassA = this.BodyA.invMass
	this.InvMassB = this.BodyB.invMass
	this.InvIA = this.BodyA.invI
	this.InvIB = this.BodyB.invI

	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w

	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	qA, qB := NewRot(aA), NewRot(aB)

	this.RA = MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	this.RB = MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))
	this.U = SubVV(AddVV(cB, this.RB), AddVV(cA, this.RA))

	this.Length = this.U.Length()

	C := this.Length - this.MaxLength
	if C > 0.0 {
		this.State = LimitState_e_atUpperLimit
	} else {
		this.State = LimitState_e_inactiveLimit
	}

	if this.Length > LinearSlop {
		this.U.Mul(1.0 / this.Length)
	} else {
		this.U.SetZero()
		this.Mass = 0.0
		this.Impulse = 0.0
		return
	}

	// Compute effective mass.
	crA := CrossVV(this.RA, this.U)
	crB := CrossVV(this.RB, this.U)
	invMass := this.InvMassA + this.InvIA*crA*crA + this.InvMassB + this.InvIB*crB*crB

	this.Mass = 0.0
	if invMass != 0.0 {
		this.Mass = 1.0 / invMass
	}

	if data.step.warmStarting {
		// Scale the impulse to support a variable time step.
		this.Impulse *= data.step.dtRatio

		P := MulFV(this.Impulse, this.U)
		vA.Sub(MulFV(this.InvMassA, P))
		wA -= this.InvIA * CrossVV(this.RA, P)
		vB.Add(MulFV(this.InvMassB, P))
		wB += this.InvIB * CrossVV(this.RB, P)
	} else {
		this.Impulse = 0.0
	}

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *RopeJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[this.IndexA].v
	wA := data.velocities[this.IndexA].w
	vB := data.velocities[this.IndexB].v
	wB := data.velocities[this.IndexB].w

	// Cdot = dot(u, v + cross(w, r))
	vpA := AddVV(vA, CrossFV(wA, this.RA))
	vpB := AddVV(vB, CrossFV(wB, this.RB))
	C := this.Length - this.MaxLength
	Cdot := DotVV(this.U, SubVV(vpB, vpA))

	// Predictive constraint.
	if C < 0.0 {
		Cdot += data.step.inv_dt * C
	}

	impulse := -this.Mass * Cdot
	oldImpulse := this.Impulse
	this.Impulse = MinF(0.0, this.Impulse+impulse)
	impulse = this.Impulse - oldImpulse

	P := MulFV(impulse, this.U)
	vA.Sub(MulFV(this.InvMassA, P))
	wA -= this.InvIA * CrossVV(this.RA, P)
	vB.Add(MulFV(this.InvMassB, P))
	wB += this.InvIB * CrossVV(this.RB, P)

	data.velocities[this.IndexA].v = vA
	data.velocities[this.IndexA].w = wA
	data.velocities[this.IndexB].v = vB
	data.velocities[this.IndexB].w = wB
}

func (this *RopeJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[this.IndexA].c
	aA := data.positions[this.IndexA].a
	cB := data.positions[this.IndexB].c
	aB := data.positions[this.IndexB].a

	qA, qB := NewRot(aA), NewRot(aB)

	rA := MulRV(qA, SubVV(this.LocalAnchorA, this.LocalCenterA))
	rB := MulRV(qB, SubVV(this.LocalAnchorB, this.LocalCenterB))
	u := SubVV(AddVV(cB, rB), AddVV(cA, rA))

	length := u.Normalize()
	C := length - this.MaxLength

	C = ClampF(C, 0.0, MaxLinearCorrection)

	impulse := -this.Mass * C
	P := MulFV(impulse, u)

	cA.Sub(MulFV(this.InvMassA, P))
	aA -= this.InvIA * CrossVV(rA, P)
	cB.Add(MulFV(this.InvMassB, P))
	aB += this.InvIB * CrossVV(rB, P)

	data.positions[this.IndexA].c = cA
	data.positions[this.IndexA].a = aA
	data.positions[this.IndexB].c = cB
	data.positions[this.IndexB].a = aB

	return length-this.MaxLength < LinearSlop
}

/// Dump joint to b2Log
func (this *RopeJoint) Dump() {
	indexA := this.BodyA.islandIndex
	indexB := this.BodyB.islandIndex

	Log("  b2RopeJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%d);\n", this.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", this.LocalAnchorA.X, this.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", this.LocalAnchorB.X, this.LocalAnchorB.Y)
	Log("  jd.maxLength = %.15lef;\n", this.MaxLength)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", this.Index)
}
