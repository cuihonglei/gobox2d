package box2d

type JointType byte

const (
	Joint_e_unknownJoint JointType = iota
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

type LimitState byte

const (
	LimitState_e_inactiveLimit LimitState = iota
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

func (jd *JointDef) GetType() JointType {
	return jd.Type
}

func (jd *JointDef) GetBodyA() *Body {
	return jd.BodyA
}

func (jd *JointDef) GetBodyB() *Body {
	return jd.BodyB
}

func (jd *JointDef) GetCollideConnected() bool {
	return jd.CollideConnected
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

func (joint *Joint) GetType() JointType {
	return joint.Type
}

/// Get the first body attached to this joint.
func (joint *Joint) GetBodyA() *Body {
	return joint.BodyA
}

/// Get the second body attached to this joint.
func (joint *Joint) GetBodyB() *Body {
	return joint.BodyB
}

// Get the next joint the world joint list.
func (joint *Joint) GetNext() IJoint {
	return joint.Next
}

func (joint *Joint) SetNext(next IJoint) {
	joint.Next = next
}

func (joint *Joint) GetPrev() IJoint {
	return joint.Prev
}

func (joint *Joint) SetPrev(prev IJoint) {
	joint.Prev = prev
}

func (joint *Joint) GetEdgeA() *JointEdge {
	return &joint.EdgeA
}

func (joint *Joint) GetEdgeB() *JointEdge {
	return &joint.EdgeB
}

// Get the user data pointer.
func (joint *Joint) GetUserData() interface{} {
	return joint.UserData
}

// Set the user data pointer.
func (joint *Joint) SetUserData(data interface{}) {
	joint.UserData = data
}

// Short-cut function to determine if either body is inactive.
func (joint *Joint) IsActive() bool {
	return joint.BodyA.IsActive() && joint.BodyB.IsActive()
}

// Get collide connected.
// Note: modifying the collide connect flag won't work correctly because
// the flag is only checked when fixture AABBs begin to overlap.
func (joint *Joint) GetCollideConnected() bool {
	return joint.CollideConnected
}

func (joint *Joint) GetIsland() bool {
	return joint.IslandFlag
}

func (joint *Joint) SetIsland(flag bool) {
	joint.IslandFlag = flag
}

func (joint *Joint) GetIndex() int {
	return joint.Index
}

func (joint *Joint) SetIndex(index int) {
	joint.Index = index
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
	jd := new(DistanceJointDef)
	jd.Type = Joint_e_distanceJoint
	jd.LocalAnchorA.Set(0.0, 0.0)
	jd.LocalAnchorB.Set(0.0, 0.0)
	jd.Length = 1.0
	jd.FrequencyHz = 0.0
	jd.DampingRatio = 0.0
	return jd
}

// Initialize the bodies, anchors, and length using the world
// anchors.
func (jd *DistanceJointDef) Initialize(bodyA *Body, bodyB *Body, anchorA Vec2, anchorB Vec2) {
	jd.BodyA = bodyA
	jd.BodyB = bodyB
	jd.LocalAnchorA = bodyA.GetLocalPoint(anchorA)
	jd.LocalAnchorB = bodyB.GetLocalPoint(anchorB)
	d := SubVV(anchorB, anchorA)
	jd.Length = d.Length()
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
	joint := new(DistanceJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.LocalAnchorA = def.LocalAnchorA
	joint.LocalAnchorB = def.LocalAnchorB
	joint.Length = def.Length
	joint.FrequencyHz = def.FrequencyHz
	joint.DampingRatio = def.DampingRatio
	return joint
}

func (joint *DistanceJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *DistanceJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

/// Get the reaction force given the inverse time step.
/// Unit is N.
func (joint *DistanceJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt*joint.Impulse, joint.U)
}

/// Get the reaction torque given the inverse time step.
/// Unit is N*m. This is always zero for a distance joint.
func (joint *DistanceJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

/// The local anchor point relative to bodyA's origin.
func (joint *DistanceJoint) GetLocalAnchorA() Vec2 {
	return joint.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint *DistanceJoint) GetLocalAnchorB() Vec2 {
	return joint.LocalAnchorB
}

/// Set/get the natural length.
/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
func (joint *DistanceJoint) SetLength(length float64) {
	joint.Length = length
}

func (joint *DistanceJoint) GetLength() float64 {
	return joint.Length
}

/// Set/get frequency in Hz.
func (joint *DistanceJoint) SetFrequency(hz float64) {
	joint.FrequencyHz = hz
}

func (joint *DistanceJoint) GetFrequency() float64 {
	return joint.FrequencyHz
}

/// Set/get damping ratio.
func (joint *DistanceJoint) SetDampingRatio(ratio float64) {
	joint.DampingRatio = ratio
}

func (joint *DistanceJoint) GetDampingRatio() float64 {
	return joint.DampingRatio
}

func (joint *DistanceJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterA = joint.BodyA.sweep.LocalCenter
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassA = joint.BodyA.invMass
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIA = joint.BodyA.invI
	joint.InvIB = joint.BodyB.invI

	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qA := MakeRot(aA)
	qB := MakeRot(aB)

	joint.RA = MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	joint.RB = MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))
	joint.U = SubVV(AddVV(cB, joint.RB), AddVV(cA, joint.RA))

	// Handle singularity.
	length := joint.U.Length()
	if length > LinearSlop {
		joint.U.Mul(1.0 / length)
	} else {
		joint.U.Set(0.0, 0.0)
	}

	crAu := CrossVV(joint.RA, joint.U)
	crBu := CrossVV(joint.RB, joint.U)
	invMass := joint.InvMassA + joint.InvIA*crAu*crAu + joint.InvMassB + joint.InvIB*crBu*crBu

	// Compute the effective mass matrix.
	joint.Mass = 0.0
	if invMass != 0.0 {
		joint.Mass = 1.0 / invMass
	}

	if joint.FrequencyHz > 0.0 {
		C := length - joint.Length

		// Frequency
		omega := 2.0 * Pi * joint.FrequencyHz

		// Damping coefficient
		d := 2.0 * joint.Mass * joint.DampingRatio * omega

		// Spring stiffness
		k := joint.Mass * omega * omega

		// magic formulas
		h := data.step.dt
		joint.Gamma = h * (d + h*k)
		joint.Gamma = 0.0
		if joint.Gamma != 0.0 {
			joint.Gamma = 1.0 / joint.Gamma
		}
		joint.Bias = C * h * k * joint.Gamma

		invMass += joint.Gamma
		joint.Mass = 0.0
		if invMass != 0.0 {
			joint.Mass = 1.0 / invMass
		}
	} else {
		joint.Gamma = 0.0
		joint.Bias = 0.0
	}

	if data.step.warmStarting {
		// Scale the impulse to support a variable time step.
		joint.Impulse *= data.step.dtRatio

		P := MulFV(joint.Impulse, joint.U)
		vA.Sub(MulFV(joint.InvMassA, P))
		wA -= joint.InvIA * CrossVV(joint.RA, P)
		vB.Add(MulFV(joint.InvMassB, P))
		wB += joint.InvIB * CrossVV(joint.RB, P)
	} else {
		joint.Impulse = 0.0
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *DistanceJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	// Cdot = dot(u, v + cross(w, r))
	vpA := AddVV(vA, CrossFV(wA, joint.RA))
	vpB := AddVV(vB, CrossFV(wB, joint.RB))
	Cdot := DotVV(joint.U, SubVV(vpB, vpA))

	impulse := -joint.Mass * (Cdot + joint.Bias + joint.Gamma*joint.Impulse)
	joint.Impulse += impulse

	P := MulFV(impulse, joint.U)
	vA.Sub(MulFV(joint.InvMassA, P))
	wA -= joint.InvIA * CrossVV(joint.RA, P)
	vB.Add(MulFV(joint.InvMassB, P))
	wB += joint.InvIB * CrossVV(joint.RB, P)

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *DistanceJoint) SolvePositionConstraints(data *solverData) bool {
	if joint.FrequencyHz > 0.0 {
		// There is no position correction for soft distance constraints.
		return true
	}

	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a

	qA := MakeRot(aA)
	qB := MakeRot(aB)

	rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))
	u := SubVV(AddVV(cB, rB), AddVV(cA, rA))

	length := u.Normalize()
	C := length - joint.Length
	C = ClampF(C, -MaxLinearCorrection, MaxLinearCorrection)

	impulse := -joint.Mass * C
	P := MulFV(impulse, u)

	cA.Sub(MulFV(joint.InvMassA, P))
	aA -= joint.InvIA * CrossVV(rA, P)
	cB.Add(MulFV(joint.InvMassB, P))
	aB += joint.InvIB * CrossVV(rB, P)

	data.positions[joint.IndexA].c = cA
	data.positions[joint.IndexA].a = aA
	data.positions[joint.IndexB].c = cB
	data.positions[joint.IndexB].a = aB

	return AbsF(C) < LinearSlop
}

/// Dump joint to dmLog
func (joint *DistanceJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	Log("  b2DistanceJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15e, %.15e);\n", joint.LocalAnchorA.X, joint.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15e, %.15e);\n", joint.LocalAnchorB.X, joint.LocalAnchorB.Y)
	Log("  jd.length = %.15e;\n", joint.Length)
	Log("  jd.frequencyHz = %.15e;\n", joint.FrequencyHz)
	Log("  jd.dampingRatio = %.15e;\n", joint.DampingRatio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
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

func MakeMouseJointDef() MouseJointDef {
	jd := MouseJointDef{}
	jd.Type = Joint_e_mouseJoint
	jd.Target.Set(0.0, 0.0)
	jd.MaxForce = 0.0
	jd.FrequencyHz = 5.0
	jd.DampingRatio = 0.7
	return jd
}

func NewMouseJointDef() *MouseJointDef {
	jd := MakeMouseJointDef()
	return &jd
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
	joint := new(MouseJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.TargetA = def.Target
	joint.LocalAnchorB = MulXT(joint.BodyB.GetTransform(), joint.TargetA)
	joint.MaxForce = def.MaxForce
	joint.Impulse.SetZero()
	joint.FrequencyHz = def.FrequencyHz
	joint.DampingRatio = def.DampingRatio
	return joint
}

/// Implements b2Joint.
func (joint *MouseJoint) GetAnchorA() Vec2 {
	return joint.TargetA
}

/// Implements b2Joint.
func (joint *MouseJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

/// Implements b2Joint.
func (joint *MouseJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt, joint.Impulse)
}

/// Implements b2Joint.
func (joint *MouseJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * 0.0
}

/// Use this to update the target point.
func (joint *MouseJoint) SetTarget(target Vec2) {
	if !joint.BodyB.IsAwake() {
		joint.BodyB.SetAwake(true)
	}
	joint.TargetA = target
}

func (joint *MouseJoint) GetTarget() Vec2 {
	return joint.TargetA
}

/// Set/get the maximum force in Newtons.
func (joint *MouseJoint) SetMaxForce(force float64) {
	joint.MaxForce = force
}

func (joint *MouseJoint) GetMaxForce() float64 {
	return joint.MaxForce
}

/// Set/get the frequency in Hertz.
func (joint *MouseJoint) SetFrequency(hz float64) {
	joint.FrequencyHz = hz
}

func (joint *MouseJoint) GetFrequency() float64 {
	return joint.FrequencyHz
}

/// Set/get the damping ratio (dimensionless).
func (joint *MouseJoint) SetDampingRatio(ratio float64) {
	joint.DampingRatio = ratio
}

func (joint *MouseJoint) GetDampingRatio() float64 {
	return joint.DampingRatio
}

/// The mouse joint does not support dumping.
func (joint *MouseJoint) Dump() {
	Log("Mouse joint dumping is not supported.\n")
}

func (joint *MouseJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIB = joint.BodyB.invI

	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qB := MakeRot(aB)

	mass := joint.BodyB.GetMass()

	// Frequency
	omega := 2.0 * Pi * joint.FrequencyHz

	// Damping coefficient
	d := 2.0 * mass * joint.DampingRatio * omega

	// Spring stiffness
	k := mass * (omega * omega)

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	h := data.step.dt
	//b2Assert(d + h * k > b2_epsilon);
	joint.Gamma = h * (d + h*k)
	if joint.Gamma != 0.0 {
		joint.Gamma = 1.0 / joint.Gamma
	}
	joint.Beta = h * k * joint.Gamma

	// Compute the effective mass matrix.
	joint.RB = MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	var K Mat22
	K.Ex.X = joint.InvMassB + joint.InvIB*joint.RB.Y*joint.RB.Y + joint.Gamma
	K.Ex.Y = -joint.InvIB * joint.RB.X * joint.RB.Y
	K.Ey.X = K.Ex.Y
	K.Ey.Y = joint.InvMassB + joint.InvIB*joint.RB.X*joint.RB.X + joint.Gamma

	joint.Mass = K.GetInverse()

	joint.C = SubVV(AddVV(cB, joint.RB), joint.TargetA)
	joint.C.Mul(joint.Beta)

	// Cheat with some damping
	wB *= 0.98

	if data.step.warmStarting {
		joint.Impulse.Mul(data.step.dtRatio)
		vB.Add(MulFV(joint.InvMassB, joint.Impulse))
		wB += joint.InvIB * CrossVV(joint.RB, joint.Impulse)
	} else {
		joint.Impulse.SetZero()
	}

	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *MouseJoint) SolveVelocityConstraints(data *solverData) {
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	// Cdot = v + cross(w, r)
	Cdot := AddVV(vB, CrossFV(wB, joint.RB))
	tmpv := AddVV(AddVV(Cdot, joint.C), MulFV(joint.Gamma, joint.Impulse))
	impulse := MulMV(joint.Mass, tmpv.Minus())

	oldImpulse := joint.Impulse
	joint.Impulse.Add(impulse)
	maxImpulse := data.step.dt * joint.MaxForce
	if joint.Impulse.LengthSquared() > maxImpulse*maxImpulse {
		joint.Impulse.Mul(maxImpulse / joint.Impulse.Length())
	}
	impulse = SubVV(joint.Impulse, oldImpulse)

	vB.Add(MulFV(joint.InvMassB, impulse))
	wB += joint.InvIB * CrossVV(joint.RB, impulse)

	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *MouseJoint) SolvePositionConstraints(data *solverData) bool {
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

func MakePrismaticJointDef() PrismaticJointDef {
	jd := PrismaticJointDef{}
	jd.Type = Joint_e_prismaticJoint
	jd.LocalAnchorA.SetZero()
	jd.LocalAnchorB.SetZero()
	jd.LocalAxisA.Set(1.0, 0.0)
	jd.ReferenceAngle = 0.0
	jd.EnableLimit = false
	jd.LowerTranslation = 0.0
	jd.UpperTranslation = 0.0
	jd.EnableMotor = false
	jd.MaxMotorForce = 0.0
	jd.MotorSpeed = 0.0
	return jd
}

func NewPrismaticJointDef() *PrismaticJointDef {
	jd := MakePrismaticJointDef()
	return &jd
}

// Initialize the bodies, anchors, axis, and reference angle using the world
// anchor and unit world axis.
func (jd *PrismaticJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2, axis Vec2) {
	jd.BodyA = bodyA
	jd.BodyB = bodyB
	jd.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	jd.LocalAnchorB = bodyB.GetLocalPoint(anchor)
	jd.LocalAxisA = bodyA.GetLocalVector(axis)
	jd.ReferenceAngle = bodyB.GetAngle() - bodyA.GetAngle()
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
	joint := new(PrismaticJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.LocalAnchorA = def.LocalAnchorA
	joint.LocalAnchorB = def.LocalAnchorB
	joint.LocalXAxisA = def.LocalAxisA
	joint.LocalXAxisA.Normalize()
	joint.LocalYAxisA = CrossFV(1.0, joint.LocalXAxisA)
	joint.ReferenceAngle = def.ReferenceAngle

	joint.Impulse.SetZero()
	joint.MotorMass = 0.0
	joint.MotorImpulse = 0.0

	joint.LowerTranslation = def.LowerTranslation
	joint.UpperTranslation = def.UpperTranslation
	joint.MaxMotorForce = def.MaxMotorForce
	joint.MotorSpeed = def.MotorSpeed
	joint.EnableLimit = def.EnableLimit
	joint.EnableMotor = def.EnableMotor
	joint.LimitState = LimitState_e_inactiveLimit

	joint.Axis.SetZero()
	joint.Perp.SetZero()
	return joint
}

func (joint *PrismaticJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *PrismaticJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

func (joint *PrismaticJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt, AddVV(MulFV(joint.Impulse.X, joint.Perp), MulFV(joint.MotorImpulse+joint.Impulse.Z, joint.Axis)))
}

func (joint *PrismaticJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.Impulse.Y
}

/// The local anchor point relative to bodyA's origin.
func (joint *PrismaticJoint) GetLocalAnchorA() Vec2 {
	return joint.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint *PrismaticJoint) GetLocalAnchorB() Vec2 {
	return joint.LocalAnchorB
}

/// The local joint axis relative to bodyA.
func (joint *PrismaticJoint) GetLocalAxisA() Vec2 {
	return joint.LocalXAxisA
}

/// Get the reference angle.
func (joint *PrismaticJoint) GetReferenceAngle() float64 {
	return joint.ReferenceAngle
}

/// Get the current joint translation, usually in meters.
func (joint *PrismaticJoint) GetJointTranslation() float64 {
	pA := joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
	pB := joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
	d := SubVV(pB, pA)
	axis := joint.BodyA.GetWorldVector(joint.LocalXAxisA)

	translation := DotVV(d, axis)
	return translation
}

/// Get the current joint translation speed, usually in meters per second.
func (joint *PrismaticJoint) GetJointSpeed() float64 {
	bA := joint.BodyA
	bB := joint.BodyB

	rA := MulRV(bA.xf.Q, SubVV(joint.LocalAnchorA, bA.sweep.LocalCenter))
	rB := MulRV(bB.xf.Q, SubVV(joint.LocalAnchorB, bB.sweep.LocalCenter))
	p1 := AddVV(bA.sweep.C, rA)
	p2 := AddVV(bB.sweep.C, rB)
	d := SubVV(p2, p1)
	axis := MulRV(bA.xf.Q, joint.LocalXAxisA)

	vA := bA.linearVelocity
	vB := bB.linearVelocity
	wA := bA.angularVelocity
	wB := bB.angularVelocity

	speed := DotVV(d, CrossFV(wA, axis)) + DotVV(axis, SubVV(AddVV(vB, CrossFV(wB, rB)), AddVV(vA, CrossFV(wA, rA))))
	return speed
}

/// Is the joint limit enabled?
func (joint *PrismaticJoint) IsLimitEnabled() bool {
	return joint.EnableLimit
}

/// Enable/disable the joint limit.
func (joint *PrismaticJoint) SetEnableLimit(flag bool) {
	if flag != joint.EnableLimit {
		joint.BodyA.SetAwake(true)
		joint.BodyB.SetAwake(true)
		joint.EnableLimit = flag
		joint.Impulse.Z = 0.0
	}
}

/// Get the lower joint limit, usually in meters.
func (joint *PrismaticJoint) GetLowerLimit() float64 {
	return joint.LowerTranslation
}

/// Get the upper joint limit, usually in meters.
func (joint *PrismaticJoint) GetUpperLimit() float64 {
	return joint.UpperTranslation
}

/// Set the joint limits, usually in meters.
func (joint *PrismaticJoint) SetLimits(lower float64, upper float64) {
	//b2Assert(lower <= upper);
	if lower != joint.LowerTranslation || upper != joint.UpperTranslation {
		joint.BodyA.SetAwake(true)
		joint.BodyB.SetAwake(true)
		joint.LowerTranslation = lower
		joint.UpperTranslation = upper
		joint.Impulse.Z = 0.0
	}
}

/// Is the joint motor enabled?
func (joint *PrismaticJoint) IsMotorEnabled() bool {
	return joint.EnableMotor
}

/// Enable/disable the joint motor.
func (joint *PrismaticJoint) SetEnableMotor(flag bool) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.EnableMotor = flag
}

/// Set the motor speed, usually in meters per second.
func (joint *PrismaticJoint) SetMotorSpeed(speed float64) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.MotorSpeed = speed
}

/// Get the motor speed, usually in meters per second.
func (joint *PrismaticJoint) GetMotorSpeed() float64 {
	return joint.MotorSpeed
}

/// Set the maximum motor force, usually in N.
func (joint *PrismaticJoint) SetMaxMotorForce(force float64) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.MaxMotorForce = force
}

func (joint *PrismaticJoint) GetMaxMotorForce() float64 {
	return joint.MaxMotorForce
}

/// Get the current motor force given the inverse time step, usually in N.
func (joint *PrismaticJoint) GetMotorForce(inv_dt float64) float64 {
	return inv_dt * joint.MotorImpulse
}

func (joint *PrismaticJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterA = joint.BodyA.sweep.LocalCenter
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassA = joint.BodyA.invMass
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIA = joint.BodyA.invI
	joint.InvIB = joint.BodyB.invI

	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qA := MakeRot(aA)
	qB := MakeRot(aB)

	// Compute the effective masses.
	rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))
	d := AddVV(SubVV(cB, cA), SubVV(rB, rA))

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	// Compute motor Jacobian and effective mass.
	{
		joint.Axis = MulRV(qA, joint.LocalXAxisA)
		joint.A1 = CrossVV(AddVV(d, rA), joint.Axis)
		joint.A2 = CrossVV(rB, joint.Axis)

		joint.MotorMass = mA + mB + iA*joint.A1*joint.A1 + iB*joint.A2*joint.A2
		if joint.MotorMass > 0.0 {
			joint.MotorMass = 1.0 / joint.MotorMass
		}
	}

	// Prismatic constraint.
	{
		joint.Perp = MulRV(qA, joint.LocalYAxisA)

		joint.S1 = CrossVV(AddVV(d, rA), joint.Perp)
		joint.S2 = CrossVV(rB, joint.Perp)

		k11 := mA + mB + iA*joint.S1*joint.S1 + iB*joint.S2*joint.S2
		k12 := iA*joint.S1 + iB*joint.S2
		k13 := iA*joint.S1*joint.A1 + iB*joint.S2*joint.A2
		k22 := iA + iB
		if k22 == 0.0 {
			// For bodies with fixed rotation.
			k22 = 1.0
		}
		k23 := iA*joint.A1 + iB*joint.A2
		k33 := mA + mB + iA*joint.A1*joint.A1 + iB*joint.A2*joint.A2

		joint.K.Ex.Set(k11, k12, k13)
		joint.K.Ey.Set(k12, k22, k23)
		joint.K.Ez.Set(k13, k23, k33)
	}

	// Compute motor and limit terms.
	if joint.EnableLimit {
		jointTranslation := DotVV(joint.Axis, d)
		if AbsF(joint.UpperTranslation-joint.LowerTranslation) < 2.0*LinearSlop {
			joint.LimitState = LimitState_e_equalLimits
		} else if jointTranslation <= joint.LowerTranslation {
			if joint.LimitState != LimitState_e_atLowerLimit {
				joint.LimitState = LimitState_e_atLowerLimit
				joint.Impulse.Z = 0.0
			}
		} else if jointTranslation >= joint.UpperTranslation {
			if joint.LimitState != LimitState_e_atUpperLimit {
				joint.LimitState = LimitState_e_atUpperLimit
				joint.Impulse.Z = 0.0
			}
		} else {
			joint.LimitState = LimitState_e_inactiveLimit
			joint.Impulse.Z = 0.0
		}
	} else {
		joint.LimitState = LimitState_e_inactiveLimit
		joint.Impulse.Z = 0.0
	}

	if !joint.EnableMotor {
		joint.MotorImpulse = 0.0
	}

	if data.step.warmStarting {
		// Account for variable time step.
		joint.Impulse.Mul(data.step.dtRatio)
		joint.MotorImpulse *= data.step.dtRatio

		P := AddVV(MulFV(joint.Impulse.X, joint.Perp), MulFV(joint.MotorImpulse+joint.Impulse.Z, joint.Axis))
		LA := joint.Impulse.X*joint.S1 + joint.Impulse.Y + (joint.MotorImpulse+joint.Impulse.Z)*joint.A1
		LB := joint.Impulse.X*joint.S2 + joint.Impulse.Y + (joint.MotorImpulse+joint.Impulse.Z)*joint.A2

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	} else {
		joint.Impulse.SetZero()
		joint.MotorImpulse = 0.0
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *PrismaticJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	// Solve linear motor constraint.
	if joint.EnableMotor && joint.LimitState != LimitState_e_equalLimits {
		Cdot := DotVV(joint.Axis, SubVV(vB, vA)) + joint.A2*wB - joint.A1*wA
		impulse := joint.MotorMass * (joint.MotorSpeed - Cdot)
		oldImpulse := joint.MotorImpulse
		maxImpulse := data.step.dt * joint.MaxMotorForce
		joint.MotorImpulse = ClampF(joint.MotorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.MotorImpulse - oldImpulse

		P := MulFV(impulse, joint.Axis)
		LA := impulse * joint.A1
		LB := impulse * joint.A2

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	}

	var Cdot1 Vec2
	Cdot1.X = DotVV(joint.Perp, SubVV(vB, vA)) + joint.S2*wB - joint.S1*wA
	Cdot1.Y = wB - wA

	if joint.EnableLimit && joint.LimitState != LimitState_e_inactiveLimit {
		// Solve prismatic and limit constraint in block form.
		Cdot2 := DotVV(joint.Axis, SubVV(vB, vA)) + joint.A2*wB - joint.A1*wA
		Cdot := Vec3{Cdot1.X, Cdot1.Y, Cdot2}

		f1 := joint.Impulse
		df := joint.K.Solve33(Cdot.Minus())
		joint.Impulse.Add(df)

		if joint.LimitState == LimitState_e_atLowerLimit {
			joint.Impulse.Z = MaxF(joint.Impulse.Z, 0.0)
		} else if joint.LimitState == LimitState_e_atUpperLimit {
			joint.Impulse.Z = MinF(joint.Impulse.Z, 0.0)
		}

		// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
		b := SubVV(Cdot1.Minus(), MulFV((joint.Impulse.Z-f1.Z), Vec2{joint.K.Ez.X, joint.K.Ez.Y}))
		f2r := AddVV(joint.K.Solve22(b), Vec2{f1.X, f1.Y})
		joint.Impulse.X = f2r.X
		joint.Impulse.Y = f2r.Y

		df = SubV3V3(joint.Impulse, f1)

		P := AddVV(MulFV(df.X, joint.Perp), MulFV(df.Z, joint.Axis))
		LA := df.X*joint.S1 + df.Y + df.Z*joint.A1
		LB := df.X*joint.S2 + df.Y + df.Z*joint.A2

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	} else {
		// Limit is inactive, just solve the prismatic constraint in block form.
		df := joint.K.Solve22(Cdot1.Minus())
		joint.Impulse.X += df.X
		joint.Impulse.Y += df.Y

		P := MulFV(df.X, joint.Perp)
		LA := df.X*joint.S1 + df.Y
		LB := df.X*joint.S2 + df.Y

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB

		//Cdot10 := Cdot1

		Cdot1.X = DotVV(joint.Perp, SubVV(vB, vA)) + joint.S2*wB - joint.S1*wA
		Cdot1.Y = wB - wA

		if AbsF(Cdot1.X) > 0.01 || AbsF(Cdot1.Y) > 0.01 {
			//test := MulM3V(joint.K, df)
			Cdot1.X += 0.0
		}
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *PrismaticJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a

	qA := MakeRot(aA)
	qB := MakeRot(aB)

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	// Compute fresh Jacobians
	rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))
	d := SubVV(AddVV(cB, rB), AddVV(cA, rA))

	axis := MulRV(qA, joint.LocalXAxisA)
	a1 := CrossVV(AddVV(d, rA), axis)
	a2 := CrossVV(rB, axis)
	perp := MulRV(qA, joint.LocalYAxisA)

	s1 := CrossVV(AddVV(d, rA), perp)
	s2 := CrossVV(rB, perp)

	var impulse Vec3
	var C1 Vec2
	C1.X = DotVV(perp, d)
	C1.Y = aB - aA - joint.ReferenceAngle

	linearError := AbsF(C1.X)
	angularError := AbsF(C1.Y)

	active := false
	C2 := 0.0
	if joint.EnableLimit {
		translation := DotVV(axis, d)
		if AbsF(joint.UpperTranslation-joint.LowerTranslation) < 2.0*LinearSlop {
			// Prevent large angular corrections
			C2 = ClampF(translation, -MaxLinearCorrection, MaxLinearCorrection)
			linearError = MaxF(linearError, AbsF(translation))
			active = true
		} else if translation <= joint.LowerTranslation {
			// Prevent large linear corrections and allow some slop.
			C2 = ClampF(translation-joint.LowerTranslation+LinearSlop, -MaxLinearCorrection, 0.0)
			linearError = MaxF(linearError, joint.LowerTranslation-translation)
			active = true
		} else if translation >= joint.UpperTranslation {
			// Prevent large linear corrections and allow some slop.
			C2 = ClampF(translation-joint.UpperTranslation-LinearSlop, 0.0, MaxLinearCorrection)
			linearError = MaxF(linearError, translation-joint.UpperTranslation)
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

	data.positions[joint.IndexA].c = cA
	data.positions[joint.IndexA].a = aA
	data.positions[joint.IndexB].c = cB
	data.positions[joint.IndexB].a = aB

	return linearError <= LinearSlop && angularError <= AngularSlop
}

/// Dump to b2Log
func (joint *PrismaticJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	Log("  b2PrismaticJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15e, %.15e);\n", joint.LocalAnchorA.X, joint.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15e, %.15e);\n", joint.LocalAnchorB.X, joint.LocalAnchorB.Y)
	Log("  jd.localAxisA.Set(%.15e, %.15e);\n", joint.LocalXAxisA.X, joint.LocalXAxisA.Y)
	Log("  jd.referenceAngle = %.15e;\n", joint.ReferenceAngle)
	Log("  jd.enableLimit = bool(%t);\n", joint.EnableLimit)
	Log("  jd.lowerTranslation = %.15e;\n", joint.LowerTranslation)
	Log("  jd.upperTranslation = %.15e;\n", joint.UpperTranslation)
	Log("  jd.enableMotor = bool(%t);\n", joint.EnableMotor)
	Log("  jd.motorSpeed = %.15e;\n", joint.MotorSpeed)
	Log("  jd.maxMotorForce = %.15e;\n", joint.MaxMotorForce)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
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
func MakeRevoluteJointDef() RevoluteJointDef {
	rjd := RevoluteJointDef{}
	rjd.Type = Joint_e_revoluteJoint
	return rjd
}

func NewRevoluteJointDef() *RevoluteJointDef {
	rjd := MakeRevoluteJointDef()
	return &rjd
}

/// Initialize the bodies, anchors, and reference angle using a world
/// anchor point.
func (jd *RevoluteJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2) {
	jd.BodyA = bodyA
	jd.BodyB = bodyB
	jd.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	jd.LocalAnchorB = bodyB.GetLocalPoint(anchor)
	jd.ReferenceAngle = bodyB.GetAngle() - bodyA.GetAngle()
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
	joint := new(RevoluteJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.LocalAnchorA = def.LocalAnchorA
	joint.LocalAnchorB = def.LocalAnchorB
	joint.ReferenceAngle = def.ReferenceAngle

	joint.Impulse.SetZero()
	joint.MotorImpulse = 0.0

	joint.LowerAngle = def.LowerAngle
	joint.UpperAngle = def.UpperAngle
	joint.MaxMotorTorque = def.MaxMotorTorque
	joint.MotorSpeed = def.MotorSpeed
	joint.EnableLimit = def.EnableLimit
	joint.EnableMotor = def.EnableMotor
	joint.LimitState = LimitState_e_inactiveLimit
	return joint
}

func (joint *RevoluteJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *RevoluteJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

/// The local anchor point relative to bodyA's origin.
func (joint *RevoluteJoint) GetLocalAnchorA() Vec2 {
	return joint.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint *RevoluteJoint) GetLocalAnchorB() Vec2 {
	return joint.LocalAnchorB
}

/// Get the reference angle.
func (joint *RevoluteJoint) GetReferenceAngle() float64 {
	return joint.ReferenceAngle
}

/// Get the current joint angle in radians.
func (joint *RevoluteJoint) GetJointAngle() float64 {
	bA := joint.BodyA
	bB := joint.BodyB
	return bB.sweep.A - bA.sweep.A - joint.ReferenceAngle
}

/// Get the current joint angle speed in radians per second.
func (joint *RevoluteJoint) GetJointSpeed() float64 {
	bA := joint.BodyA
	bB := joint.BodyB
	return bB.angularVelocity - bA.angularVelocity
}

/// Is the joint limit enabled?
func (joint *RevoluteJoint) IsLimitEnabled() bool {
	return joint.EnableLimit
}

/// Enable/disable the joint limit.
func (joint *RevoluteJoint) SetEnableLimit(flag bool) {
	if flag != joint.EnableLimit {
		joint.BodyA.SetAwake(true)
		joint.BodyB.SetAwake(true)
		joint.EnableLimit = flag
		joint.Impulse.Z = 0.0
	}
}

/// Get the lower joint limit in radians.
func (joint *RevoluteJoint) GetLowerLimit() float64 {
	return joint.LowerAngle
}

/// Get the upper joint limit in radians.
func (joint *RevoluteJoint) GetUpperLimit() float64 {
	return joint.UpperAngle
}

/// Set the joint limits in radians.
func (joint *RevoluteJoint) SetLimits(lower float64, upper float64) {
	//b2Assert(lower <= upper);
	if lower != joint.LowerAngle || upper != joint.UpperAngle {
		joint.BodyA.SetAwake(true)
		joint.BodyB.SetAwake(true)
		joint.Impulse.Z = 0.0
		joint.LowerAngle = lower
		joint.UpperAngle = upper
	}
}

/// Is the joint motor enabled?
func (joint *RevoluteJoint) IsMotorEnabled() bool {
	return joint.EnableMotor
}

/// Enable/disable the joint motor.
func (joint *RevoluteJoint) SetEnableMotor(flag bool) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.EnableMotor = flag
}

/// Set the motor speed in radians per second.
func (joint *RevoluteJoint) SetMotorSpeed(speed float64) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.MotorSpeed = speed
}

/// Get the motor speed in radians per second.
func (joint *RevoluteJoint) GetMotorSpeed() float64 {
	return joint.MotorSpeed
}

/// Set the maximum motor torque, usually in N-m.
func (joint *RevoluteJoint) SetMaxMotorTorque(torque float64) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.MaxMotorTorque = torque
}

func (joint *RevoluteJoint) GetMaxMotorTorque() float64 {
	return joint.MaxMotorTorque
}

/// Get the reaction force given the inverse time step.
/// Unit is N.
func (joint *RevoluteJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := Vec2{joint.Impulse.X, joint.Impulse.Y}
	return MulFV(inv_dt, P)
}

/// Get the reaction torque due to the joint limit given the inverse time step.
/// Unit is N*m.
func (joint *RevoluteJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.Impulse.Z
}

/// Get the current motor torque given the inverse time step.
/// Unit is N*m.
func (joint *RevoluteJoint) GetMotorTorque(inv_dt float64) float64 {
	return inv_dt * joint.MotorImpulse
}

func (joint *RevoluteJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterA = joint.BodyA.sweep.LocalCenter
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassA = joint.BodyA.invMass
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIA = joint.BodyA.invI
	joint.InvIB = joint.BodyB.invI

	//cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	//cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qA := MakeRot(aA)
	qB := MakeRot(aB)

	joint.RA = MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	joint.RB = MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	fixedRotation := (iA+iB == 0.0)

	joint.Mass.Ex.X = mA + mB + joint.RA.Y*joint.RA.Y*iA + joint.RB.Y*joint.RB.Y*iB
	joint.Mass.Ey.X = -joint.RA.Y*joint.RA.X*iA - joint.RB.Y*joint.RB.X*iB
	joint.Mass.Ez.X = -joint.RA.Y*iA - joint.RB.Y*iB
	joint.Mass.Ex.Y = joint.Mass.Ey.X
	joint.Mass.Ey.Y = mA + mB + joint.RA.X*joint.RA.X*iA + joint.RB.X*joint.RB.X*iB
	joint.Mass.Ez.Y = joint.RA.X*iA + joint.RB.X*iB
	joint.Mass.Ex.Z = joint.Mass.Ez.X
	joint.Mass.Ey.Z = joint.Mass.Ez.Y
	joint.Mass.Ez.Z = iA + iB

	joint.MotorMass = iA + iB
	if joint.MotorMass > 0.0 {
		joint.MotorMass = 1.0 / joint.MotorMass
	}

	if !joint.EnableMotor || fixedRotation {
		joint.MotorImpulse = 0.0
	}

	if joint.EnableLimit && !fixedRotation {
		jointAngle := aB - aA - joint.ReferenceAngle
		if AbsF(joint.UpperAngle-joint.LowerAngle) < 2.0*AngularSlop {
			joint.LimitState = LimitState_e_equalLimits
		} else if jointAngle <= joint.LowerAngle {
			if joint.LimitState != LimitState_e_atLowerLimit {
				joint.Impulse.Z = 0.0
			}
			joint.LimitState = LimitState_e_atLowerLimit
		} else if jointAngle >= joint.UpperAngle {
			if joint.LimitState != LimitState_e_atUpperLimit {
				joint.Impulse.Z = 0.0
			}
			joint.LimitState = LimitState_e_atUpperLimit
		} else {
			joint.LimitState = LimitState_e_inactiveLimit
			joint.Impulse.Z = 0.0
		}
	} else {
		joint.LimitState = LimitState_e_inactiveLimit
	}

	if data.step.warmStarting {
		// Scale impulses to support a variable time step.
		joint.Impulse.Mul(data.step.dtRatio)
		joint.MotorImpulse *= data.step.dtRatio

		P := Vec2{joint.Impulse.X, joint.Impulse.Y}

		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(joint.RA, P) + joint.MotorImpulse + joint.Impulse.Z)

		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(joint.RB, P) + joint.MotorImpulse + joint.Impulse.Z)
	} else {
		joint.Impulse.SetZero()
		joint.MotorImpulse = 0.0
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *RevoluteJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	fixedRotation := (iA+iB == 0.0)

	// Solve motor constraint.
	if joint.EnableMotor && joint.LimitState != LimitState_e_equalLimits && !fixedRotation {
		Cdot := wB - wA - joint.MotorSpeed
		impulse := -joint.MotorMass * Cdot
		oldImpulse := joint.MotorImpulse
		maxImpulse := data.step.dt * joint.MaxMotorTorque
		joint.MotorImpulse = ClampF(joint.MotorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.MotorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve limit constraint.
	if joint.EnableLimit && joint.LimitState != LimitState_e_inactiveLimit && !fixedRotation {
		Cdot1 := SubVV(AddVV(vB, CrossFV(wB, joint.RB)), AddVV(vA, CrossFV(wA, joint.RA)))
		Cdot2 := wB - wA
		Cdot := Vec3{Cdot1.X, Cdot1.Y, Cdot2}

		impulse := joint.Mass.Solve33(Cdot)
		impulse = impulse.Minus()

		if joint.LimitState == LimitState_e_equalLimits {
			joint.Impulse.Add(impulse)
		} else if joint.LimitState == LimitState_e_atLowerLimit {
			newImpulse := joint.Impulse.Z + impulse.Z
			if newImpulse < 0.0 {
				rhs := AddVV(Cdot1.Minus(), MulFV(joint.Impulse.Z, Vec2{joint.Mass.Ez.X, joint.Mass.Ez.Y}))
				reduced := joint.Mass.Solve22(rhs)
				impulse.X = reduced.X
				impulse.Y = reduced.Y
				impulse.Z = -joint.Impulse.Z
				joint.Impulse.X += reduced.X
				joint.Impulse.Y += reduced.Y
				joint.Impulse.Z = 0.0
			} else {
				joint.Impulse.Add(impulse)
			}
		} else if joint.LimitState == LimitState_e_atUpperLimit {
			newImpulse := joint.Impulse.Z + impulse.Z
			if newImpulse > 0.0 {
				rhs := AddVV(Cdot1.Minus(), MulFV(joint.Impulse.Z, Vec2{joint.Mass.Ez.X, joint.Mass.Ez.Y}))
				reduced := joint.Mass.Solve22(rhs)
				impulse.X = reduced.X
				impulse.Y = reduced.Y
				impulse.Z = -joint.Impulse.Z
				joint.Impulse.X += reduced.X
				joint.Impulse.Y += reduced.Y
				joint.Impulse.Z = 0.0
			} else {
				joint.Impulse.Add(impulse)
			}
		}

		P := Vec2{impulse.X, impulse.Y}

		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(joint.RA, P) + impulse.Z)

		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(joint.RB, P) + impulse.Z)
	} else {
		// Solve point-to-point constraint
		Cdot := SubVV(AddVV(vB, CrossFV(wB, joint.RB)), AddVV(vA, CrossFV(wA, joint.RA)))
		impulse := joint.Mass.Solve22(Cdot.Minus())

		joint.Impulse.X += impulse.X
		joint.Impulse.Y += impulse.Y

		vA.Sub(MulFV(mA, impulse))
		wA -= iA * CrossVV(joint.RA, impulse)

		vB.Add(MulFV(mB, impulse))
		wB += iB * CrossVV(joint.RB, impulse)
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *RevoluteJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a

	qA := MakeRot(aA)
	qB := MakeRot(aB)

	angularError := 0.0
	positionError := 0.0

	fixedRotation := (joint.InvIA+joint.InvIB == 0.0)

	// Solve angular limit constraint.
	if joint.EnableLimit && joint.LimitState != LimitState_e_inactiveLimit && !fixedRotation {
		angle := aB - aA - joint.ReferenceAngle
		limitImpulse := 0.0

		if joint.LimitState == LimitState_e_equalLimits {
			// Prevent large angular corrections
			C := ClampF(angle-joint.LowerAngle, -MaxAngularCorrection, MaxAngularCorrection)
			limitImpulse = -joint.MotorMass * C
			angularError = AbsF(C)
		} else if joint.LimitState == LimitState_e_atLowerLimit {
			C := angle - joint.LowerAngle
			angularError = -C

			// Prevent large angular corrections and allow some slop.
			C = ClampF(C+AngularSlop, -MaxAngularCorrection, 0.0)
			limitImpulse = -joint.MotorMass * C
		} else if joint.LimitState == LimitState_e_atUpperLimit {
			C := angle - joint.UpperAngle
			angularError = C

			// Prevent large angular corrections and allow some slop.
			C = ClampF(C-AngularSlop, 0.0, MaxAngularCorrection)
			limitImpulse = -joint.MotorMass * C
		}

		aA -= joint.InvIA * limitImpulse
		aB += joint.InvIB * limitImpulse
	}

	// Solve point-to-point constraint.
	{
		qA.Set(aA)
		qB.Set(aB)
		rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
		rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))

		C := SubVV(AddVV(cB, rB), AddVV(cA, rA))
		positionError = C.Length()

		mA := joint.InvMassA
		mB := joint.InvMassB
		iA := joint.InvIA
		iB := joint.InvIB

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

	data.positions[joint.IndexA].c = cA
	data.positions[joint.IndexA].a = aA
	data.positions[joint.IndexB].c = cB
	data.positions[joint.IndexB].a = aB

	return positionError <= LinearSlop && angularError <= AngularSlop
}

/// Dump to b2Log.
func (joint *RevoluteJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	Log("  b2RevoluteJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15e, %.15e);\n", joint.LocalAnchorA.X, joint.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15e, %.15e);\n", joint.LocalAnchorB.X, joint.LocalAnchorB.Y)
	Log("  jd.referenceAngle = %.15e;\n", joint.ReferenceAngle)
	Log("  jd.enableLimit = bool(%t);\n", joint.EnableLimit)
	Log("  jd.lowerAngle = %.15e;\n", joint.LowerAngle)
	Log("  jd.upperAngle = %.15e;\n", joint.UpperAngle)
	Log("  jd.enableMotor = bool(%t);\n", joint.EnableMotor)
	Log("  jd.motorSpeed = %.15e;\n", joint.MotorSpeed)
	Log("  jd.maxMotorTorque = %.15e;\n", joint.MaxMotorTorque)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
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
	jd := new(PulleyJointDef)
	jd.Type = Joint_e_pulleyJoint
	jd.GroundAnchorA.Set(-1.0, 1.0)
	jd.GroundAnchorB.Set(1.0, 1.0)
	jd.LocalAnchorA.Set(-1.0, 0.0)
	jd.LocalAnchorB.Set(1.0, 0.0)
	jd.LengthA = 0.0
	jd.LengthB = 0.0
	jd.Ratio = 1.0
	jd.CollideConnected = true
	return jd
}

/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
func (jd *PulleyJointDef) Initialize(bodyA *Body, bodyB *Body, groundAnchorA Vec2,
	groundAnchorB Vec2, anchorA Vec2, anchorB Vec2, ratio float64) {
	jd.BodyA = bodyA
	jd.BodyB = bodyB
	jd.GroundAnchorA = groundAnchorA
	jd.GroundAnchorB = groundAnchorB
	jd.LocalAnchorA = bodyA.GetLocalPoint(anchorA)
	jd.LocalAnchorB = bodyB.GetLocalPoint(anchorB)
	dA := SubVV(anchorA, groundAnchorA)
	jd.LengthA = dA.Length()
	dB := SubVV(anchorB, groundAnchorB)
	jd.LengthB = dB.Length()
	jd.Ratio = ratio
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
	joint := new(PulleyJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.GroundAnchorA = def.GroundAnchorA
	joint.GroundAnchorB = def.GroundAnchorB
	joint.LocalAnchorA = def.LocalAnchorA
	joint.LocalAnchorB = def.LocalAnchorB

	joint.LengthA = def.LengthA
	joint.LengthB = def.LengthB

	//b2Assert(def->ratio != 0.0f);
	joint.Ratio = def.Ratio

	joint.Constant = def.LengthA + joint.Ratio*def.LengthB

	joint.Impulse = 0.0
	return joint
}

func (joint *PulleyJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *PulleyJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

func (joint *PulleyJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := MulFV(joint.Impulse, joint.UB)
	return MulFV(inv_dt, P)
}

func (joint *PulleyJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

/// Get the first ground anchor.
func (joint *PulleyJoint) GetGroundAnchorA() Vec2 {
	return joint.GroundAnchorA
}

/// Get the second ground anchor.
func (joint *PulleyJoint) GetGroundAnchorB() Vec2 {
	return joint.GroundAnchorB
}

/// Get the current length of the segment attached to bodyA.
func (joint *PulleyJoint) GetLengthA() float64 {
	p := joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
	s := joint.GroundAnchorA
	d := SubVV(p, s)
	return d.Length()
}

/// Get the current length of the segment attached to bodyB.
func (joint *PulleyJoint) GetLengthB() float64 {
	p := joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
	s := joint.GroundAnchorB
	d := SubVV(p, s)
	return d.Length()
}

/// Get the pulley ratio.
func (joint *PulleyJoint) GetRatio() float64 {
	return joint.Ratio
}

func (joint *PulleyJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterA = joint.BodyA.sweep.LocalCenter
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassA = joint.BodyA.invMass
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIA = joint.BodyA.invI
	joint.InvIB = joint.BodyB.invI

	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qA := MakeRot(aA)
	qB := MakeRot(aB)

	joint.RA = MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	joint.RB = MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))

	// Get the pulley axes.
	joint.UA = SubVV(AddVV(cA, joint.RA), joint.GroundAnchorA)
	joint.UB = SubVV(AddVV(cB, joint.RB), joint.GroundAnchorB)

	lengthA := joint.UA.Length()
	lengthB := joint.UB.Length()

	if lengthA > 10.0*LinearSlop {
		joint.UA.Mul(1.0 / lengthA)
	} else {
		joint.UA.SetZero()
	}

	if lengthB > 10.0*LinearSlop {
		joint.UB.Mul(1.0 / lengthB)
	} else {
		joint.UB.SetZero()
	}

	// Compute effective mass.
	ruA := CrossVV(joint.RA, joint.UA)
	ruB := CrossVV(joint.RB, joint.UB)

	mA := joint.InvMassA + joint.InvIA*ruA*ruA
	mB := joint.InvMassB + joint.InvIB*ruB*ruB

	joint.Mass = mA + joint.Ratio*joint.Ratio*mB

	if joint.Mass > 0.0 {
		joint.Mass = 1.0 / joint.Mass
	}

	if data.step.warmStarting {
		// Scale impulses to support variable time steps.
		joint.Impulse *= data.step.dtRatio

		// Warm starting.
		PA := MulFV(-joint.Impulse, joint.UA)
		PB := MulFV((-joint.Ratio * joint.Impulse), joint.UB)

		vA.Add(MulFV(joint.InvMassA, PA))
		wA += joint.InvIA * CrossVV(joint.RA, PA)
		vB.Add(MulFV(joint.InvMassB, PB))
		wB += joint.InvIB * CrossVV(joint.RB, PB)
	} else {
		joint.Impulse = 0.0
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *PulleyJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	vpA := AddVV(vA, CrossFV(wA, joint.RA))
	vpB := AddVV(vB, CrossFV(wB, joint.RB))

	Cdot := -DotVV(joint.UA, vpA) - joint.Ratio*DotVV(joint.UB, vpB)
	impulse := -joint.Mass * Cdot
	joint.Impulse += impulse

	PA := MulFV(-impulse, joint.UA)
	PB := MulFV(-joint.Ratio*impulse, joint.UB)
	vA.Add(MulFV(joint.InvMassA, PA))
	wA += joint.InvIA * CrossVV(joint.RA, PA)
	vB.Add(MulFV(joint.InvMassB, PB))
	wB += joint.InvIB * CrossVV(joint.RB, PB)

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *PulleyJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a

	qA := MakeRot(aA)
	qB := MakeRot(aB)

	rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))

	// Get the pulley axes.
	uA := SubVV(AddVV(cA, rA), joint.GroundAnchorA)
	uB := SubVV(AddVV(cB, rB), joint.GroundAnchorB)

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

	mA := joint.InvMassA + joint.InvIA*ruA*ruA
	mB := joint.InvMassB + joint.InvIB*ruB*ruB

	mass := mA + joint.Ratio*joint.Ratio*mB

	if mass > 0.0 {
		mass = 1.0 / mass
	}

	C := joint.Constant - lengthA - joint.Ratio*lengthB
	linearError := AbsF(C)

	impulse := -mass * C

	PA := MulFV(-impulse, uA)
	PB := MulFV(-joint.Ratio*impulse, uB)

	cA.Add(MulFV(joint.InvMassA, PA))
	aA += joint.InvIA * CrossVV(rA, PA)
	cB.Add(MulFV(joint.InvMassB, PB))
	aB += joint.InvIB * CrossVV(rB, PB)

	data.positions[joint.IndexA].c = cA
	data.positions[joint.IndexA].a = aA
	data.positions[joint.IndexB].c = cB
	data.positions[joint.IndexB].a = aB

	return linearError < LinearSlop
}

/// Dump joint to dmLog
func (joint *PulleyJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	Log("  b2PulleyJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.groundAnchorA.Set(%.15e, %.15e);\n", joint.GroundAnchorA.X, joint.GroundAnchorA.Y)
	Log("  jd.groundAnchorB.Set(%.15e, %.15e);\n", joint.GroundAnchorB.X, joint.GroundAnchorB.Y)
	Log("  jd.localAnchorA.Set(%.15e, %.15e);\n", joint.LocalAnchorA.X, joint.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15e, %.15e);\n", joint.LocalAnchorB.X, joint.LocalAnchorB.Y)
	Log("  jd.lengthA = %.15e;\n", joint.LengthA)
	Log("  jd.lengthB = %.15e;\n", joint.LengthB)
	Log("  jd.ratio = %.15e;\n", joint.Ratio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
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
	jd := new(GearJointDef)
	jd.Type = Joint_e_gearJoint
	jd.Joint1 = nil
	jd.Joint2 = nil
	jd.Ratio = 1.0
	return jd
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
	joint := new(GearJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.Joint1 = def.Joint1
	joint.Joint2 = def.Joint2

	joint.TypeA = joint.Joint1.GetType()
	joint.TypeB = joint.Joint2.GetType()

	//b2Assert(m_typeA == e_revoluteJoint || m_typeA == e_prismaticJoint);
	//b2Assert(m_typeB == e_revoluteJoint || m_typeB == e_prismaticJoint);

	var coordinateA, coordinateB float64

	// TODO_ERIN there might be some problem with the joint edges in b2Joint.

	joint.BodyC = joint.Joint1.GetBodyA()
	joint.BodyA = joint.Joint1.GetBodyB()

	// Get geometry of joint1
	xfA := joint.BodyA.xf
	aA := joint.BodyA.sweep.A
	xfC := joint.BodyC.xf
	aC := joint.BodyC.sweep.A

	if joint.TypeA == Joint_e_revoluteJoint {
		revolute := def.Joint1.(*RevoluteJoint)
		joint.LocalAnchorC = revolute.LocalAnchorA
		joint.LocalAnchorA = revolute.LocalAnchorB
		joint.ReferenceAngleA = revolute.ReferenceAngle
		joint.LocalAxisC.SetZero()

		coordinateA = aA - aC - joint.ReferenceAngleA
	} else {
		prismatic := def.Joint1.(*PrismaticJoint)
		joint.LocalAnchorC = prismatic.LocalAnchorA
		joint.LocalAnchorA = prismatic.LocalAnchorB
		joint.ReferenceAngleA = prismatic.ReferenceAngle
		joint.LocalAxisC = prismatic.LocalXAxisA

		pC := joint.LocalAnchorC
		pA := MulTRV(xfC.Q, AddVV(MulRV(xfA.Q, joint.LocalAnchorA), SubVV(xfA.P, xfC.P)))
		coordinateA = DotVV(SubVV(pA, pC), joint.LocalAxisC)
	}

	joint.BodyD = joint.Joint2.GetBodyA()
	joint.BodyB = joint.Joint2.GetBodyB()

	// Get geometry of joint2
	xfB := joint.BodyB.xf
	aB := joint.BodyB.sweep.A
	xfD := joint.BodyD.xf
	aD := joint.BodyD.sweep.A

	if joint.TypeB == Joint_e_revoluteJoint {
		revolute := def.Joint2.(*RevoluteJoint)
		joint.LocalAnchorD = revolute.LocalAnchorA
		joint.LocalAnchorB = revolute.LocalAnchorB
		joint.ReferenceAngleB = revolute.ReferenceAngle
		joint.LocalAxisD.SetZero()

		coordinateB = aB - aD - joint.ReferenceAngleB
	} else {
		prismatic := def.Joint2.(*PrismaticJoint)
		joint.LocalAnchorD = prismatic.LocalAnchorA
		joint.LocalAnchorB = prismatic.LocalAnchorB
		joint.ReferenceAngleB = prismatic.ReferenceAngle
		joint.LocalAxisD = prismatic.LocalXAxisA

		pD := joint.LocalAnchorD
		pB := MulTRV(xfD.Q, AddVV(MulRV(xfB.Q, joint.LocalAnchorB), AddVV(xfB.P, xfD.P)))
		coordinateB = DotVV(SubVV(pB, pD), joint.LocalAxisD)
	}

	joint.Ratio = def.Ratio

	joint.Constant = coordinateA + joint.Ratio*coordinateB

	joint.Impulse = 0.0
	return joint
}

func (joint *GearJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *GearJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

func (joint *GearJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := MulFV(joint.Impulse, joint.JvAC)
	return MulFV(inv_dt, P)
}

func (joint *GearJoint) GetReactionTorque(inv_dt float64) float64 {
	L := joint.Impulse * joint.JwA
	return inv_dt * L
}

/// Get the first joint.
func (joint *GearJoint) GetJoint1() IJoint {
	return joint.Joint1
}

/// Get the second joint.
func (joint *GearJoint) GetJoint2() IJoint {
	return joint.Joint2
}

/// Set/Get the gear ratio.
func (joint *GearJoint) SetRatio(ratio float64) {
	joint.Ratio = ratio
}

func (joint *GearJoint) GetRatio() float64 {
	return joint.Ratio
}

func (joint *GearJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.IndexC = joint.BodyC.islandIndex
	joint.IndexD = joint.BodyD.islandIndex
	joint.LcA = joint.BodyA.sweep.LocalCenter
	joint.LcB = joint.BodyB.sweep.LocalCenter
	joint.LcC = joint.BodyC.sweep.LocalCenter
	joint.LcD = joint.BodyD.sweep.LocalCenter
	joint.MA = joint.BodyA.invMass
	joint.MB = joint.BodyB.invMass
	joint.MC = joint.BodyC.invMass
	joint.MD = joint.BodyD.invMass
	joint.IA = joint.BodyA.invI
	joint.IB = joint.BodyB.invI
	joint.IC = joint.BodyC.invI
	joint.ID = joint.BodyD.invI

	//cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	//cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	//cC := data.positions[joint.IndexC].c
	aC := data.positions[joint.IndexC].a
	vC := data.velocities[joint.IndexC].v
	wC := data.velocities[joint.IndexC].w

	//cD := data.positions[joint.IndexD].c
	aD := data.positions[joint.IndexD].a
	vD := data.velocities[joint.IndexD].v
	wD := data.velocities[joint.IndexD].w

	qA, qB, qC, qD := MakeRot(aA), MakeRot(aB), MakeRot(aC), MakeRot(aD)

	joint.Mass = 0.0

	if joint.TypeA == Joint_e_revoluteJoint {
		joint.JvAC.SetZero()
		joint.JwA = 1.0
		joint.JwC = 1.0
		joint.Mass += joint.IA + joint.IC
	} else {
		u := MulRV(qC, joint.LocalAxisC)
		rC := MulRV(qC, SubVV(joint.LocalAnchorC, joint.LcC))
		rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LcA))
		joint.JvAC = u
		joint.JwC = CrossVV(rC, u)
		joint.JwA = CrossVV(rA, u)
		joint.Mass += joint.MC + joint.MA + joint.IC*joint.JwC*joint.JwC + joint.IA*joint.JwA*joint.JwA
	}

	if joint.TypeB == Joint_e_revoluteJoint {
		joint.JvBD.SetZero()
		joint.JwB = joint.Ratio
		joint.JwD = joint.Ratio
		joint.Mass += joint.Ratio * joint.Ratio * (joint.IB + joint.ID)
	} else {
		u := MulRV(qD, joint.LocalAxisD)
		rD := MulRV(qD, SubVV(joint.LocalAnchorD, joint.LcD))
		rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LcB))
		joint.JvBD = MulFV(joint.Ratio, u)
		joint.JwD = joint.Ratio * CrossVV(rD, u)
		joint.JwB = joint.Ratio * CrossVV(rB, u)
		joint.Mass += joint.Ratio*joint.Ratio*(joint.MD+joint.MB) + joint.ID*joint.JwD*joint.JwD + joint.IB*joint.JwB*joint.JwB
	}

	// Compute effective mass.
	joint.Mass = 0.0
	if joint.Mass > 0.0 {
		joint.Mass = 1.0 / joint.Mass
	}

	if data.step.warmStarting {
		vA.Add(MulFV(joint.MA*joint.Impulse, joint.JvAC))
		wA += joint.IA * joint.Impulse * joint.JwA
		vB.Add(MulFV(joint.MB*joint.Impulse, joint.JvBD))
		wB += joint.IB * joint.Impulse * joint.JwB
		vC.Sub(MulFV(joint.MC*joint.Impulse, joint.JvAC))
		wC -= joint.IC * joint.Impulse * joint.JwC
		vD.Sub(MulFV(joint.MD*joint.Impulse, joint.JvBD))
		wD -= joint.ID * joint.Impulse * joint.JwD
	} else {
		joint.Impulse = 0.0
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
	data.velocities[joint.IndexC].v = vC
	data.velocities[joint.IndexC].w = wC
	data.velocities[joint.IndexD].v = vD
	data.velocities[joint.IndexD].w = wD
}

func (joint *GearJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w
	vC := data.velocities[joint.IndexC].v
	wC := data.velocities[joint.IndexC].w
	vD := data.velocities[joint.IndexD].v
	wD := data.velocities[joint.IndexD].w

	Cdot := DotVV(joint.JvAC, SubVV(vA, vC)) + DotVV(joint.JvBD, SubVV(vB, vD))
	Cdot += (joint.JwA*wA - joint.JwC*wC) + (joint.JwB*wB - joint.JwD*wD)

	impulse := -joint.Mass * Cdot
	joint.Impulse += impulse

	vA.Add(MulFV(joint.MA*impulse, joint.JvAC))
	wA += joint.IA * impulse * joint.JwA
	vB.Add(MulFV(joint.MB*impulse, joint.JvBD))
	wB += joint.IB * impulse * joint.JwB
	vC.Sub(MulFV(joint.MC*impulse, joint.JvAC))
	wC -= joint.IC * impulse * joint.JwC
	vD.Sub(MulFV(joint.MD*impulse, joint.JvBD))
	wD -= joint.ID * impulse * joint.JwD

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
	data.velocities[joint.IndexC].v = vC
	data.velocities[joint.IndexC].w = wC
	data.velocities[joint.IndexD].v = vD
	data.velocities[joint.IndexD].w = wD
}

func (joint *GearJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	cC := data.positions[joint.IndexC].c
	aC := data.positions[joint.IndexC].a
	cD := data.positions[joint.IndexD].c
	aD := data.positions[joint.IndexD].a

	qA, qB, qC, qD := MakeRot(aA), MakeRot(aB), MakeRot(aC), MakeRot(aD)

	linearError := 0.0

	var coordinateA, coordinateB float64

	var JvAC, JvBD Vec2
	var JwA, JwB, JwC, JwD float64
	mass := 0.0

	if joint.TypeA == Joint_e_revoluteJoint {
		JvAC.SetZero()
		JwA = 1.0
		JwC = 1.0
		mass += joint.IA + joint.IC

		coordinateA = aA - aC - joint.ReferenceAngleA
	} else {
		u := MulRV(qC, joint.LocalAxisC)
		rC := MulRV(qC, SubVV(joint.LocalAnchorC, joint.LcC))
		rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LcA))
		JvAC = u
		JwC = CrossVV(rC, u)
		JwA = CrossVV(rA, u)
		mass += joint.MC + joint.MA + joint.IC*JwC*JwC + joint.IA*JwA*JwA

		pC := SubVV(joint.LocalAnchorC, joint.LcC)
		pA := MulTRV(qC, AddVV(rA, SubVV(cA, cC)))
		coordinateA = DotVV(SubVV(pA, pC), joint.LocalAxisC)
	}

	if joint.TypeB == Joint_e_revoluteJoint {
		JvBD.SetZero()
		JwB = joint.Ratio
		JwD = joint.Ratio
		mass += joint.Ratio * joint.Ratio * (joint.IB + joint.ID)

		coordinateB = aB - aD - joint.ReferenceAngleB
	} else {
		u := MulRV(qD, joint.LocalAxisD)
		rD := MulRV(qD, SubVV(joint.LocalAnchorD, joint.LcD))
		rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LcB))
		JvBD = MulFV(joint.Ratio, u)
		JwD = joint.Ratio * CrossVV(rD, u)
		JwB = joint.Ratio * CrossVV(rB, u)
		mass += joint.Ratio*joint.Ratio*(joint.MD+joint.MB) + joint.ID*JwD*JwD + joint.IB*JwB*JwB

		pD := SubVV(joint.LocalAnchorD, joint.LcD)
		pB := MulTRV(qD, AddVV(rB, SubVV(cB, cD)))
		coordinateB = DotVV(SubVV(pB, pD), joint.LocalAxisD)
	}

	C := (coordinateA + joint.Ratio*coordinateB) - joint.Constant

	impulse := 0.0
	if mass > 0.0 {
		impulse = -C / mass
	}

	cA.Add(MulFV(joint.MA*impulse, JvAC))
	aA += joint.IA * impulse * JwA
	cB.Add(MulFV(joint.MB*impulse, JvBD))
	aB += joint.IB * impulse * JwB
	cC.Sub(MulFV(joint.MC*impulse, JvAC))
	aC -= joint.IC * impulse * JwC
	cD.Sub(MulFV(joint.MD*impulse, JvBD))
	aD -= joint.ID * impulse * JwD

	data.positions[joint.IndexA].c = cA
	data.positions[joint.IndexA].a = aA
	data.positions[joint.IndexB].c = cB
	data.positions[joint.IndexB].a = aB
	data.positions[joint.IndexC].c = cC
	data.positions[joint.IndexC].a = aC
	data.positions[joint.IndexD].c = cD
	data.positions[joint.IndexD].a = aD

	// TODO_ERIN not implemented
	return linearError < LinearSlop
}

/// Dump joint to dmLog
func (joint *GearJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	index1 := joint.Joint1.GetIndex()
	index2 := joint.Joint2.GetIndex()

	Log("  b2GearJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.joint1 = joints[%d];\n", index1)
	Log("  jd.joint2 = joints[%d];\n", index2)
	Log("  jd.ratio = %.15e;\n", joint.Ratio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
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

func MakeWheelJointDef() WheelJointDef {
	jd := WheelJointDef{}
	jd.Type = Joint_e_wheelJoint
	jd.LocalAnchorA.SetZero()
	jd.LocalAnchorB.SetZero()
	jd.LocalAxisA.Set(1.0, 0.0)
	jd.EnableMotor = false
	jd.MaxMotorTorque = 0.0
	jd.MotorSpeed = 0.0
	jd.FrequencyHz = 2.0
	jd.DampingRatio = 0.7
	return jd
}

func NewWheelJointDef() *WheelJointDef {
	jd := MakeWheelJointDef()
	return &jd
}

/// Initialize the bodies, anchors, axis, and reference angle using the world
/// anchor and world axis.
func (jd *WheelJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2, axis Vec2) {
	jd.BodyA = bodyA
	jd.BodyB = bodyB
	jd.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	jd.LocalAnchorB = bodyB.GetLocalPoint(anchor)
	jd.LocalAxisA = bodyA.GetLocalVector(axis)
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
	joint := new(WheelJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.LocalAnchorA = def.LocalAnchorA
	joint.LocalAnchorB = def.LocalAnchorB
	joint.LocalXAxisA = def.LocalAxisA
	joint.LocalYAxisA = CrossFV(1.0, joint.LocalXAxisA)

	joint.Mass = 0.0
	joint.Impulse = 0.0
	joint.MotorMass = 0.0
	joint.MotorImpulse = 0.0
	joint.SpringMass = 0.0
	joint.SpringImpulse = 0.0

	joint.MaxMotorTorque = def.MaxMotorTorque
	joint.MotorSpeed = def.MotorSpeed
	joint.EnableMotor = def.EnableMotor

	joint.FrequencyHz = def.FrequencyHz
	joint.DampingRatio = def.DampingRatio

	joint.Bias = 0.0
	joint.Gamma = 0.0

	joint.Ax.SetZero()
	joint.Ay.SetZero()
	return joint
}

func (joint *WheelJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *WheelJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

func (joint *WheelJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt, AddVV(MulFV(joint.Impulse, joint.Ay), MulFV(joint.SpringImpulse, joint.Ax)))
}

func (joint *WheelJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.MotorImpulse
}

/// The local anchor point relative to bodyA's origin.
func (joint *WheelJoint) GetLocalAnchorA() Vec2 {
	return joint.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint *WheelJoint) GetLocalAnchorB() Vec2 {
	return joint.LocalAnchorB
}

/// The local joint axis relative to bodyA.
func (joint *WheelJoint) GetLocalAxisA() Vec2 {
	return joint.LocalXAxisA
}

/// Get the current joint translation, usually in meters.
func (joint *WheelJoint) GetJointTranslation() float64 {
	bA := joint.BodyA
	bB := joint.BodyB

	pA := bA.GetWorldPoint(joint.LocalAnchorA)
	pB := bB.GetWorldPoint(joint.LocalAnchorB)
	d := SubVV(pB, pA)
	axis := bA.GetWorldVector(joint.LocalXAxisA)

	translation := DotVV(d, axis)
	return translation
}

/// Get the current joint translation speed, usually in meters per second.
func (joint *WheelJoint) GetJointSpeed() float64 {
	wA := joint.BodyA.angularVelocity
	wB := joint.BodyB.angularVelocity
	return wB - wA
}

/// Is the joint motor enabled?
func (joint *WheelJoint) IsMotorEnabled() bool {
	return joint.EnableMotor
}

/// Enable/disable the joint motor.
func (joint *WheelJoint) SetEnableMotor(flag bool) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.EnableMotor = flag
}

/// Set the motor speed, usually in radians per second.
func (joint *WheelJoint) SetMotorSpeed(speed float64) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.MotorSpeed = speed
}

/// Get the motor speed, usually in radians per second.
func (joint *WheelJoint) GetMotorSpeed() float64 {
	return joint.MotorSpeed
}

/// Set/Get the maximum motor force, usually in N-m.
func (joint *WheelJoint) SetMaxMotorTorque(torque float64) {
	joint.BodyA.SetAwake(true)
	joint.BodyB.SetAwake(true)
	joint.MaxMotorTorque = torque
}

func (joint *WheelJoint) GetMaxMotorTorque() float64 {
	return joint.MaxMotorTorque
}

/// Get the current motor torque given the inverse time step, usually in N-m.
func (joint *WheelJoint) GetMotorTorque(inv_dt float64) float64 {
	return inv_dt * joint.MotorImpulse
}

/// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
func (joint *WheelJoint) SetSpringFrequencyHz(hz float64) {
	joint.FrequencyHz = hz
}

func (joint *WheelJoint) GetSpringFrequencyHz() float64 {
	return joint.FrequencyHz
}

/// Set/Get the spring damping ratio
func (joint *WheelJoint) SetSpringDampingRatio(ratio float64) {
	joint.DampingRatio = ratio
}

func (joint *WheelJoint) GetSpringDampingRatio() float64 {
	return joint.DampingRatio
}

func (joint *WheelJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterA = joint.BodyA.sweep.LocalCenter
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassA = joint.BodyA.invMass
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIA = joint.BodyA.invI
	joint.InvIB = joint.BodyB.invI

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qA, qB := MakeRot(aA), MakeRot(aB)

	// Compute the effective masses.
	rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))
	d := SubVV(AddVV(cB, rB), AddVV(cA, rA))

	// Point to line constraint
	{
		joint.Ay = MulRV(qA, joint.LocalYAxisA)
		joint.SAy = CrossVV(AddVV(d, rA), joint.Ay)
		joint.SBy = CrossVV(rB, joint.Ay)

		joint.Mass = mA + mB + iA*joint.SAy*joint.SAy + iB*joint.SBy*joint.SBy

		if joint.Mass > 0.0 {
			joint.Mass = 1.0 / joint.Mass
		}
	}

	// Spring constraint
	joint.SpringMass = 0.0
	joint.Bias = 0.0
	joint.Gamma = 0.0
	if joint.FrequencyHz > 0.0 {
		joint.Ax = MulRV(qA, joint.LocalXAxisA)
		joint.SAx = CrossVV(AddVV(d, rA), joint.Ax)
		joint.SBx = CrossVV(rB, joint.Ax)

		invMass := mA + mB + iA*joint.SAx*joint.SAx + iB*joint.SBx*joint.SBx

		if invMass > 0.0 {
			joint.SpringMass = 1.0 / invMass

			C := DotVV(d, joint.Ax)

			// Frequency
			omega := 2.0 * Pi * joint.FrequencyHz

			// Damping coefficient
			d := 2.0 * joint.SpringMass * joint.DampingRatio * omega

			// Spring stiffness
			k := joint.SpringMass * omega * omega

			// magic formulas
			h := data.step.dt
			joint.Gamma = h * (d + h*k)
			if joint.Gamma > 0.0 {
				joint.Gamma = 1.0 / joint.Gamma
			}

			joint.Bias = C * h * k * joint.Gamma

			joint.SpringMass = invMass + joint.Gamma
			if joint.SpringMass > 0.0 {
				joint.SpringMass = 1.0 / joint.SpringMass
			}
		}
	} else {
		joint.SpringImpulse = 0.0
	}

	// Rotational motor
	if joint.EnableMotor {
		joint.MotorMass = iA + iB
		if joint.MotorMass > 0.0 {
			joint.MotorMass = 1.0 / joint.MotorMass
		}
	} else {
		joint.MotorMass = 0.0
		joint.MotorImpulse = 0.0
	}

	if data.step.warmStarting {
		// Account for variable time step.
		joint.Impulse *= data.step.dtRatio
		joint.SpringImpulse *= data.step.dtRatio
		joint.MotorImpulse *= data.step.dtRatio

		P := AddVV(MulFV(joint.Impulse, joint.Ay), MulFV(joint.SpringImpulse, joint.Ax))
		LA := joint.Impulse*joint.SAy + joint.SpringImpulse*joint.SAx + joint.MotorImpulse
		LB := joint.Impulse*joint.SBy + joint.SpringImpulse*joint.SBx + joint.MotorImpulse

		vA.Sub(MulFV(joint.InvMassA, P))
		wA -= joint.InvIA * LA

		vB.Add(MulFV(joint.InvMassB, P))
		wB += joint.InvIB * LB
	} else {
		joint.Impulse = 0.0
		joint.SpringImpulse = 0.0
		joint.MotorImpulse = 0.0
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *WheelJoint) SolveVelocityConstraints(data *solverData) {
	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	// Solve spring constraint
	{
		Cdot := DotVV(joint.Ax, SubVV(vB, vA)) + joint.SBx*wB - joint.SAx*wA
		impulse := -joint.SpringMass * (Cdot + joint.Bias + joint.Gamma*joint.SpringImpulse)
		joint.SpringImpulse += impulse

		P := MulFV(impulse, joint.Ax)
		LA := impulse * joint.SAx
		LB := impulse * joint.SBx

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	}

	// Solve rotational motor constraint
	{
		Cdot := wB - wA - joint.MotorSpeed
		impulse := -joint.MotorMass * Cdot

		oldImpulse := joint.MotorImpulse
		maxImpulse := data.step.dt * joint.MaxMotorTorque
		joint.MotorImpulse = ClampF(joint.MotorImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.MotorImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve point to line constraint
	{
		Cdot := DotVV(joint.Ay, SubVV(vB, vA)) + joint.SBy*wB - joint.SAy*wA
		impulse := -joint.Mass * Cdot
		joint.Impulse += impulse

		P := MulFV(impulse, joint.Ay)
		LA := impulse * joint.SAy
		LB := impulse * joint.SBy

		vA.Sub(MulFV(mA, P))
		wA -= iA * LA

		vB.Add(MulFV(mB, P))
		wB += iB * LB
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *WheelJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a

	qA, qB := MakeRot(aA), MakeRot(aB)

	rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))
	d := AddVV(SubVV(cB, cA), SubVV(rB, rA))

	ay := MulRV(qA, joint.LocalYAxisA)

	sAy := CrossVV(AddVV(d, rA), ay)
	sBy := CrossVV(rB, ay)

	C := DotVV(d, ay)

	k := joint.InvMassA + joint.InvMassB + joint.InvIA*joint.SAy*joint.SAy + joint.InvIB*joint.SBy*joint.SBy

	var impulse float64
	if k != 0.0 {
		impulse = -C / k
	} else {
		impulse = 0.0
	}

	P := MulFV(impulse, ay)
	LA := impulse * sAy
	LB := impulse * sBy

	cA.Sub(MulFV(joint.InvMassA, P))
	aA -= joint.InvIA * LA
	cB.Add(MulFV(joint.InvMassB, P))
	aB += joint.InvIB * LB

	data.positions[joint.IndexA].c = cA
	data.positions[joint.IndexA].a = aA
	data.positions[joint.IndexB].c = cB
	data.positions[joint.IndexB].a = aB

	return AbsF(C) <= LinearSlop
}

/// Dump to b2Log
func (joint *WheelJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	Log("  b2WheelJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15e, %.15e);\n", joint.LocalAnchorA.X, joint.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15e, %.15e);\n", joint.LocalAnchorB.X, joint.LocalAnchorB.Y)
	Log("  jd.localAxisA.Set(%.15e, %.15e);\n", joint.LocalXAxisA.X, joint.LocalXAxisA.Y)
	Log("  jd.enableMotor = bool(%t);\n", joint.EnableMotor)
	Log("  jd.motorSpeed = %.15e;\n", joint.MotorSpeed)
	Log("  jd.maxMotorTorque = %.15e;\n", joint.MaxMotorTorque)
	Log("  jd.frequencyHz = %.15e;\n", joint.FrequencyHz)
	Log("  jd.dampingRatio = %.15e;\n", joint.DampingRatio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
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
	jd := new(WeldJointDef)
	jd.Type = Joint_e_weldJoint
	jd.LocalAnchorA.Set(0.0, 0.0)
	jd.LocalAnchorB.Set(0.0, 0.0)
	jd.ReferenceAngle = 0.0
	jd.FrequencyHz = 0.0
	jd.DampingRatio = 0.0
	return jd
}

/// Initialize the bodies, anchors, and reference angle using a world
/// anchor point.
func (jd *WeldJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2) {
	jd.BodyA = bodyA
	jd.BodyB = bodyB
	jd.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	jd.LocalAnchorB = bodyB.GetLocalPoint(anchor)
	jd.ReferenceAngle = bodyB.GetAngle() - bodyA.GetAngle()
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
	joint := new(WeldJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.LocalAnchorA = def.LocalAnchorA
	joint.LocalAnchorB = def.LocalAnchorB
	joint.ReferenceAngle = def.ReferenceAngle
	joint.FrequencyHz = def.FrequencyHz
	joint.DampingRatio = def.DampingRatio

	joint.Impulse.SetZero()
	return joint
}

func (joint *WeldJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *WeldJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

func (joint *WeldJoint) GetReactionForce(inv_dt float64) Vec2 {
	P := Vec2{joint.Impulse.X, joint.Impulse.Y}
	return MulFV(inv_dt, P)
}

func (joint *WeldJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.Impulse.Z
}

/// The local anchor point relative to bodyA's origin.
func (joint *WeldJoint) GetLocalAnchorA() Vec2 {
	return joint.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint *WeldJoint) GetLocalAnchorB() Vec2 {
	return joint.LocalAnchorB
}

/// Get the reference angle.
func (joint *WeldJoint) GetReferenceAngle() float64 {
	return joint.ReferenceAngle
}

/// Set/get frequency in Hz.
func (joint *WeldJoint) SetFrequency(hz float64) {
	joint.FrequencyHz = hz
}

func (joint *WeldJoint) GetFrequency() float64 {
	return joint.FrequencyHz
}

/// Set/get damping ratio.
func (joint *WeldJoint) SetDampingRatio(ratio float64) {
	joint.DampingRatio = ratio
}

func (joint *WeldJoint) GetDampingRatio() float64 {
	return joint.DampingRatio
}

func (joint *WeldJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterA = joint.BodyA.sweep.LocalCenter
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassA = joint.BodyA.invMass
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIA = joint.BodyA.invI
	joint.InvIB = joint.BodyB.invI

	//cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	//cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qA, qB := MakeRot(aA), MakeRot(aB)

	joint.RA = MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	joint.RB = MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	var K Mat33
	K.Ex.X = mA + mB + joint.RA.Y*joint.RA.Y*iA + joint.RB.Y*joint.RB.Y*iB
	K.Ex.X = mA + mB + joint.RA.Y*joint.RA.Y*iA + joint.RB.Y*joint.RB.Y*iB
	K.Ey.X = -joint.RA.Y*joint.RA.X*iA - joint.RB.Y*joint.RB.X*iB
	K.Ez.X = -joint.RA.Y*iA - joint.RB.Y*iB
	K.Ex.Y = K.Ey.X
	K.Ey.Y = mA + mB + joint.RA.X*joint.RA.X*iA + joint.RB.X*joint.RB.X*iB
	K.Ez.Y = joint.RA.X*iA + joint.RB.X*iB
	K.Ex.Z = K.Ez.X
	K.Ey.Z = K.Ez.Y
	K.Ez.Z = iA + iB

	if joint.FrequencyHz > 0.0 {
		K.GetInverse22(&joint.Mass)

		invM := iA + iB
		m := 0.0
		if invM > 0.0 {
			m = 1.0 / invM
		}

		C := aB - aA - joint.ReferenceAngle

		// Frequency
		omega := 2.0 * Pi * joint.FrequencyHz

		// Damping coefficient
		d := 2.0 * m * joint.DampingRatio * omega

		// Spring stiffness
		k := m * omega * omega

		// magic formulas
		h := data.step.dt
		joint.Gamma = h * (d + h*k)
		joint.Gamma = 0.0
		if joint.Gamma != 0.0 {
			joint.Gamma = 1.0 / joint.Gamma
		}
		joint.Bias = C * h * k * joint.Gamma

		invM += joint.Gamma
		joint.Mass.Ez.Z = 0.0
		if invM != 0.0 {
			invM = 1.0 / invM
		}
	} else {
		K.GetSymInverse33(&joint.Mass)
		joint.Gamma = 0.0
		joint.Bias = 0.0
	}

	if data.step.warmStarting {
		// Scale impulses to support a variable time step.
		joint.Impulse.Mul(data.step.dtRatio)

		P := Vec2{joint.Impulse.X, joint.Impulse.Y}

		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(joint.RA, P) + joint.Impulse.Z)

		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(joint.RB, P) + joint.Impulse.Z)
	} else {
		joint.Impulse.SetZero()
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *WeldJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	if joint.FrequencyHz > 0.0 {
		Cdot2 := wB - wA

		impulse2 := -joint.Mass.Ez.Z * (Cdot2 + joint.Bias + joint.Gamma*joint.Impulse.Z)
		joint.Impulse.Z += impulse2

		wA -= iA * impulse2
		wB += iB * impulse2

		Cdot1 := SubVV(AddVV(vB, CrossFV(wB, joint.RB)), AddVV(vA, CrossFV(wA, joint.RA)))

		impulse1 := MulM3V(joint.Mass, Cdot1)
		impulse1 = impulse1.Minus()
		joint.Impulse.X += impulse1.X
		joint.Impulse.Y += impulse1.Y

		P := impulse1

		vA.Sub(MulFV(mA, P))
		wA -= iA * CrossVV(joint.RA, P)

		vB.Add(MulFV(mB, P))
		wB += iB * CrossVV(joint.RB, P)
	} else {
		Cdot1 := SubVV(AddVV(vB, CrossFV(wB, joint.RB)), AddVV(vA, CrossFV(wA, joint.RA)))
		Cdot2 := wB - wA
		Cdot := Vec3{Cdot1.X, Cdot1.Y, Cdot2}

		impulse := MulM3V3(joint.Mass, Cdot)
		impulse = impulse.Minus()
		joint.Impulse.Add(impulse)

		P := Vec2{impulse.X, impulse.Y}

		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(joint.RA, P) + impulse.Z)

		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(joint.RB, P) + impulse.Z)
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *WeldJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a

	qA, qB := MakeRot(aA), MakeRot(aB)

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))

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

	if joint.FrequencyHz > 0.0 {
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
		C2 := aB - aA - joint.ReferenceAngle

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

	data.positions[joint.IndexA].c = cA
	data.positions[joint.IndexA].a = aA
	data.positions[joint.IndexB].c = cB
	data.positions[joint.IndexB].a = aB

	return positionError <= LinearSlop && angularError <= AngularSlop
}

/// Dump to b2Log
func (joint *WeldJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	Log("  b2WeldJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15e, %.15e);\n", joint.LocalAnchorA.X, joint.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15e, %.15e);\n", joint.LocalAnchorB.X, joint.LocalAnchorB.Y)
	Log("  jd.referenceAngle = %.15e;\n", joint.ReferenceAngle)
	Log("  jd.frequencyHz = %.15e;\n", joint.FrequencyHz)
	Log("  jd.dampingRatio = %.15e;\n", joint.DampingRatio)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
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
	jd := new(FrictionJointDef)
	jd.Type = Joint_e_frictionJoint
	jd.LocalAnchorA.SetZero()
	jd.LocalAnchorB.SetZero()
	jd.MaxForce = 0.0
	jd.MaxTorque = 0.0
	return jd
}

/// Initialize the bodies, anchors, axis, and reference angle using the world
/// anchor and world axis.
func (jd *FrictionJointDef) Initialize(bodyA *Body, bodyB *Body, anchor Vec2) {
	jd.BodyA = bodyA
	jd.BodyB = bodyB
	jd.LocalAnchorA = bodyA.GetLocalPoint(anchor)
	jd.LocalAnchorB = bodyB.GetLocalPoint(anchor)
}

/// Friction joint. jd is used for top-down friction.
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
	joint := new(FrictionJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.LocalAnchorA = def.LocalAnchorA
	joint.LocalAnchorB = def.LocalAnchorB

	joint.LinearImpulse.SetZero()
	joint.AngularImpulse = 0.0

	joint.MaxForce = def.MaxForce
	joint.MaxTorque = def.MaxTorque
	return joint
}

func (joint *FrictionJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *FrictionJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

func (joint *FrictionJoint) GetReactionForce(inv_dt float64) Vec2 {
	return MulFV(inv_dt, joint.LinearImpulse)
}

func (joint *FrictionJoint) GetReactionTorque(inv_dt float64) float64 {
	return inv_dt * joint.AngularImpulse
}

/// The local anchor point relative to bodyA's origin.
func (joint *FrictionJoint) GetLocalAnchorA() Vec2 {
	return joint.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint *FrictionJoint) GetLocalAnchorB() Vec2 {
	return joint.LocalAnchorB
}

/// Set the maximum friction force in N.
func (joint *FrictionJoint) SetMaxForce(force float64) {
	joint.MaxForce = force
}

/// Get the maximum friction force in N.
func (joint *FrictionJoint) GetMaxForce() float64 {
	return joint.MaxForce
}

/// Set the maximum friction torque in N*m.
func (joint *FrictionJoint) SetMaxTorque(torque float64) {
	joint.MaxTorque = torque
}

/// Get the maximum friction torque in N*m.
func (joint *FrictionJoint) GetMaxTorque() float64 {
	return joint.MaxTorque
}

func (joint *FrictionJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterA = joint.BodyA.sweep.LocalCenter
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassA = joint.BodyA.invMass
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIA = joint.BodyA.invI
	joint.InvIB = joint.BodyB.invI

	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qA, qB := MakeRot(aA), MakeRot(aB)

	// Compute the effective mass matrix.
	joint.RA = MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	joint.RB = MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	var K Mat22
	K.Ex.X = mA + mB + iA*joint.RA.Y*joint.RA.Y + iB*joint.RB.Y*joint.RB.Y
	K.Ex.Y = -iA*joint.RA.X*joint.RA.Y - iB*joint.RB.X*joint.RB.Y
	K.Ey.X = K.Ex.Y
	K.Ey.Y = mA + mB + iA*joint.RA.X*joint.RA.X + iB*joint.RB.X*joint.RB.X

	joint.LinearMass = K.GetInverse()

	joint.AngularMass = iA + iB
	if joint.AngularMass > 0.0 {
		joint.AngularMass = 1.0 / joint.AngularMass
	}

	if data.step.warmStarting {
		// Scale impulses to support a variable time step.
		joint.LinearImpulse.Mul(data.step.dtRatio)
		joint.AngularImpulse *= data.step.dtRatio

		P := Vec2{joint.LinearImpulse.X, joint.LinearImpulse.Y}
		vA.Sub(MulFV(mA, P))
		wA -= iA * (CrossVV(joint.RA, P) + joint.AngularImpulse)
		vB.Add(MulFV(mB, P))
		wB += iB * (CrossVV(joint.RB, P) + joint.AngularImpulse)
	} else {
		joint.LinearImpulse.SetZero()
		joint.AngularImpulse = 0.0
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *FrictionJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	mA := joint.InvMassA
	mB := joint.InvMassB
	iA := joint.InvIA
	iB := joint.InvIB

	h := data.step.dt

	// Solve angular friction
	{
		Cdot := wB - wA
		impulse := -joint.AngularMass * Cdot

		oldImpulse := joint.AngularImpulse
		maxImpulse := h * joint.MaxTorque
		joint.AngularImpulse = ClampF(joint.AngularImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.AngularImpulse - oldImpulse

		wA -= iA * impulse
		wB += iB * impulse
	}

	// Solve linear friction
	{
		Cdot := SubVV(AddVV(vB, CrossFV(wB, joint.RB)), AddVV(vA, CrossFV(wA, joint.RA)))

		impulse := MulMV(joint.LinearMass, Cdot)
		impulse = impulse.Minus()
		oldImpulse := joint.LinearImpulse
		joint.LinearImpulse.Add(impulse)

		maxImpulse := h * joint.MaxForce

		if joint.LinearImpulse.LengthSquared() > maxImpulse*maxImpulse {
			joint.LinearImpulse.Normalize()
			joint.LinearImpulse.Mul(maxImpulse)
		}

		impulse = SubVV(joint.LinearImpulse, oldImpulse)

		vA.Sub(MulFV(mA, impulse))
		wA -= iA * CrossVV(joint.RA, impulse)

		vB.Add(MulFV(mB, impulse))
		wB += iB * CrossVV(joint.RB, impulse)
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *FrictionJoint) SolvePositionConstraints(data *solverData) bool {
	return true
}

/// Dump joint to b2Log
func (joint *FrictionJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	Log("  b2FrictionJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15e, %.15e);\n", joint.LocalAnchorA.X, joint.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15e, %.15e);\n", joint.LocalAnchorB.X, joint.LocalAnchorB.Y)
	Log("  jd.maxForce = %.15e;\n", joint.MaxForce)
	Log("  jd.maxTorque = %.15e;\n", joint.MaxTorque)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
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
	jd := new(RopeJointDef)
	jd.Type = Joint_e_ropeJoint
	jd.LocalAnchorA.Set(-1.0, 0.0)
	jd.LocalAnchorB.Set(1.0, 0.0)
	jd.MaxLength = 0.0
	return jd
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
	joint := new(RopeJoint)
	joint.Type = def.Type
	joint.BodyA = def.BodyA
	joint.BodyB = def.BodyB
	joint.CollideConnected = def.CollideConnected
	joint.UserData = def.UserData

	joint.LocalAnchorA = def.LocalAnchorA
	joint.LocalAnchorB = def.LocalAnchorB

	joint.MaxLength = def.MaxLength

	joint.Mass = 0.0
	joint.Impulse = 0.0
	joint.State = LimitState_e_inactiveLimit
	joint.Length = 0.0
	return joint
}

func (joint *RopeJoint) GetAnchorA() Vec2 {
	return joint.BodyA.GetWorldPoint(joint.LocalAnchorA)
}

func (joint *RopeJoint) GetAnchorB() Vec2 {
	return joint.BodyB.GetWorldPoint(joint.LocalAnchorB)
}

func (joint *RopeJoint) GetReactionForce(inv_dt float64) Vec2 {
	F := MulFV((inv_dt * joint.Impulse), joint.U)
	return F
}

func (joint *RopeJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

/// The local anchor point relative to bodyA's origin.
func (joint *RopeJoint) GetLocalAnchorA() Vec2 {
	return joint.LocalAnchorA
}

/// The local anchor point relative to bodyB's origin.
func (joint *RopeJoint) GetLocalAnchorB() Vec2 {
	return joint.LocalAnchorB
}

/// Set/Get the maximum length of the rope.
func (joint *RopeJoint) SetMaxLength(length float64) {
	joint.MaxLength = length
}

func (joint *RopeJoint) GetMaxLength() float64 {
	return joint.MaxLength
}

func (joint *RopeJoint) GetLimitState() LimitState {
	return joint.State
}

func (joint *RopeJoint) InitVelocityConstraints(data *solverData) {
	joint.IndexA = joint.BodyA.islandIndex
	joint.IndexB = joint.BodyB.islandIndex
	joint.LocalCenterA = joint.BodyA.sweep.LocalCenter
	joint.LocalCenterB = joint.BodyB.sweep.LocalCenter
	joint.InvMassA = joint.BodyA.invMass
	joint.InvMassB = joint.BodyB.invMass
	joint.InvIA = joint.BodyA.invI
	joint.InvIB = joint.BodyB.invI

	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w

	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	qA, qB := MakeRot(aA), MakeRot(aB)

	joint.RA = MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	joint.RB = MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))
	joint.U = SubVV(AddVV(cB, joint.RB), AddVV(cA, joint.RA))

	joint.Length = joint.U.Length()

	C := joint.Length - joint.MaxLength
	if C > 0.0 {
		joint.State = LimitState_e_atUpperLimit
	} else {
		joint.State = LimitState_e_inactiveLimit
	}

	if joint.Length > LinearSlop {
		joint.U.Mul(1.0 / joint.Length)
	} else {
		joint.U.SetZero()
		joint.Mass = 0.0
		joint.Impulse = 0.0
		return
	}

	// Compute effective mass.
	crA := CrossVV(joint.RA, joint.U)
	crB := CrossVV(joint.RB, joint.U)
	invMass := joint.InvMassA + joint.InvIA*crA*crA + joint.InvMassB + joint.InvIB*crB*crB

	joint.Mass = 0.0
	if invMass != 0.0 {
		joint.Mass = 1.0 / invMass
	}

	if data.step.warmStarting {
		// Scale the impulse to support a variable time step.
		joint.Impulse *= data.step.dtRatio

		P := MulFV(joint.Impulse, joint.U)
		vA.Sub(MulFV(joint.InvMassA, P))
		wA -= joint.InvIA * CrossVV(joint.RA, P)
		vB.Add(MulFV(joint.InvMassB, P))
		wB += joint.InvIB * CrossVV(joint.RB, P)
	} else {
		joint.Impulse = 0.0
	}

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *RopeJoint) SolveVelocityConstraints(data *solverData) {
	vA := data.velocities[joint.IndexA].v
	wA := data.velocities[joint.IndexA].w
	vB := data.velocities[joint.IndexB].v
	wB := data.velocities[joint.IndexB].w

	// Cdot = dot(u, v + cross(w, r))
	vpA := AddVV(vA, CrossFV(wA, joint.RA))
	vpB := AddVV(vB, CrossFV(wB, joint.RB))
	C := joint.Length - joint.MaxLength
	Cdot := DotVV(joint.U, SubVV(vpB, vpA))

	// Predictive constraint.
	if C < 0.0 {
		Cdot += data.step.inv_dt * C
	}

	impulse := -joint.Mass * Cdot
	oldImpulse := joint.Impulse
	joint.Impulse = MinF(0.0, joint.Impulse+impulse)
	impulse = joint.Impulse - oldImpulse

	P := MulFV(impulse, joint.U)
	vA.Sub(MulFV(joint.InvMassA, P))
	wA -= joint.InvIA * CrossVV(joint.RA, P)
	vB.Add(MulFV(joint.InvMassB, P))
	wB += joint.InvIB * CrossVV(joint.RB, P)

	data.velocities[joint.IndexA].v = vA
	data.velocities[joint.IndexA].w = wA
	data.velocities[joint.IndexB].v = vB
	data.velocities[joint.IndexB].w = wB
}

func (joint *RopeJoint) SolvePositionConstraints(data *solverData) bool {
	cA := data.positions[joint.IndexA].c
	aA := data.positions[joint.IndexA].a
	cB := data.positions[joint.IndexB].c
	aB := data.positions[joint.IndexB].a

	qA, qB := MakeRot(aA), MakeRot(aB)

	rA := MulRV(qA, SubVV(joint.LocalAnchorA, joint.LocalCenterA))
	rB := MulRV(qB, SubVV(joint.LocalAnchorB, joint.LocalCenterB))
	u := SubVV(AddVV(cB, rB), AddVV(cA, rA))

	length := u.Normalize()
	C := length - joint.MaxLength

	C = ClampF(C, 0.0, MaxLinearCorrection)

	impulse := -joint.Mass * C
	P := MulFV(impulse, u)

	cA.Sub(MulFV(joint.InvMassA, P))
	aA -= joint.InvIA * CrossVV(rA, P)
	cB.Add(MulFV(joint.InvMassB, P))
	aB += joint.InvIB * CrossVV(rB, P)

	data.positions[joint.IndexA].c = cA
	data.positions[joint.IndexA].a = aA
	data.positions[joint.IndexB].c = cB
	data.positions[joint.IndexB].a = aB

	return length-joint.MaxLength < LinearSlop
}

/// Dump joint to b2Log
func (joint *RopeJoint) Dump() {
	indexA := joint.BodyA.islandIndex
	indexB := joint.BodyB.islandIndex

	Log("  b2RopeJointDef jd;\n")
	Log("  jd.bodyA = bodies[%d];\n", indexA)
	Log("  jd.bodyB = bodies[%d];\n", indexB)
	Log("  jd.collideConnected = bool(%t);\n", joint.CollideConnected)
	Log("  jd.localAnchorA.Set(%.15e, %.15e);\n", joint.LocalAnchorA.X, joint.LocalAnchorA.Y)
	Log("  jd.localAnchorB.Set(%.15e, %.15e);\n", joint.LocalAnchorB.X, joint.LocalAnchorB.Y)
	Log("  jd.maxLength = %.15e;\n", joint.MaxLength)
	Log("  joints[%d] = m_world->CreateJoint(&jd);\n", joint.Index)
}
