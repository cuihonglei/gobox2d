package box2d

type Island struct {
	Listener IContactListener

	Bodies   []*Body
	Contacts []IContact
	Joints   []IJoint

	Positions  []position
	Velocities []velocity

	BodyCount    int
	JointCount   int
	ContactCount int

	BodyCapacity    int
	ContactCapacity int
	JointCapacity   int
}

func NewIsland(bodyCapacity int, contactCapacity int, jointCapacity int, listener IContactListener) *Island {
	this := new(Island)

	this.BodyCapacity = bodyCapacity
	this.ContactCapacity = contactCapacity
	this.JointCapacity = jointCapacity

	this.Listener = listener
	this.Bodies = make([]*Body, this.BodyCapacity, this.BodyCapacity)
	this.Contacts = make([]IContact, this.ContactCapacity, this.ContactCapacity)
	this.Joints = make([]IJoint, this.JointCapacity, this.JointCapacity)

	this.Positions = make([]position, this.BodyCapacity, this.BodyCapacity)
	this.Velocities = make([]velocity, this.BodyCapacity, this.BodyCapacity)

	return this
}

func (this *Island) Clear() {
	this.BodyCount = 0
	this.JointCount = 0
	this.ContactCount = 0
}

func (this *Island) Solve(profile *Profile, step *timeStep, gravity Vec2, allowSleep bool) {
	timer := MakeTimer()

	h := step.dt

	// Integrate velocities and apply damping. Initialize the body state.
	for i := 0; i < this.BodyCount; i++ {
		b := this.Bodies[i]

		c := b.sweep.C
		a := b.sweep.A
		v := b.linearVelocity
		w := b.angularVelocity

		// Store positions for continuous collision.
		b.sweep.C0 = b.sweep.C
		b.sweep.A0 = b.sweep.A

		if b.xtype == DynamicBody {

			// Integrate velocities.
			v.Add(MulFV(h, AddVV(MulFV(b.gravityScale, gravity), MulFV(b.invMass, b.force))))
			w += h * b.invI * b.torque

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Taylor expansion:
			// v2 = (1.0f - c * dt) * v1
			v.Mul(ClampF(1.0-h*b.linearDamping, 0.0, 1.0))
			w *= ClampF(1.0-h*b.angularDamping, 0.0, 1.0)

		}

		this.Positions[i].c = c
		this.Positions[i].a = a
		this.Velocities[i].v = v
		this.Velocities[i].w = w
	}

	timer.Reset()

	// Solver data
	var solverData solverData
	solverData.step = *step
	solverData.positions = this.Positions
	solverData.velocities = this.Velocities

	// Initialize velocity constraints.
	var contactSolverDef ContactSolverDef
	contactSolverDef.Step = *step
	contactSolverDef.Contacts = this.Contacts
	contactSolverDef.Count = this.ContactCount
	contactSolverDef.Positions = this.Positions
	contactSolverDef.Velocities = this.Velocities

	contactSolver := NewContactSolver(&contactSolverDef)
	contactSolver.InitializeVelocityConstraints()

	if step.warmStarting {
		contactSolver.WarmStart()
	}

	for i := 0; i < this.JointCount; i++ {
		this.Joints[i].InitVelocityConstraints(&solverData)
	}

	profile.solveInit = timer.GetMilliseconds()

	// Solve velocity constraints
	timer.Reset()
	for i := 0; i < step.velocityIterations; i++ {
		for j := 0; j < this.JointCount; j++ {
			this.Joints[j].SolveVelocityConstraints(&solverData)
		}

		contactSolver.SolveVelocityConstraints()
	}

	// Store impulses for warm starting
	contactSolver.StoreImpulses()
	profile.solveVelocity = timer.GetMilliseconds()

	// Integrate positions
	for i := 0; i < this.BodyCount; i++ {
		c := this.Positions[i].c
		a := this.Positions[i].a
		v := this.Velocities[i].v
		w := this.Velocities[i].w

		// Check for large velocities
		translation := MulFV(h, v)
		if DotVV(translation, translation) > MaxTranslationSquared {
			ratio := MaxTranslation / translation.Length()
			v.Mul(ratio)
		}

		rotation := h * w
		if rotation*rotation > MaxRotationSquared {
			ratio := MaxRotation / AbsF(rotation)
			w *= ratio
		}

		// Integrate
		c.Add(MulFV(h, v))
		a += h * w

		this.Positions[i].c = c
		this.Positions[i].a = a
		this.Velocities[i].v = v
		this.Velocities[i].w = w
	}

	// Solve position constraints
	timer.Reset()
	positionSolved := false
	for i := 0; i < step.positionIterations; i++ {
		contactsOkay := contactSolver.SolvePositionConstraints()

		jointsOkay := true
		for i := 0; i < this.JointCount; i++ {
			jointOkay := this.Joints[i].SolvePositionConstraints(&solverData)
			jointsOkay = jointsOkay && jointOkay
		}

		if contactsOkay && jointsOkay {
			// Exit early if the position errors are small.
			positionSolved = true
			break
		}
	}

	// Copy state buffers back to the bodies
	for i := 0; i < this.BodyCount; i++ {
		body := this.Bodies[i]
		body.sweep.C = this.Positions[i].c
		body.sweep.A = this.Positions[i].a
		body.linearVelocity = this.Velocities[i].v
		body.angularVelocity = this.Velocities[i].w
		body.synchronizeTransform()
	}

	profile.solvePosition = timer.GetMilliseconds()

	this.Report(contactSolver.VelocityConstraints)

	if allowSleep {
		minSleepTime := MaxFloat

		linTolSqr := LinearSleepTolerance * LinearSleepTolerance
		angTolSqr := AngularSleepTolerance * AngularSleepTolerance

		for i := 0; i < this.BodyCount; i++ {
			b := this.Bodies[i]
			if b.GetType() == StaticBody {
				continue
			}

			if (b.flags&body_e_autoSleepFlag) == 0 ||
				b.angularVelocity*b.angularVelocity > angTolSqr ||
				DotVV(b.linearVelocity, b.linearVelocity) > linTolSqr {
				b.sleepTime = 0.0
				minSleepTime = 0.0
			} else {
				b.sleepTime += h
				minSleepTime = MinF(minSleepTime, b.sleepTime)
			}
		}

		if minSleepTime >= TimeToSleep && positionSolved {
			for i := 0; i < this.BodyCount; i++ {
				b := this.Bodies[i]
				b.SetAwake(false)
			}
		}
	}
}

func (this *Island) SolveTOI(subStep *timeStep, toiIndexA int, toiIndexB int) {
	// Initialize the body state.
	for i := 0; i < this.BodyCount; i++ {
		b := this.Bodies[i]
		this.Positions[i].c = b.sweep.C
		this.Positions[i].a = b.sweep.A
		this.Velocities[i].v = b.linearVelocity
		this.Velocities[i].w = b.angularVelocity
	}

	var contactSolverDef ContactSolverDef
	contactSolverDef.Contacts = this.Contacts
	contactSolverDef.Count = this.ContactCount
	contactSolverDef.Step = *subStep
	contactSolverDef.Positions = this.Positions
	contactSolverDef.Velocities = this.Velocities
	contactSolver := NewContactSolver(&contactSolverDef)

	// Solve position constraints.
	for i := 0; i < subStep.positionIterations; i++ {
		contactsOkay := contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB)
		if contactsOkay {
			break
		}
	}

	//#if 0
	//	// Is the new position really safe?
	//	for (int i = 0; i < m_contactCount; ++i)
	//	{
	//		b2Contact* c = m_contacts[i];
	//		b2Fixture* fA = c->GetFixtureA();
	//		b2Fixture* fB = c->GetFixtureB();

	//		b2Body* bA = fA->GetBody();
	//		b2Body* bB = fB->GetBody();

	//		int indexA = c->GetChildIndexA();
	//		int indexB = c->GetChildIndexB();

	//		b2DistanceInput input;
	//		input.proxyA.Set(fA->GetShape(), indexA);
	//		input.proxyB.Set(fB->GetShape(), indexB);
	//		input.transformA = bA->GetTransform();
	//		input.transformB = bB->GetTransform();
	//		input.useRadii = false;

	//		b2DistanceOutput output;
	//		b2SimplexCache cache;
	//		cache.count = 0;
	//		b2Distance(&output, &cache, &input);

	//		if (output.distance == 0 || cache.count == 3)
	//		{
	//			cache.count += 0;
	//		}
	//	}
	//#endif

	// Leap of faith to new safe state.
	this.Bodies[toiIndexA].sweep.C0 = this.Positions[toiIndexA].c
	this.Bodies[toiIndexA].sweep.A0 = this.Positions[toiIndexA].a
	this.Bodies[toiIndexB].sweep.C0 = this.Positions[toiIndexB].c
	this.Bodies[toiIndexB].sweep.A0 = this.Positions[toiIndexB].a

	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.
	contactSolver.InitializeVelocityConstraints()

	// Solve velocity constraints.
	for i := 0; i < subStep.velocityIterations; i++ {
		contactSolver.SolveVelocityConstraints()
	}

	// Don't store the TOI contact forces for warm starting
	// because they can be quite large.

	h := subStep.dt

	// Integrate positions
	for i := 0; i < this.BodyCount; i++ {
		c := this.Positions[i].c
		a := this.Positions[i].a
		v := this.Velocities[i].v
		w := this.Velocities[i].w

		// Check for large velocities
		translation := MulFV(h, v)
		if DotVV(translation, translation) > MaxTranslationSquared {
			ratio := MaxTranslation / translation.Length()
			v.Mul(ratio)
		}

		rotation := h * w
		if rotation*rotation > MaxRotationSquared {
			ratio := MaxRotation / AbsF(rotation)
			w *= ratio
		}

		// Integrate
		c.Add(MulFV(h, v))
		a += h * w

		this.Positions[i].c = c
		this.Positions[i].a = a
		this.Velocities[i].v = v
		this.Velocities[i].w = w

		// Sync bodies
		body := this.Bodies[i]
		body.sweep.C = c
		body.sweep.A = a
		body.linearVelocity = v
		body.angularVelocity = w
		body.synchronizeTransform()
	}

	this.Report(contactSolver.VelocityConstraints)
}

func (this *Island) AddBody(body *Body) {
	body.islandIndex = this.BodyCount
	this.Bodies[this.BodyCount] = body
	this.BodyCount++
}

func (this *Island) AddContact(contact IContact) {
	this.Contacts[this.ContactCount] = contact
	this.ContactCount++
}

func (this *Island) AddJoint(joint IJoint) {
	this.Joints[this.JointCount] = joint
	this.JointCount++
}

func (this *Island) Report(constraints []ContactVelocityConstraint) {
	if this.Listener == nil {
		return
	}

	for i := 0; i < this.ContactCount; i++ {
		c := this.Contacts[i]

		vc := &constraints[i]

		var impulse ContactImpulse
		impulse.Count = vc.PointCount
		for j := 0; j < vc.PointCount; j++ {
			impulse.NormalImpulses[j] = vc.Points[j].NormalImpulse
			impulse.TangentImpulses[j] = vc.Points[j].TangentImpulse
		}

		this.Listener.PostSolve(c, &impulse)
	}
}
