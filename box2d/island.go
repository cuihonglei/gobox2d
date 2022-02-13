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
	island := new(Island)

	island.BodyCapacity = bodyCapacity
	island.ContactCapacity = contactCapacity
	island.JointCapacity = jointCapacity

	island.Listener = listener
	island.Bodies = make([]*Body, island.BodyCapacity)
	island.Contacts = make([]IContact, island.ContactCapacity)
	island.Joints = make([]IJoint, island.JointCapacity)

	island.Positions = make([]position, island.BodyCapacity)
	island.Velocities = make([]velocity, island.BodyCapacity)

	return island
}

func (island *Island) Clear() {
	island.BodyCount = 0
	island.JointCount = 0
	island.ContactCount = 0
}

func (island *Island) Solve(profile *Profile, step *timeStep, gravity Vec2, allowSleep bool) {
	timer := MakeTimer()

	h := step.dt

	// Integrate velocities and apply damping. Initialize the body state.
	for i := 0; i < island.BodyCount; i++ {
		b := island.Bodies[i]

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

		island.Positions[i].c = c
		island.Positions[i].a = a
		island.Velocities[i].v = v
		island.Velocities[i].w = w
	}

	timer.Reset()

	// Solver data
	var solverData solverData
	solverData.step = *step
	solverData.positions = island.Positions
	solverData.velocities = island.Velocities

	// Initialize velocity constraints.
	var contactSolverDef ContactSolverDef
	contactSolverDef.Step = *step
	contactSolverDef.Contacts = island.Contacts
	contactSolverDef.Count = island.ContactCount
	contactSolverDef.Positions = island.Positions
	contactSolverDef.Velocities = island.Velocities

	contactSolver := NewContactSolver(&contactSolverDef)
	contactSolver.InitializeVelocityConstraints()

	if step.warmStarting {
		contactSolver.WarmStart()
	}

	for i := 0; i < island.JointCount; i++ {
		island.Joints[i].InitVelocityConstraints(&solverData)
	}

	profile.solveInit = timer.GetMilliseconds()

	// Solve velocity constraints
	timer.Reset()
	for i := 0; i < step.velocityIterations; i++ {
		for j := 0; j < island.JointCount; j++ {
			island.Joints[j].SolveVelocityConstraints(&solverData)
		}

		contactSolver.SolveVelocityConstraints()
	}

	// Store impulses for warm starting
	contactSolver.StoreImpulses()
	profile.solveVelocity = timer.GetMilliseconds()

	// Integrate positions
	for i := 0; i < island.BodyCount; i++ {
		c := island.Positions[i].c
		a := island.Positions[i].a
		v := island.Velocities[i].v
		w := island.Velocities[i].w

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

		island.Positions[i].c = c
		island.Positions[i].a = a
		island.Velocities[i].v = v
		island.Velocities[i].w = w
	}

	// Solve position constraints
	timer.Reset()
	positionSolved := false
	for i := 0; i < step.positionIterations; i++ {
		contactsOkay := contactSolver.SolvePositionConstraints()

		jointsOkay := true
		for i := 0; i < island.JointCount; i++ {
			jointOkay := island.Joints[i].SolvePositionConstraints(&solverData)
			jointsOkay = jointsOkay && jointOkay
		}

		if contactsOkay && jointsOkay {
			// Exit early if the position errors are small.
			positionSolved = true
			break
		}
	}

	// Copy state buffers back to the bodies
	for i := 0; i < island.BodyCount; i++ {
		body := island.Bodies[i]
		body.sweep.C = island.Positions[i].c
		body.sweep.A = island.Positions[i].a
		body.linearVelocity = island.Velocities[i].v
		body.angularVelocity = island.Velocities[i].w
		body.synchronizeTransform()
	}

	profile.solvePosition = timer.GetMilliseconds()

	island.Report(contactSolver.VelocityConstraints)

	if allowSleep {
		minSleepTime := MaxFloat

		linTolSqr := LinearSleepTolerance * LinearSleepTolerance
		angTolSqr := AngularSleepTolerance * AngularSleepTolerance

		for i := 0; i < island.BodyCount; i++ {
			b := island.Bodies[i]
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
			for i := 0; i < island.BodyCount; i++ {
				b := island.Bodies[i]
				b.SetAwake(false)
			}
		}
	}
}

func (island *Island) SolveTOI(subStep *timeStep, toiIndexA int, toiIndexB int) {
	// Initialize the body state.
	for i := 0; i < island.BodyCount; i++ {
		b := island.Bodies[i]
		island.Positions[i].c = b.sweep.C
		island.Positions[i].a = b.sweep.A
		island.Velocities[i].v = b.linearVelocity
		island.Velocities[i].w = b.angularVelocity
	}

	var contactSolverDef ContactSolverDef
	contactSolverDef.Contacts = island.Contacts
	contactSolverDef.Count = island.ContactCount
	contactSolverDef.Step = *subStep
	contactSolverDef.Positions = island.Positions
	contactSolverDef.Velocities = island.Velocities
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
	island.Bodies[toiIndexA].sweep.C0 = island.Positions[toiIndexA].c
	island.Bodies[toiIndexA].sweep.A0 = island.Positions[toiIndexA].a
	island.Bodies[toiIndexB].sweep.C0 = island.Positions[toiIndexB].c
	island.Bodies[toiIndexB].sweep.A0 = island.Positions[toiIndexB].a

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
	for i := 0; i < island.BodyCount; i++ {
		c := island.Positions[i].c
		a := island.Positions[i].a
		v := island.Velocities[i].v
		w := island.Velocities[i].w

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

		island.Positions[i].c = c
		island.Positions[i].a = a
		island.Velocities[i].v = v
		island.Velocities[i].w = w

		// Sync bodies
		body := island.Bodies[i]
		body.sweep.C = c
		body.sweep.A = a
		body.linearVelocity = v
		body.angularVelocity = w
		body.synchronizeTransform()
	}

	island.Report(contactSolver.VelocityConstraints)
}

func (island *Island) AddBody(body *Body) {
	body.islandIndex = island.BodyCount
	island.Bodies[island.BodyCount] = body
	island.BodyCount++
}

func (island *Island) AddContact(contact IContact) {
	island.Contacts[island.ContactCount] = contact
	island.ContactCount++
}

func (island *Island) AddJoint(joint IJoint) {
	island.Joints[island.JointCount] = joint
	island.JointCount++
}

func (island *Island) Report(constraints []ContactVelocityConstraint) {
	if island.Listener == nil {
		return
	}

	for i := 0; i < island.ContactCount; i++ {
		c := island.Contacts[i]

		vc := &constraints[i]

		var impulse ContactImpulse
		impulse.Count = vc.PointCount
		for j := 0; j < vc.PointCount; j++ {
			impulse.NormalImpulses[j] = vc.Points[j].NormalImpulse
			impulse.TangentImpulses[j] = vc.Points[j].TangentImpulse
		}

		island.Listener.PostSolve(c, &impulse)
	}
}
