package box2d

type VelocityConstraintPoint struct {
	RA             Vec2
	RB             Vec2
	NormalImpulse  float64
	TangentImpulse float64
	NormalMass     float64
	TangentMass    float64
	VelocityBias   float64
}

type ContactPositionConstraint struct {
	LocalPoints                [MaxManifoldPoints]Vec2
	LocalNormal                Vec2
	LocalPoint                 Vec2
	IndexA                     int
	IndexB                     int
	InvMassA, InvMassB         float64
	LocalCenterA, LocalCenterB Vec2
	InvIA, InvIB               float64
	Type                       ManifoldType
	RadiusA, RadiusB           float64
	PointCount                 int
}

type ContactVelocityConstraint struct {
	Points             [MaxManifoldPoints]VelocityConstraintPoint
	Normal             Vec2
	NormalMass         Mat22
	K                  Mat22
	IndexA             int
	IndexB             int
	InvMassA, InvMassB float64
	InvIA, InvIB       float64
	Friction           float64
	Restitution        float64
	PointCount         int
	ContactIndex       int
}

type ContactSolverDef struct {
	Step       timeStep
	Contacts   []IContact
	Count      int
	Positions  []position
	Velocities []velocity
}

type ContactSolver struct {
	Step                timeStep
	Positions           []position
	Velocities          []velocity
	PositionConstraints []ContactPositionConstraint
	VelocityConstraints []ContactVelocityConstraint
	Contacts            []IContact
	Count               int
}

func NewContactSolver(def *ContactSolverDef) *ContactSolver {
	this := new(ContactSolver)
	this.Step = def.Step
	this.Count = def.Count
	this.PositionConstraints = make([]ContactPositionConstraint, this.Count, this.Count)
	this.VelocityConstraints = make([]ContactVelocityConstraint, this.Count, this.Count)
	this.Positions = def.Positions
	this.Velocities = def.Velocities
	this.Contacts = def.Contacts

	// Initialize position independent portions of the constraints.
	for i := 0; i < this.Count; i++ {
		contact := this.Contacts[i]

		fixtureA := contact.GetFixtureA()
		fixtureB := contact.GetFixtureB()
		shapeA := fixtureA.GetShape()
		shapeB := fixtureB.GetShape()
		radiusA := shapeA.GetRadius()
		radiusB := shapeB.GetRadius()
		bodyA := fixtureA.GetBody()
		bodyB := fixtureB.GetBody()
		manifold := contact.GetManifold()

		pointCount := manifold.PointCount

		vc := &this.VelocityConstraints[i]
		vc.Friction = contact.GetFriction()
		vc.Restitution = contact.GetRestitution()
		vc.IndexA = bodyA.islandIndex
		vc.IndexB = bodyB.islandIndex
		vc.InvMassA = bodyA.invMass
		vc.InvMassB = bodyB.invMass
		vc.InvIA = bodyA.invI
		vc.InvIB = bodyB.invI
		vc.ContactIndex = i
		vc.PointCount = pointCount
		vc.K.SetZero()
		vc.NormalMass.SetZero()

		pc := &this.PositionConstraints[i]
		pc.IndexA = bodyA.islandIndex
		pc.IndexB = bodyB.islandIndex
		pc.InvMassA = bodyA.invMass
		pc.InvMassB = bodyB.invMass
		pc.LocalCenterA = bodyA.sweep.LocalCenter
		pc.LocalCenterB = bodyB.sweep.LocalCenter
		pc.InvIA = bodyA.invI
		pc.InvIB = bodyB.invI
		pc.LocalNormal = manifold.LocalNormal
		pc.LocalPoint = manifold.LocalPoint
		pc.PointCount = pointCount
		pc.RadiusA = radiusA
		pc.RadiusB = radiusB
		pc.Type = manifold.Type

		for j := 0; j < pointCount; j++ {
			cp := &manifold.Points[j]
			vcp := &vc.Points[j]

			if this.Step.warmStarting {
				vcp.NormalImpulse = this.Step.dtRatio * cp.NormalImpulse
				vcp.TangentImpulse = this.Step.dtRatio * cp.TangentImpulse
			} else {
				vcp.NormalImpulse = 0.0
				vcp.TangentImpulse = 0.0
			}

			vcp.RA.SetZero()
			vcp.RB.SetZero()
			vcp.NormalMass = 0.0
			vcp.TangentMass = 0.0
			vcp.VelocityBias = 0.0

			pc.LocalPoints[j] = cp.LocalPoint
		}
	}
	return this
}

func (this *ContactSolver) InitializeVelocityConstraints() {
	for i := 0; i < this.Count; i++ {
		vc := &this.VelocityConstraints[i]
		pc := &this.PositionConstraints[i]

		radiusA := pc.RadiusA
		radiusB := pc.RadiusB
		manifold := this.Contacts[vc.ContactIndex].GetManifold()

		indexA := vc.IndexA
		indexB := vc.IndexB

		mA := vc.InvMassA
		mB := vc.InvMassB
		iA := vc.InvIA
		iB := vc.InvIB
		localCenterA := pc.LocalCenterA
		localCenterB := pc.LocalCenterB

		cA := this.Positions[indexA].c
		aA := this.Positions[indexA].a
		vA := this.Velocities[indexA].v
		wA := this.Velocities[indexA].w

		cB := this.Positions[indexB].c
		aB := this.Positions[indexB].a
		vB := this.Velocities[indexB].v
		wB := this.Velocities[indexB].w

		var xfA, xfB Transform
		xfA.Q.Set(aA)
		xfB.Q.Set(aB)
		xfA.P = SubVV(cA, MulRV(xfA.Q, localCenterA))
		xfB.P = SubVV(cB, MulRV(xfB.Q, localCenterB))

		var worldManifold WorldManifold
		worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB)

		vc.Normal = worldManifold.Normal

		pointCount := vc.PointCount
		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]

			vcp.RA = SubVV(worldManifold.Points[j], cA)
			vcp.RB = SubVV(worldManifold.Points[j], cB)

			rnA := CrossVV(vcp.RA, vc.Normal)
			rnB := CrossVV(vcp.RB, vc.Normal)

			kNormal := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			vcp.NormalMass = 0.0
			if kNormal > 0.0 {
				vcp.NormalMass = 1.0 / kNormal
			}

			tangent := CrossVF(vc.Normal, 1.0)

			rtA := CrossVV(vcp.RA, tangent)
			rtB := CrossVV(vcp.RB, tangent)

			kTangent := mA + mB + iA*rtA*rtA + iB*rtB*rtB

			vcp.TangentMass = 0.0
			if kTangent > 0.0 {
				vcp.TangentMass = 1.0 / kTangent
			}

			// Setup a velocity bias for restitution.
			vcp.VelocityBias = 0.0
			vRel := DotVV(vc.Normal, SubVV(AddVV(vB, CrossFV(wB, vcp.RB)), AddVV(vA, CrossFV(wA, vcp.RA))))
			if vRel < -VelocityThreshold {
				vcp.VelocityBias = -vc.Restitution * vRel
			}
		}

		// If we have two points, then prepare the block solver.
		if vc.PointCount == 2 {
			vcp1 := &vc.Points[0]
			vcp2 := &vc.Points[1]

			rn1A := CrossVV(vcp1.RA, vc.Normal)
			rn1B := CrossVV(vcp1.RB, vc.Normal)
			rn2A := CrossVV(vcp2.RA, vc.Normal)
			rn2B := CrossVV(vcp2.RB, vc.Normal)

			k11 := mA + mB + iA*rn1A*rn1A + iB*rn1B*rn1B
			k22 := mA + mB + iA*rn2A*rn2A + iB*rn2B*rn2B
			k12 := mA + mB + iA*rn1A*rn2A + iB*rn1B*rn2B

			// Ensure a reasonable condition number.
			const k_maxConditionNumber float64 = 1000.0
			if k11*k11 < k_maxConditionNumber*(k11*k22-k12*k12) {
				// K is safe to invert.
				vc.K.Ex.Set(k11, k12)
				vc.K.Ey.Set(k12, k22)
				vc.NormalMass = vc.K.GetInverse()
			} else {
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc.PointCount = 1
			}
		}
	}
}

func (this *ContactSolver) WarmStart() {
	// Warm start.
	for i := 0; i < this.Count; i++ {
		vc := &this.VelocityConstraints[i]

		indexA := vc.IndexA
		indexB := vc.IndexB
		mA := vc.InvMassA
		iA := vc.InvIA
		mB := vc.InvMassB
		iB := vc.InvIB
		pointCount := vc.PointCount

		vA := this.Velocities[indexA].v
		wA := this.Velocities[indexA].w
		vB := this.Velocities[indexB].v
		wB := this.Velocities[indexB].w

		normal := vc.Normal
		tangent := CrossVF(normal, 1.0)

		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]
			P := AddVV(MulFV(vcp.NormalImpulse, normal), MulFV(vcp.TangentImpulse, tangent))
			wA -= iA * CrossVV(vcp.RA, P)
			vA.Sub(MulFV(mA, P))
			wB += iB * CrossVV(vcp.RB, P)
			vB.Add(MulFV(mB, P))
		}

		this.Velocities[indexA].v = vA
		this.Velocities[indexA].w = wA
		this.Velocities[indexB].v = vB
		this.Velocities[indexB].w = wB
	}
}

func (this *ContactSolver) SolveVelocityConstraints() {
	for i := 0; i < this.Count; i++ {
		vc := &this.VelocityConstraints[i]

		indexA := vc.IndexA
		indexB := vc.IndexB
		mA := vc.InvMassA
		iA := vc.InvIA
		mB := vc.InvMassB
		iB := vc.InvIB
		pointCount := vc.PointCount

		vA := this.Velocities[indexA].v
		wA := this.Velocities[indexA].w
		vB := this.Velocities[indexB].v
		wB := this.Velocities[indexB].w

		normal := vc.Normal
		tangent := CrossVF(normal, 1.0)
		friction := vc.Friction

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for j := 0; j < pointCount; j++ {
			vcp := &vc.Points[j]

			// Relative velocity at contact
			dv := SubVV(AddVV(vB, CrossFV(wB, vcp.RB)), AddVV(vA, CrossFV(wA, vcp.RA)))

			// Compute tangent force
			vt := DotVV(dv, tangent)
			lambda := vcp.TangentMass * -vt

			// b2Clamp the accumulated force
			maxFriction := friction * vcp.NormalImpulse
			newImpulse := ClampF(vcp.TangentImpulse+lambda, -maxFriction, maxFriction)
			lambda = newImpulse - vcp.TangentImpulse
			vcp.TangentImpulse = newImpulse

			// Apply contact impulse
			P := MulFV(lambda, tangent)

			vA.Sub(MulFV(mA, P))
			wA -= iA * CrossVV(vcp.RA, P)

			vB.Add(MulFV(mB, P))
			wB += iB * CrossVV(vcp.RB, P)
		}

		// Solve normal constraints
		if vc.PointCount == 1 {
			vcp := &vc.Points[0]

			// Relative velocity at contact
			dv := SubVV(AddVV(vB, CrossFV(wB, vcp.RB)), AddVV(vA, CrossFV(wA, vcp.RA)))

			// Compute normal impulse
			vn := DotVV(dv, normal)
			lambda := -vcp.NormalMass * (vn - vcp.VelocityBias)

			// b2Clamp the accumulated impulse
			newImpulse := MaxF(vcp.NormalImpulse+lambda, 0.0)
			lambda = newImpulse - vcp.NormalImpulse
			vcp.NormalImpulse = newImpulse

			// Apply contact impulse
			P := MulFV(lambda, normal)
			vA.Sub(MulFV(mA, P))
			wA -= iA * CrossVV(vcp.RA, P)

			vB.Add(MulFV(mB, P))
			wB += iB * CrossVV(vcp.RB, P)
		} else {
			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			// Build the mini LCP for this contact patch
			//
			// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			//
			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			// b = vn0 - velocityBias
			//
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
			// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
			// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
			// solution that satisfies the problem is chosen.
			//
			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
			// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
			//
			// Substitute:
			//
			// x = a + d
			//
			// a := old total impulse
			// x := new total impulse
			// d := incremental impulse
			//
			// For the current iteration we extend the formula for the incremental impulse
			// to compute the new total impulse:
			//
			// vn = A * d + b
			//    = A * (x - a) + b
			//    = A * x + b - A * a
			//    = A * x + b'
			// b' = b - A * a;

			cp1 := &vc.Points[0]
			cp2 := &vc.Points[1]

			a := Vec2{cp1.NormalImpulse, cp2.NormalImpulse}

			// Relative velocity at contact
			dv1 := SubVV(AddVV(vB, CrossFV(wB, cp1.RB)), AddVV(vA, CrossFV(wA, cp1.RA)))
			dv2 := SubVV(AddVV(vB, CrossFV(wB, cp2.RB)), AddVV(vA, CrossFV(wA, cp2.RA)))

			// Compute normal velocity
			vn1 := DotVV(dv1, normal)
			vn2 := DotVV(dv2, normal)

			var b Vec2
			b.X = vn1 - cp1.VelocityBias
			b.Y = vn2 - cp2.VelocityBias

			// Compute b'
			b.Sub(MulMV(vc.K, a))

			//const k_errorTol float64 = 1e-3
			//NOT_USED(k_errorTol)

			for {
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// Solve for x:
				//
				// x = - inv(A) * b'
				//
				x := MulMV(vc.NormalMass, b)
				x = x.Minus()

				if x.X >= 0.0 && x.Y >= 0.0 {
					// Get the incremental impulse
					d := SubVV(x, a)

					// Apply incremental impulse
					P1 := MulFV(d.X, normal)
					P2 := MulFV(d.Y, normal)
					vA.Sub(MulFV(mA, AddVV(P1, P2)))
					wA -= iA * (CrossVV(cp1.RA, P1) + CrossVV(cp2.RA, P2))

					vB.Add(MulFV(mB, AddVV(P1, P2)))
					wB += iB * (CrossVV(cp1.RB, P1) + CrossVV(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					//#if B2_DEBUG_SOLVER == 1
					//					// Postconditions
					//					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
					//					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					//					// Compute normal velocity
					//					vn1 = b2Dot(dv1, normal);
					//					vn2 = b2Dot(dv2, normal);

					//					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
					//					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
					//#endif
					break
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1'
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.X = -cp1.NormalMass * b.X
				x.Y = 0.0
				vn1 = 0.0
				vn2 = vc.K.Ex.Y*x.X + b.Y

				if x.X >= 0.0 && vn2 >= 0.0 {
					// Get the incremental impulse
					d := SubVV(x, a)

					// Apply incremental impulse
					P1 := MulFV(d.X, normal)
					P2 := MulFV(d.Y, normal)
					vA.Sub(MulFV(mA, AddVV(P1, P2)))
					wA -= iA * (CrossVV(cp1.RA, P1) + CrossVV(cp2.RA, P2))

					vB.Add(MulFV(mB, AddVV(P1, P2)))
					wB += iB * (CrossVV(cp1.RB, P1) + CrossVV(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					//#if B2_DEBUG_SOLVER == 1
					//					// Postconditions
					//					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);

					//					// Compute normal velocity
					//					vn1 = b2Dot(dv1, normal);

					//					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
					//#endif
					break
				}

				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1'
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.X = 0.0
				x.Y = -cp2.NormalMass * b.Y
				vn1 = vc.K.Ey.X*x.Y + b.X
				vn2 = 0.0

				if x.Y >= 0.0 && vn1 >= 0.0 {
					// Resubstitute for the incremental impulse
					d := SubVV(x, a)

					// Apply incremental impulse
					P1 := MulFV(d.X, normal)
					P2 := MulFV(d.Y, normal)
					vA.Sub(MulFV(mA, AddVV(P1, P2)))
					wA -= iA * (CrossVV(cp1.RA, P1) + CrossVV(cp2.RA, P2))

					vB.Add(MulFV(mB, AddVV(P1, P2)))
					wB += iB * (CrossVV(cp1.RB, P1) + CrossVV(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					//#if B2_DEBUG_SOLVER == 1
					//					// Postconditions
					//					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					//					// Compute normal velocity
					//					vn2 = b2Dot(dv2, normal);

					//					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
					//#endif
					break
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				//
				// vn1 = b1
				// vn2 = b2;
				x.X = 0.0
				x.Y = 0.0
				vn1 = b.X
				vn2 = b.Y

				if vn1 >= 0.0 && vn2 >= 0.0 {
					// Resubstitute for the incremental impulse
					d := SubVV(x, a)

					// Apply incremental impulse
					P1 := MulFV(d.X, normal)
					P2 := MulFV(d.Y, normal)
					vA.Sub(MulFV(mA, AddVV(P1, P2)))
					wA -= iA * (CrossVV(cp1.RA, P1) + CrossVV(cp2.RA, P2))

					vB.Add(MulFV(mB, AddVV(P1, P2)))
					wB += iB * (CrossVV(cp1.RB, P1) + CrossVV(cp2.RB, P2))

					// Accumulate
					cp1.NormalImpulse = x.X
					cp2.NormalImpulse = x.Y

					break
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break
			}
		}

		this.Velocities[indexA].v = vA
		this.Velocities[indexA].w = wA
		this.Velocities[indexB].v = vB
		this.Velocities[indexB].w = wB
	}
}

func (this *ContactSolver) StoreImpulses() {
	for i := 0; i < this.Count; i++ {
		vc := &this.VelocityConstraints[i]
		manifold := this.Contacts[vc.ContactIndex].GetManifold()

		for j := 0; j < vc.PointCount; j++ {
			manifold.Points[j].NormalImpulse = vc.Points[j].NormalImpulse
			manifold.Points[j].TangentImpulse = vc.Points[j].TangentImpulse
		}
	}
}

type PositionSolverManifold struct {
	Normal     Vec2
	Point      Vec2
	Separation float64
}

func (this *PositionSolverManifold) Initialize(pc *ContactPositionConstraint, xfA, xfB Transform, index int) {
	//Assert(pc.PointCount > 0)
	switch pc.Type {
	case Manifold_e_circles:
		pointA := MulX(xfA, pc.LocalPoint)
		pointB := MulX(xfB, pc.LocalPoints[0])
		this.Normal = SubVV(pointB, pointA)
		this.Normal.Normalize()
		this.Point = MulFV(0.5, AddVV(pointA, pointB))
		this.Separation = DotVV(SubVV(pointB, pointA), this.Normal) - pc.RadiusA - pc.RadiusB

	case Manifold_e_faceA:
		this.Normal = MulRV(xfA.Q, pc.LocalNormal)
		planePoint := MulX(xfA, pc.LocalPoint)

		clipPoint := MulX(xfB, pc.LocalPoints[index])
		this.Separation = DotVV(SubVV(clipPoint, planePoint), this.Normal) - pc.RadiusA - pc.RadiusB
		this.Point = clipPoint

	case Manifold_e_faceB:
		this.Normal = MulRV(xfB.Q, pc.LocalNormal)
		planePoint := MulX(xfB, pc.LocalPoint)

		clipPoint := MulX(xfA, pc.LocalPoints[index])
		this.Separation = DotVV(SubVV(clipPoint, planePoint), this.Normal) - pc.RadiusA - pc.RadiusB
		this.Point = clipPoint

		// Ensure normal points from A to B
		this.Normal = this.Normal.Minus()
	}
}

func (this *ContactSolver) SolvePositionConstraints() bool {
	minSeparation := 0.0

	for i := 0; i < this.Count; i++ {
		pc := &this.PositionConstraints[i]

		indexA := pc.IndexA
		indexB := pc.IndexB
		localCenterA := pc.LocalCenterA
		mA := pc.InvMassA
		iA := pc.InvIA
		localCenterB := pc.LocalCenterB
		mB := pc.InvMassB
		iB := pc.InvIB
		pointCount := pc.PointCount

		cA := this.Positions[indexA].c
		aA := this.Positions[indexA].a

		cB := this.Positions[indexB].c
		aB := this.Positions[indexB].a

		// Solve normal constraints
		for j := 0; j < pointCount; j++ {
			var xfA, xfB Transform
			xfA.Q.Set(aA)
			xfB.Q.Set(aB)
			xfA.P = SubVV(cA, MulRV(xfA.Q, localCenterA))
			xfB.P = SubVV(cB, MulRV(xfB.Q, localCenterB))

			var psm PositionSolverManifold
			psm.Initialize(pc, xfA, xfB, j)
			normal := psm.Normal

			point := psm.Point
			separation := psm.Separation

			rA := SubVV(point, cA)
			rB := SubVV(point, cB)

			// Track max constraint error.
			minSeparation = MinF(minSeparation, separation)

			// Prevent large corrections and allow slop.
			C := ClampF(Baumgarte*(separation+LinearSlop), -MaxLinearCorrection, 0.0)

			// Compute the effective mass.
			rnA := CrossVV(rA, normal)
			rnB := CrossVV(rB, normal)
			K := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			// Compute normal impulse
			impulse := 0.0
			if K > 0.0 {
				impulse = -C / K
			}

			P := MulFV(impulse, normal)

			cA.Sub(MulFV(mA, P))
			aA -= iA * CrossVV(rA, P)

			cB.Add(MulFV(mB, P))
			aB += iB * CrossVV(rB, P)
		}

		this.Positions[indexA].c = cA
		this.Positions[indexA].a = aA

		this.Positions[indexB].c = cB
		this.Positions[indexB].a = aB
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0*LinearSlop
}

func (this *ContactSolver) SolveTOIPositionConstraints(toiIndexA, toiIndexB int) bool {
	minSeparation := 0.0

	for i := 0; i < this.Count; i++ {
		pc := &this.PositionConstraints[i]

		indexA := pc.IndexA
		indexB := pc.IndexB
		localCenterA := pc.LocalCenterA
		localCenterB := pc.LocalCenterB
		pointCount := pc.PointCount

		mA := 0.0
		iA := 0.0
		if indexA == toiIndexA || indexA == toiIndexB {
			mA = pc.InvMassA
			iA = pc.InvIA
		}

		mB := pc.InvMassB
		iB := pc.InvIB
		if indexB == toiIndexA || indexB == toiIndexB {
			mB = pc.InvMassB
			iB = pc.InvIB
		}

		cA := this.Positions[indexA].c
		aA := this.Positions[indexA].a

		cB := this.Positions[indexB].c
		aB := this.Positions[indexB].a

		// Solve normal constraints
		for j := 0; j < pointCount; j++ {
			var xfA, xfB Transform
			xfA.Q.Set(aA)
			xfB.Q.Set(aB)
			xfA.P = SubVV(cA, MulRV(xfA.Q, localCenterA))
			xfB.P = SubVV(cB, MulRV(xfB.Q, localCenterB))

			var psm PositionSolverManifold
			psm.Initialize(pc, xfA, xfB, j)
			normal := psm.Normal

			point := psm.Point
			separation := psm.Separation

			rA := SubVV(point, cA)
			rB := SubVV(point, cB)

			// Track max constraint error.
			minSeparation = MinF(minSeparation, separation)

			// Prevent large corrections and allow slop.
			C := ClampF(ToiBaugarte*(separation+LinearSlop), -MaxLinearCorrection, 0.0)

			// Compute the effective mass.
			rnA := CrossVV(rA, normal)
			rnB := CrossVV(rB, normal)
			K := mA + mB + iA*rnA*rnA + iB*rnB*rnB

			// Compute normal impulse
			impulse := 0.0
			if K > 0.0 {
				impulse = -C / K
			}

			P := MulFV(impulse, normal)

			cA.Sub(MulFV(mA, P))
			aA -= iA * CrossVV(rA, P)

			cB.Add(MulFV(mB, P))
			aB += iB * CrossVV(rB, P)
		}

		this.Positions[indexA].c = cA
		this.Positions[indexA].a = aA

		this.Positions[indexB].c = cB
		this.Positions[indexB].a = aB
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5*LinearSlop
}
