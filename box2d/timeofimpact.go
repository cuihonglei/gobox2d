package box2d

// Input parameters for b2TimeOfImpact
type TOIInput struct {
	ProxyA DistanceProxy
	ProxyB DistanceProxy
	SweepA Sweep
	SweepB Sweep
	TMax   float64 // defines sweep interval [0, tMax]
}

// Output parameters for b2TimeOfImpact.
type TOIOutput struct {
	State TOIOutputState
	T     float64
}

type TOIOutputState byte

const (
	TOIOutput_e_unknown = iota
	TOIOutput_e_failed
	TOIOutput_e_overlapped
	TOIOutput_e_touching
	TOIOutput_e_separated
)

type SeparationFunction struct {
	proxyA         *DistanceProxy
	proxyB         *DistanceProxy
	sweepA, sweepB Sweep
	itype          SeparationFunctionType
	localPoint     Vec2
	axis           Vec2
}

type SeparationFunctionType byte

const (
	SeparationFunction_e_points = iota
	SeparationFunction_e_faceA
	SeparationFunction_e_faceB
)

func (this *SeparationFunction) Initialize(cache *SimplexCache,
	proxyA *DistanceProxy, sweepA *Sweep,
	proxyB *DistanceProxy, sweepB *Sweep,
	t1 float64) float64 {

	this.proxyA = proxyA
	this.proxyB = proxyB
	count := cache.Count

	this.sweepA = *sweepA
	this.sweepB = *sweepB

	xfA := this.sweepA.GetTransform(t1)
	xfB := this.sweepB.GetTransform(t1)

	if count == 1 {
		this.itype = SeparationFunction_e_points
		localPointA := this.proxyA.GetVertex(int32(cache.IndexA[0]))
		localPointB := this.proxyB.GetVertex(int32(cache.IndexB[0]))
		pointA := MulX(xfA, localPointA)
		pointB := MulX(xfB, localPointB)
		this.axis = SubVV(pointB, pointA)
		s := this.axis.Normalize()
		return s
	} else if cache.IndexA[0] == cache.IndexA[1] {
		// Two points on B and one on A.
		this.itype = SeparationFunction_e_faceB
		localPointB1 := proxyB.GetVertex(int32(cache.IndexB[0]))
		localPointB2 := proxyB.GetVertex(int32(cache.IndexB[1]))

		this.axis = CrossVF(SubVV(localPointB2, localPointB1), 1.0)
		this.axis.Normalize()
		normal := MulRV(xfB.Q, this.axis)

		this.localPoint = MulFV(0.5, AddVV(localPointB1, localPointB2))
		pointB := MulX(xfB, this.localPoint)

		localPointA := proxyA.GetVertex(int32(cache.IndexA[0]))
		pointA := MulX(xfA, localPointA)

		s := DotVV(SubVV(pointA, pointB), normal)
		if s < 0.0 {
			this.axis = this.axis.Minus()
			s = -s
		}
		return s
	} else {
		// Two points on A and one or two points on B.
		this.itype = SeparationFunction_e_faceA
		localPointA1 := this.proxyA.GetVertex(int32(cache.IndexA[0]))
		localPointA2 := this.proxyA.GetVertex(int32(cache.IndexA[1]))

		this.axis = CrossVF(SubVV(localPointA2, localPointA1), 1.0)
		this.axis.Normalize()
		normal := MulRV(xfA.Q, this.axis)

		this.localPoint = MulFV(0.5, AddVV(localPointA1, localPointA2))
		pointA := MulX(xfA, this.localPoint)

		localPointB := this.proxyB.GetVertex(int32(cache.IndexB[0]))
		pointB := MulX(xfB, localPointB)

		s := DotVV(SubVV(pointB, pointA), normal)
		if s < 0.0 {
			this.axis = this.axis.Minus()
			s = -s
		}
		return s
	}
}

func (this *SeparationFunction) FindMinSeparation(t float64) (indexA, indexB int32, separation float64) {
	xfA := this.sweepA.GetTransform(t)
	xfB := this.sweepB.GetTransform(t)

	switch this.itype {
	case SeparationFunction_e_points:
		axisA := MulTRV(xfA.Q, this.axis)
		axisB := MulTRV(xfB.Q, this.axis.Minus())

		indexA = this.proxyA.GetSupport(axisA)
		indexB = this.proxyB.GetSupport(axisB)

		localPointA := this.proxyA.GetVertex(indexA)
		localPointB := this.proxyB.GetVertex(indexB)

		pointA := MulX(xfA, localPointA)
		pointB := MulX(xfB, localPointB)

		separation = DotVV(SubVV(pointB, pointA), this.axis)
		return
	case SeparationFunction_e_faceA:
		normal := MulRV(xfA.Q, this.axis)
		pointA := MulX(xfA, this.localPoint)

		axisB := MulTRV(xfB.Q, normal.Minus())

		indexA = -1
		indexB = this.proxyB.GetSupport(axisB)

		localPointB := this.proxyB.GetVertex(indexB)
		pointB := MulX(xfB, localPointB)

		separation = DotVV(SubVV(pointB, pointA), normal)
		return
	case SeparationFunction_e_faceB:
		normal := MulRV(xfB.Q, this.axis)
		pointB := MulX(xfB, this.localPoint)

		axisA := MulTRV(xfA.Q, normal.Minus())

		indexB = -1
		indexA = this.proxyA.GetSupport(axisA)

		localPointA := this.proxyA.GetVertex(indexA)
		pointA := MulX(xfA, localPointA)

		separation = DotVV(SubVV(pointA, pointB), normal)
		return
	default:
		//Assert(false)
		indexA = -1
		indexB = -1
		separation = 0.0
		return
	}
	return
}

func (this *SeparationFunction) Evaluate(indexA int32, indexB int32, t float64) (separation float64) {
	xfA := this.sweepA.GetTransform(t)
	xfB := this.sweepB.GetTransform(t)

	switch this.itype {
	case SeparationFunction_e_points:
		//axisA := MulTRV(xfA.Q, this.axis)
		//axisB := MulTRV(xfB.Q, this.axis.Minus())

		localPointA := this.proxyA.GetVertex(indexA)
		localPointB := this.proxyB.GetVertex(indexB)

		pointA := MulX(xfA, localPointA)
		pointB := MulX(xfB, localPointB)
		separation = DotVV(SubVV(pointB, pointA), this.axis)
		return
	case SeparationFunction_e_faceA:
		normal := MulRV(xfA.Q, this.axis)
		pointA := MulX(xfA, this.localPoint)

		//axisB := MulTRV(xfB.Q, normal.Minus())

		localPointB := this.proxyB.GetVertex(indexB)
		pointB := MulX(xfB, localPointB)

		separation = DotVV(SubVV(pointB, pointA), normal)
		return
	case SeparationFunction_e_faceB:
		normal := MulRV(xfB.Q, this.axis)
		pointB := MulX(xfB, this.localPoint)

		//axisA := MulTRV(xfA.Q, normal.Minus())

		localPointA := this.proxyA.GetVertex(indexA)
		pointA := MulX(xfA, localPointA)

		separation = DotVV(SubVV(pointA, pointB), normal)
		return
	default:
		//Assert(false)
		separation = 0.0
		return
	}
	return
}

var ToiCalls, ToiIters, ToiMaxIters int32
var ToiRootIters, ToiMaxRootIters int32

// Compute the upper bound on time before two shapes penetrate. Time is represented as
// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
// non-tunneling collision. If you change the time interval, you should call this function
// again.
// Note: use b2Distance to compute the contact point and normal at the time of impact.
func TimeOfImpact(output *TOIOutput, input *TOIInput) {
	ToiCalls++

	output.State = TOIOutput_e_unknown
	output.T = input.TMax

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	sweepA := input.SweepA
	sweepB := input.SweepB

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	sweepA.Normalize()
	sweepB.Normalize()

	tMax := input.TMax

	totalRadius := proxyA.Radius + proxyB.Radius
	target := MaxF(LinearSlop, totalRadius-3.0*LinearSlop)
	tolerance := 0.25 * LinearSlop

	t1 := 0.0
	const k_maxIterations int32 = 20 // TODO_ERIN b2Settings
	iter := int32(0)

	// Prepare input for distance query.
	var cache SimplexCache
	cache.Count = 0
	distanceInput := DistanceInput{}
	distanceInput.ProxyA = input.ProxyA
	distanceInput.ProxyB = input.ProxyB
	distanceInput.UseRadii = false

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for {
		xfA := sweepA.GetTransform(t1)
		xfB := sweepB.GetTransform(t1)

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.TransformA = xfA
		distanceInput.TransformB = xfB
		distanceOutput := Distance(&cache, &distanceInput)

		// If the shapes are overlapped, we give up on continuous collision.
		if distanceOutput.Distance <= 0.0 {
			// Failure!
			output.State = TOIOutput_e_overlapped
			output.T = 0.0
			break
		}

		if distanceOutput.Distance < target+tolerance {
			// Victory!
			output.State = TOIOutput_e_touching
			output.T = t1
			break
		}

		// Initialize the separating axis.
		var fcn SeparationFunction
		fcn.Initialize(&cache, proxyA, &sweepA, proxyB, &sweepB, t1)
		/*#if 0
				// Dump the curve seen by the root finder
				{
					const int32 N = 100;
					float64 dx = 1.0f / N;
					float64 xs[N+1];
					float64 fs[N+1];

					float64 x = 0.0f;

					for (int32 i = 0; i <= N; ++i)
					{
						sweepA.GetTransform(&xfA, x);
						sweepB.GetTransform(&xfB, x);
						float64 f = fcn.Evaluate(xfA, xfB) - target;

						printf("%g %g\n", x, f);

						xs[i] = x;
						fs[i] = f;

						x += dx;
					}
				}
		#endif */

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		done := false
		t2 := tMax
		pushBackIter := int32(0)
		for {
			// Find the deepest point at t2. Store the witness point indices.
			indexA, indexB, s2 := fcn.FindMinSeparation(t2)

			// Is the final configuration separated?
			if s2 > target+tolerance {
				// Victory!
				output.State = TOIOutput_e_separated
				output.T = tMax
				done = true
				break
			}

			// Has the separation reached tolerance?
			if s2 > target-tolerance {
				// Advance the sweeps
				t1 = t2
				break
			}

			// Compute the initial separation of the witness points.
			s1 := fcn.Evaluate(indexA, indexB, t1)

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if s1 < target-tolerance {
				output.State = TOIOutput_e_failed
				output.T = t1
				done = true
				break
			}

			// Check for touching
			if s1 <= target+tolerance {
				// Victory! t1 should hold the TOI (could be 0.0).
				output.State = TOIOutput_e_touching
				output.T = t1
				done = true
				break
			}

			// Compute 1D root of: f(x) - target = 0
			rootIterCount := int32(0)
			a1, a2 := t1, t2
			for {
				// Use a mix of the secant rule and bisection.
				var t float64
				if (rootIterCount & 1) != 0 {
					// Secant rule to improve convergence.
					t = a1 + (target-s1)*(a2-a1)/(s2-s1)
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2)
				}

				s := fcn.Evaluate(indexA, indexB, t)

				if AbsF(s-target) < tolerance {
					// t2 holds a tentative value for t1
					t2 = t
					break
				}

				// Ensure we continue to bracket the root.
				if s > target {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}

				rootIterCount++
				ToiRootIters++

				if rootIterCount == 50 {
					break
				}
			}

			ToiMaxRootIters = MaxI(ToiMaxRootIters, rootIterCount)

			pushBackIter++

			if pushBackIter == MaxPolygonVertices {
				break
			}
		}

		iter++
		ToiIters++

		if done {
			break
		}

		if iter == k_maxIterations {
			// Root finder got stuck. Semi-victory.
			output.State = TOIOutput_e_failed
			output.T = t1
			break
		}
	}

	ToiMaxIters = MaxI(ToiMaxIters, iter)
}
