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
	TOIOutput_e_unknown TOIOutputState = iota
	TOIOutput_e_failed
	TOIOutput_e_overlapped
	TOIOutput_e_touching
	TOIOutput_e_separated
)

type SeparationFunction struct {
	proxyA         *DistanceProxy
	proxyB         *DistanceProxy
	sweepA, sweepB Sweep
	xtype          SeparationFunctionType
	localPoint     Vec2
	axis           Vec2
}

type SeparationFunctionType byte

const (
	SeparationFunction_e_points SeparationFunctionType = iota
	SeparationFunction_e_faceA
	SeparationFunction_e_faceB
)

func (sf *SeparationFunction) Initialize(cache *SimplexCache,
	proxyA *DistanceProxy, sweepA *Sweep,
	proxyB *DistanceProxy, sweepB *Sweep,
	t1 float64) float64 {

	sf.proxyA = proxyA
	sf.proxyB = proxyB
	count := cache.Count

	sf.sweepA = *sweepA
	sf.sweepB = *sweepB

	xfA := sf.sweepA.GetTransform(t1)
	xfB := sf.sweepB.GetTransform(t1)

	if count == 1 {
		sf.xtype = SeparationFunction_e_points
		localPointA := sf.proxyA.GetVertex(int(cache.IndexA[0]))
		localPointB := sf.proxyB.GetVertex(int(cache.IndexB[0]))
		pointA := MulX(xfA, localPointA)
		pointB := MulX(xfB, localPointB)
		sf.axis = SubVV(pointB, pointA)
		s := sf.axis.Normalize()
		return s
	} else if cache.IndexA[0] == cache.IndexA[1] {
		// Two points on B and one on A.
		sf.xtype = SeparationFunction_e_faceB
		localPointB1 := proxyB.GetVertex(int(cache.IndexB[0]))
		localPointB2 := proxyB.GetVertex(int(cache.IndexB[1]))

		sf.axis = CrossVF(SubVV(localPointB2, localPointB1), 1.0)
		sf.axis.Normalize()
		normal := MulRV(xfB.Q, sf.axis)

		sf.localPoint = MulFV(0.5, AddVV(localPointB1, localPointB2))
		pointB := MulX(xfB, sf.localPoint)

		localPointA := proxyA.GetVertex(int(cache.IndexA[0]))
		pointA := MulX(xfA, localPointA)

		s := DotVV(SubVV(pointA, pointB), normal)
		if s < 0.0 {
			sf.axis = sf.axis.Minus()
			s = -s
		}
		return s
	} else {
		// Two points on A and one or two points on B.
		sf.xtype = SeparationFunction_e_faceA
		localPointA1 := sf.proxyA.GetVertex(int(cache.IndexA[0]))
		localPointA2 := sf.proxyA.GetVertex(int(cache.IndexA[1]))

		sf.axis = CrossVF(SubVV(localPointA2, localPointA1), 1.0)
		sf.axis.Normalize()
		normal := MulRV(xfA.Q, sf.axis)

		sf.localPoint = MulFV(0.5, AddVV(localPointA1, localPointA2))
		pointA := MulX(xfA, sf.localPoint)

		localPointB := sf.proxyB.GetVertex(int(cache.IndexB[0]))
		pointB := MulX(xfB, localPointB)

		s := DotVV(SubVV(pointB, pointA), normal)
		if s < 0.0 {
			sf.axis = sf.axis.Minus()
			s = -s
		}
		return s
	}
}

func (sf *SeparationFunction) FindMinSeparation(t float64) (indexA, indexB int, separation float64) {
	xfA := sf.sweepA.GetTransform(t)
	xfB := sf.sweepB.GetTransform(t)

	switch sf.xtype {
	case SeparationFunction_e_points:
		axisA := MulTRV(xfA.Q, sf.axis)
		axisB := MulTRV(xfB.Q, sf.axis.Minus())

		indexA = sf.proxyA.GetSupport(axisA)
		indexB = sf.proxyB.GetSupport(axisB)

		localPointA := sf.proxyA.GetVertex(indexA)
		localPointB := sf.proxyB.GetVertex(indexB)

		pointA := MulX(xfA, localPointA)
		pointB := MulX(xfB, localPointB)

		separation = DotVV(SubVV(pointB, pointA), sf.axis)
		return
	case SeparationFunction_e_faceA:
		normal := MulRV(xfA.Q, sf.axis)
		pointA := MulX(xfA, sf.localPoint)

		axisB := MulTRV(xfB.Q, normal.Minus())

		indexA = -1
		indexB = sf.proxyB.GetSupport(axisB)

		localPointB := sf.proxyB.GetVertex(indexB)
		pointB := MulX(xfB, localPointB)

		separation = DotVV(SubVV(pointB, pointA), normal)
		return
	case SeparationFunction_e_faceB:
		normal := MulRV(xfB.Q, sf.axis)
		pointB := MulX(xfB, sf.localPoint)

		axisA := MulTRV(xfA.Q, normal.Minus())

		indexB = -1
		indexA = sf.proxyA.GetSupport(axisA)

		localPointA := sf.proxyA.GetVertex(indexA)
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
}

func (sf *SeparationFunction) Evaluate(indexA int, indexB int, t float64) (separation float64) {
	xfA := sf.sweepA.GetTransform(t)
	xfB := sf.sweepB.GetTransform(t)

	switch sf.xtype {
	case SeparationFunction_e_points:
		//axisA := MulTRV(xfA.Q, sf.axis)
		//axisB := MulTRV(xfB.Q, sf.axis.Minus())

		localPointA := sf.proxyA.GetVertex(indexA)
		localPointB := sf.proxyB.GetVertex(indexB)

		pointA := MulX(xfA, localPointA)
		pointB := MulX(xfB, localPointB)
		separation = DotVV(SubVV(pointB, pointA), sf.axis)
		return
	case SeparationFunction_e_faceA:
		normal := MulRV(xfA.Q, sf.axis)
		pointA := MulX(xfA, sf.localPoint)

		//axisB := MulTRV(xfB.Q, normal.Minus())

		localPointB := sf.proxyB.GetVertex(indexB)
		pointB := MulX(xfB, localPointB)

		separation = DotVV(SubVV(pointB, pointA), normal)
		return
	case SeparationFunction_e_faceB:
		normal := MulRV(xfB.Q, sf.axis)
		pointB := MulX(xfB, sf.localPoint)

		//axisA := MulTRV(xfA.Q, normal.Minus())

		localPointA := sf.proxyA.GetVertex(indexA)
		pointA := MulX(xfA, localPointA)

		separation = DotVV(SubVV(pointA, pointB), normal)
		return
	default:
		//Assert(false)
		separation = 0.0
		return
	}
}

var ToiCalls, ToiIters, ToiMaxIters int
var ToiRootIters, ToiMaxRootIters int

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
	const k_maxIterations int = 20 // TODO_ERIN b2Settings
	iter := 0

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
					const int N = 100;
					float64 dx = 1.0f / N;
					float64 xs[N+1];
					float64 fs[N+1];

					float64 x = 0.0f;

					for (int i = 0; i <= N; ++i)
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
		pushBackIter := 0
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
			rootIterCount := 0
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
