package box2d

// A distance proxy is used by the GJK algorithm.
// It encapsulates any shape.
type DistanceProxy struct {
	Vertices []Vec2
	Count    int
	Radius   float64
}

// Initialize the proxy using the given shape. The shape
// must remain in scope while the proxy is in use.
func (dp *DistanceProxy) Set(shape IShape, index int) {
	switch shape.GetType() {
	case Shape_e_circle:
		circle := shape.(*CircleShape)
		dp.Vertices = []Vec2{circle.P}
		dp.Count = 1
		dp.Radius = circle.Radius
	case Shape_e_polygon:
		polygon := shape.(*PolygonShape)
		dp.Vertices = make([]Vec2, polygon.VertexCount)
		copy(dp.Vertices, polygon.Vertices[:])
		dp.Count = polygon.VertexCount
		dp.Radius = polygon.Radius
	case Shape_e_chain:
		chain := shape.(*ChainShape)
		dp.Vertices = make([]Vec2, 2)
		dp.Vertices[0] = chain.Vertices[index]
		if index+1 < chain.Count {
			dp.Vertices[1] = chain.Vertices[index+1]
		} else {
			dp.Vertices[1] = chain.Vertices[0]
		}
		dp.Count = 2
		dp.Radius = chain.Radius
	case Shape_e_edge:
		edge := shape.(*EdgeShape)
		dp.Vertices = []Vec2{edge.Vertex1, edge.Vertex2}
		dp.Count = 2
		dp.Radius = edge.Radius
	}
}

// Get the supporting vertex index in the given direction.
func (dp *DistanceProxy) GetSupport(d Vec2) int {
	bestIndex := 0
	bestValue := DotVV(dp.Vertices[0], d)
	for i := 1; i < dp.Count; i++ {
		value := DotVV(dp.Vertices[i], d)
		if value > bestValue {
			bestIndex = i
			bestValue = value
		}
	}

	return bestIndex
}

// Get the supporting vertex in the given direction.
func (dp *DistanceProxy) GetSupportVertex(d Vec2) Vec2 {
	bestIndex := 0
	bestValue := DotVV(dp.Vertices[0], d)
	for i := 1; i < dp.Count; i++ {
		value := DotVV(dp.Vertices[i], d)
		if value > bestValue {
			bestIndex = i
			bestValue = value
		}
	}

	return dp.Vertices[bestIndex]
}

// Get the vertex count.
func (dp *DistanceProxy) GetVertexCount() int {
	return dp.Count
}

// Get a vertex by index. Used by b2Distance.
func (dp *DistanceProxy) GetVertex(index int) Vec2 {
	return dp.Vertices[index]
}

// Used to warm start b2Distance.
// Set count to zero on first call.
type SimplexCache struct {
	Metric float64 // length or area
	Count  uint16
	IndexA [3]uint8 // vertices on shape A
	IndexB [3]uint8 // vertices on shape B
}

// Input for b2Distance.
// You have to option to use the shape radii
// in the computation. Even
type DistanceInput struct {
	ProxyA     DistanceProxy
	ProxyB     DistanceProxy
	TransformA Transform
	TransformB Transform
	UseRadii   bool
}

/// Output for b2Distance.
type DistanceOutput struct {
	PointA     Vec2 // closest point on shapeA
	PointB     Vec2 // closest point on shapeB
	Distance   float64
	Iterations int // number of GJK iterations used
}

type SimplexVertex struct {
	wA     Vec2    // support point in proxyA
	wB     Vec2    // support point in proxyB
	w      Vec2    // wB - wA
	a      float64 // barycentric coordinate for closest point
	indexA int     // wA index
	indexB int     // wB index
}

type Simplex struct {
	v1, v2, v3 SimplexVertex
	count      int
}

func (s *Simplex) ReadCache(cache *SimplexCache,
	proxyA *DistanceProxy, transformA Transform,
	proxyB *DistanceProxy, transformB Transform) {
	// Copy data from cache.
	s.count = int(cache.Count)
	vertices := []*SimplexVertex{&s.v1, &s.v2, &s.v3}
	for i := 0; i < s.count; i++ {
		v := vertices[i]
		v.indexA = int(cache.IndexA[i])
		v.indexB = int(cache.IndexB[i])
		wALocal := proxyA.GetVertex(v.indexA)
		wBLocal := proxyB.GetVertex(v.indexB)
		v.wA = MulX(transformA, wALocal)
		v.wB = MulX(transformB, wBLocal)
		v.w = SubVV(v.wB, v.wA)
		v.a = 0.0
	}

	// Compute the new simplex metric, if it is substantially different than
	// old metric then flush the simplex.
	if s.count > 1 {
		metric1 := cache.Metric
		metric2 := s.GetMetric()
		if metric2 < 0.5*metric1 || 2.0*metric1 < metric2 || metric2 < Epsilon {
			// Reset the simplex.
			s.count = 0
		}
	}

	// If the cache is empty or invalid ...
	if s.count == 0 {
		v := vertices[0]
		v.indexA = 0
		v.indexB = 0
		wALocal := proxyA.GetVertex(0)
		wBLocal := proxyB.GetVertex(0)
		v.wA = MulX(transformA, wALocal)
		v.wB = MulX(transformB, wBLocal)
		v.w = SubVV(v.wB, v.wA)
		s.count = 1
	}
}

func (s *Simplex) WriteCache(cache *SimplexCache) {
	cache.Metric = s.GetMetric()
	cache.Count = uint16(s.count)
	vertices := []*SimplexVertex{&s.v1, &s.v2, &s.v3}
	for i := 0; i < s.count; i++ {
		cache.IndexA[i] = uint8(vertices[i].indexA)
		cache.IndexB[i] = uint8(vertices[i].indexB)
	}
}

func (s *Simplex) GetSearchDirection() Vec2 {
	switch s.count {
	case 1:
		return s.v1.w.Minus()

	case 2:
		e12 := SubVV(s.v2.w, s.v1.w)
		sgn := CrossVV(e12, s.v1.w.Minus())
		if sgn > 0.0 {
			// Origin is left of e12.
			return CrossFV(1.0, e12)
		} else {
			// Origin is right of e12.
			return CrossVF(e12, 1.0)
		}

	default:
		return Vec2_zero
	}
}

func (s *Simplex) GetClosestPoint() Vec2 {
	switch s.count {
	case 0:
		return Vec2_zero

	case 1:
		return s.v1.w

	case 2:
		return AddVV(MulFV(s.v1.a, s.v1.w), MulFV(s.v2.a, s.v2.w))

	case 3:
		return Vec2_zero

	default:
		return Vec2_zero
	}
}

func (s *Simplex) GetWitnessPoints(pA *Vec2, pB *Vec2) {
	switch s.count {
	case 0:
	case 1:
		*pA = s.v1.wA
		*pB = s.v1.wB
	case 2:
		*pA = AddVV(MulFV(s.v1.a, s.v1.wA), MulFV(s.v2.a, s.v2.wA))
		*pB = AddVV(MulFV(s.v1.a, s.v1.wB), MulFV(s.v2.a, s.v2.wB))
	case 3:
		*pA = AddVV(AddVV(MulFV(s.v1.a, s.v1.wA), MulFV(s.v2.a, s.v2.wA)), MulFV(s.v3.a, s.v3.wA))
		*pB = *pA
	default:
	}
}

func (s *Simplex) GetMetric() float64 {
	switch s.count {
	case 0:
		return 0.0
	case 1:
		return 0.0
	case 2:
		return DistanceVV(s.v1.w, s.v2.w)
	case 3:
		return CrossVV(SubVV(s.v2.w, s.v1.w), SubVV(s.v3.w, s.v1.w))
	default:
		return 0.0
	}
}

// Solve a line segment using barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
func (s *Simplex) Solve2() {
	w1 := s.v1.w
	w2 := s.v2.w
	e12 := SubVV(w2, w1)

	// w1 region
	d12_2 := -DotVV(w1, e12)
	if d12_2 <= 0.0 {
		// a2 <= 0, so we clamp it to 0
		s.v1.a = 1.0
		s.count = 1
		return
	}

	// w2 region
	d12_1 := DotVV(w2, e12)
	if d12_1 <= 0.0 {
		// a1 <= 0, so we clamp it to 0
		s.v2.a = 1.0
		s.count = 1
		s.v1 = s.v2
		return
	}

	// Must be in e12 region.
	inv_d12 := 1.0 / (d12_1 + d12_2)
	s.v1.a = d12_1 * inv_d12
	s.v2.a = d12_2 * inv_d12
	s.count = 2
}

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
func (s *Simplex) Solve3() {
	w1 := s.v1.w
	w2 := s.v2.w
	w3 := s.v3.w

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	e12 := SubVV(w2, w1)
	w1e12 := DotVV(w1, e12)
	w2e12 := DotVV(w2, e12)
	d12_1 := w2e12
	d12_2 := -w1e12

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	e13 := SubVV(w3, w1)
	w1e13 := DotVV(w1, e13)
	w3e13 := DotVV(w3, e13)
	d13_1 := w3e13
	d13_2 := -w1e13

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	e23 := SubVV(w3, w2)
	w2e23 := DotVV(w2, e23)
	w3e23 := DotVV(w3, e23)
	d23_1 := w3e23
	d23_2 := -w2e23

	// Triangle123
	n123 := CrossVV(e12, e13)

	d123_1 := n123 * CrossVV(w2, w3)
	d123_2 := n123 * CrossVV(w3, w1)
	d123_3 := n123 * CrossVV(w1, w2)

	// w1 region
	if d12_2 <= 0.0 && d13_2 <= 0.0 {
		s.v1.a = 1.0
		s.count = 1
		return
	}

	// e12
	if d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0 {
		inv_d12 := 1.0 / (d12_1 + d12_2)
		s.v1.a = d12_1 * inv_d12
		s.v2.a = d12_2 * inv_d12
		s.count = 2
		return
	}

	// e13
	if d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0 {
		inv_d13 := 1.0 / (d13_1 + d13_2)
		s.v1.a = d13_1 * inv_d13
		s.v3.a = d13_2 * inv_d13
		s.count = 2
		s.v2 = s.v3
		return
	}

	// w2 region
	if d12_1 <= 0.0 && d23_2 <= 0.0 {
		s.v2.a = 1.0
		s.count = 1
		s.v1 = s.v2
		return
	}

	// w3 region
	if d13_1 <= 0.0 && d23_1 <= 0.0 {
		s.v3.a = 1.0
		s.count = 1
		s.v1 = s.v3
		return
	}

	// e23
	if d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0 {
		inv_d23 := 1.0 / (d23_1 + d23_2)
		s.v2.a = d23_1 * inv_d23
		s.v3.a = d23_2 * inv_d23
		s.count = 2
		s.v1 = s.v3
		return
	}

	// Must be in triangle123
	inv_d123 := 1.0 / (d123_1 + d123_2 + d123_3)
	s.v1.a = d123_1 * inv_d123
	s.v2.a = d123_2 * inv_d123
	s.v3.a = d123_3 * inv_d123
	s.count = 3
}

// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
var GjkCalls, GjkIters, GjkMaxIters int

// Compute the closest points between two shapes. Supports any combination of:
// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
// On the first call set b2SimplexCache.count to zero.
func Distance(cache *SimplexCache, input *DistanceInput) (output DistanceOutput) {
	GjkCalls++

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	transformA := input.TransformA
	transformB := input.TransformB

	// Initialize the simplex.
	var simplex Simplex
	simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB)

	// Get simplex vertices as an array.
	vertices := []*SimplexVertex{&simplex.v1, &simplex.v2, &simplex.v3}
	const k_maxIters int = 20

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	var saveA, saveB [3]int
	saveCount := 0

	closestPoint := simplex.GetClosestPoint()
	distanceSqr1 := closestPoint.LengthSquared()
	distanceSqr2 := distanceSqr1

	// Main iteration loop.
	iter := 0
	for iter < k_maxIters {
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.count
		for i := 0; i < saveCount; i++ {
			saveA[i] = vertices[i].indexA
			saveB[i] = vertices[i].indexB
		}

		switch simplex.count {
		case 1:
		case 2:
			simplex.Solve2()
		case 3:
			simplex.Solve3()
		default:
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if simplex.count == 3 {
			break
		}

		// Compute closest point.
		p := simplex.GetClosestPoint()
		distanceSqr2 = p.LengthSquared()

		// Ensure progress
		if distanceSqr2 >= distanceSqr1 {
			//break;
		}
		distanceSqr1 = distanceSqr2

		// Get search direction.
		d := simplex.GetSearchDirection()

		// Ensure the search direction is numerically fit.
		if d.LengthSquared() < Epsilon*Epsilon {
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break
		}

		// Compute a tentative new simplex vertex using support points.
		vertex := vertices[simplex.count]
		vertex.indexA = proxyA.GetSupport(MulTRV(transformA.Q, d.Minus()))
		vertex.wA = MulX(transformA, proxyA.GetVertex(vertex.indexA))
		//var wBLocal Vec2
		vertex.indexB = proxyB.GetSupport(MulTRV(transformB.Q, d))
		vertex.wB = MulX(transformB, proxyB.GetVertex(vertex.indexB))
		vertex.w = SubVV(vertex.wB, vertex.wA)

		// Iteration count is equated to the number of support point calls.
		iter++
		GjkIters++

		// Check for duplicate support points. This is the main termination criteria.
		duplicate := false
		for i := 0; i < saveCount; i++ {
			if vertex.indexA == saveA[i] && vertex.indexB == saveB[i] {
				duplicate = true
				break
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if duplicate {
			break
		}

		// New vertex is ok and needed.
		simplex.count++
	}

	GjkMaxIters = MaxI(GjkMaxIters, iter)

	// Prepare output.
	simplex.GetWitnessPoints(&output.PointA, &output.PointB)
	output.Distance = DistanceVV(output.PointA, output.PointB)
	output.Iterations = iter

	// Cache the simplex.
	simplex.WriteCache(cache)

	// Apply radii if requested.
	if input.UseRadii {
		rA := proxyA.Radius
		rB := proxyB.Radius

		if output.Distance > rA+rB && output.Distance > Epsilon {
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output.Distance -= rA + rB
			normal := SubVV(output.PointB, output.PointA)
			normal.Normalize()
			output.PointA.Add(MulFV(rA, normal))
			output.PointB.Sub(MulFV(rB, normal))
		} else {
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			p := MulFV(0.5, AddVV(output.PointA, output.PointB))
			output.PointA = p
			output.PointB = p
			output.Distance = 0.0
		}
	}

	return
}
