package box2d

// This holds the mass data computed for a shape.
type MassData struct {
	// The mass of the shape, usually in kilograms.
	Mass float64

	// The position of the shape's centroid relative to the shape's origin.
	Center Vec2

	// The rotational inertia of the shape about the local origin.
	I float64
}

type ShapeType byte

const (
	Shape_e_circle    ShapeType = 0
	Shape_e_edge      ShapeType = 1
	Shape_e_polygon   ShapeType = 2
	Shape_e_chain     ShapeType = 3
	Shape_e_typeCount ShapeType = 4
)

type IShape interface {
	// Clone the concrete shape.
	Clone() IShape

	// Get the type of this shape. You can use this to down cast to the concrete shape.
	// @return the shape type.
	GetType() ShapeType

	//
	GetRadius() float64

	// Get the number of child primitives.
	GetChildCount() int

	// Test a point for containment in this shape. This only works for convex shapes.
	// @param xf the shape world transform.
	// @param p a point in world coordinates.
	TestPoint(xf Transform, p Vec2) bool

	// Cast a ray against a child shape.
	// @param output the ray-cast results.
	// @param input the ray-cast input parameters.
	// @param transform the transform to be applied to the shape.
	// @param childIndex the child shape index
	RayCast(input RayCastInput, xf Transform, childIndex int) (output RayCastOutput, ret bool)

	// Given a transform, compute the associated axis aligned bounding box for a child shape.
	// @param aabb returns the axis aligned box.
	// @param xf the world transform of the shape.
	// @param childIndex the child shape
	ComputeAABB(aabb *AABB, xf Transform, childIndex int)

	// Compute the mass properties of this shape using its dimensions and density.
	// The inertia tensor is computed about the local origin.
	// @param massData returns the mass data for this shape.
	// @param density the density in kilograms per meter squared.
	ComputeMass(massData *MassData, density float64)
}

type Shape struct {
	Type   ShapeType
	Radius float64
}

// Get the type of this shape. You can use this to down cast to the concrete shape.
// @return the shape type.
func (s *Shape) GetType() ShapeType {
	return s.Type
}

func (s *Shape) GetRadius() float64 {
	return s.Radius
}

// A circle shape.
type CircleShape struct {
	Shape
	P Vec2
}

func MakeCircleShape() CircleShape {
	cs := CircleShape{}
	cs.Type = Shape_e_circle
	return cs
}

func NewCircleShape() *CircleShape {
	cs := MakeCircleShape()
	return &cs
}

func (cs *CircleShape) Clone() IShape {
	clone := NewCircleShape()
	*clone = *cs
	return clone
}

// @see b2Shape::GetChildCount
func (cs *CircleShape) GetChildCount() int {
	return 1
}

// Implement b2Shape.
func (cs *CircleShape) TestPoint(transform Transform, p Vec2) bool {
	center := AddVV(transform.P, MulRV(transform.Q, cs.P))
	d := SubVV(p, center)
	return DotVV(d, d) <= cs.Radius*cs.Radius
}

// Implement b2Shape.
func (cs *CircleShape) RayCast(input RayCastInput, transform Transform, childIndex int) (output RayCastOutput, ret bool) {
	position := AddVV(transform.P, MulRV(transform.Q, cs.P))
	s := SubVV(input.P1, position)
	b := DotVV(s, s) - cs.Radius*cs.Radius

	// Solve quadratic equation.
	r := SubVV(input.P2, input.P1)
	c := DotVV(s, r)
	rr := DotVV(r, r)
	sigma := c*c - rr*b

	// Check for negative discriminant and short segment.
	if sigma < 0.0 || rr < Epsilon {
		return
	}

	// Find the point of intersection of the line with the circle.
	a := -(c + Sqrt(sigma))

	// Is the intersection point on the segment?
	if 0.0 <= a && a <= input.MaxFraction*rr {
		a /= rr
		output.Fraction = a
		output.Normal = AddVV(s, MulFV(a, r))
		output.Normal.Normalize()
		ret = true
		return
	}

	return
}

// @see b2Shape::ComputeAABB
func (cs *CircleShape) ComputeAABB(aabb *AABB, transform Transform, childIndex int) {
	p := AddVV(transform.P, MulRV(transform.Q, cs.P))
	aabb.LowerBound.Set(p.X-cs.Radius, p.Y-cs.Radius)
	aabb.UpperBound.Set(p.X+cs.Radius, p.Y+cs.Radius)
}

// @see b2Shape::ComputeMass
func (cs *CircleShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = density * Pi * cs.Radius * cs.Radius
	massData.Center = cs.P

	// inertia about the local origin
	massData.I = massData.Mass * (0.5*cs.Radius*cs.Radius + DotVV(cs.P, cs.P))
}

// Get the supporting vertex index in the given direction.
func (cs *CircleShape) GetSupport(d *Vec2) int {
	return 0
}

// Get the supporting vertex in the given direction.
func (cs *CircleShape) GetSupportVertex(d Vec2) Vec2 {
	return cs.P
}

// Get the vertex count.
func (cs *CircleShape) GetVertexCount() int {
	return 1
}

// Get a vertex by index. Used by b2Distance.
func (cs *CircleShape) GetVertex(index int) Vec2 {
	return cs.P
}

// A line segment (edge) shape. These can be connected in chains or loops
// to other edge shapes. The connectivity information is used to ensure
// correct contact normals.
type EdgeShape struct {
	Shape

	// These are the edge vertices
	Vertex1, Vertex2 Vec2

	// Optional adjacent vertices. These are used for smooth collision.
	Vertex0, Vertex3       Vec2
	HasVertex0, HasVertex3 bool
}

func MakeEdgeShape() EdgeShape {
	es := EdgeShape{}
	es.Type = Shape_e_edge
	es.Radius = PolygonRadius
	return es
}

func NewEdgeShape() *EdgeShape {
	es := MakeEdgeShape()
	return &es
}

func (es *EdgeShape) Set(v1, v2 Vec2) {
	es.Vertex1 = v1
	es.Vertex2 = v2
	es.HasVertex0 = false
	es.HasVertex3 = false
}

func (es *EdgeShape) Clone() IShape {
	clone := NewEdgeShape()
	*clone = *es
	return clone
}

// @see b2Shape::GetChildCount
func (es *EdgeShape) GetChildCount() int {
	return 1
}

// @see b2Shape::TestPoint
func (es *EdgeShape) TestPoint(xf Transform, p Vec2) bool {
	return false
}

// Implement b2Shape.
func (es *EdgeShape) RayCast(input RayCastInput, xf Transform, childIndex int) (output RayCastOutput, ret bool) {
	// Put the ray into the edge's frame of reference.
	p1 := MulTRV(xf.Q, SubVV(input.P1, xf.P))
	p2 := MulTRV(xf.Q, SubVV(input.P2, xf.P))
	d := SubVV(p2, p1)

	v1 := es.Vertex1
	v2 := es.Vertex2
	e := SubVV(v2, v1)
	normal := Vec2{e.Y, -e.X}
	normal.Normalize()

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	numerator := DotVV(normal, SubVV(v1, p1))
	denominator := DotVV(normal, d)

	if denominator == 0.0 {
		return
	}

	t := numerator / denominator
	if t < 0.0 || input.MaxFraction < t {
		return
	}

	q := AddVV(p1, MulFV(t, d))

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	r := SubVV(v2, v1)
	rr := DotVV(r, r)
	if rr == 0.0 {
		return
	}

	s := DotVV(SubVV(q, v1), r) / rr
	if s < 0.0 || 1.0 < s {
		return
	}

	output.Fraction = t
	if numerator > 0.0 {
		output.Normal = normal.Minus()
	} else {
		output.Normal = normal
	}
	ret = true
	return
}

// @see b2Shape::ComputeAABB
func (es *EdgeShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int) {
	v1 := MulX(xf, es.Vertex1)
	v2 := MulX(xf, es.Vertex2)

	lower := MinV(v1, v2)
	upper := MaxV(v1, v2)

	r := Vec2{es.Radius, es.Radius}
	aabb.LowerBound = SubVV(lower, r)
	aabb.UpperBound = AddVV(upper, r)
}

// @see b2Shape::ComputeMass
func (es *EdgeShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = 0.0
	massData.Center = MulFV(0.5, AddVV(es.Vertex1, es.Vertex2))
	massData.I = 0.0
}

// A convex polygon. It is assumed that the interior of the polygon is to
// the left of each edge.
// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
// In most cases you should not need many vertices for a convex polygon.
type PolygonShape struct {
	Shape
	Centroid    Vec2
	Vertices    [MaxPolygonVertices]Vec2
	Normals     [MaxPolygonVertices]Vec2
	VertexCount int
}

func MakePolygonShape() PolygonShape {
	ps := PolygonShape{}
	ps.Type = Shape_e_polygon
	ps.Radius = PolygonRadius
	return ps
}

func NewPolygonShape() *PolygonShape {
	ps := MakePolygonShape()
	return &ps
}

func (ps *PolygonShape) Clone() IShape {
	clone := NewPolygonShape()
	*clone = *ps
	return clone
}

// @see b2Shape::GetChildCount
func (ps *PolygonShape) GetChildCount() int {
	return 1
}

func computeCentroid(vs []Vec2, count int) Vec2 {
	c := Vec2{0.0, 0.0}
	area := 0.0

	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	pRef := Vec2{0.0, 0.0}

	/*
	   #if 0
	   	// This code would put the reference point inside the polygon.
	   	for (int i = 0; i < count; ++i)
	   	{
	   		pRef += vs[i];
	   	}
	   	pRef *= 1.0f / count;
	   #endif
	*/

	const inv3 float64 = 1.0 / 3.0

	for i := 0; i < count; i++ {
		// Triangle vertices.
		p1 := pRef
		p2 := vs[i]
		p3 := vs[0]
		if i+1 < count {
			p3 = vs[i+1]
		}

		e1 := SubVV(p2, p1)
		e2 := SubVV(p3, p1)

		D := CrossVV(e1, e2)

		triangleArea := 0.5 * D
		area += triangleArea

		// Area weighted centroid
		c.Add(MulFV(triangleArea*inv3, AddVV(AddVV(p1, p2), p3)))
	}

	// Centroid
	c.Mul(1.0 / area)
	return c
}

// Copy vertices. This assumes the vertices define a convex polygon.
// It is assumed that the exterior is the the right of each edge.
// The count must be in the range [3, b2_maxPolygonVertices].
func (ps *PolygonShape) Set(vertices []Vec2) {
	ps.VertexCount = len(vertices)

	// Copy vertices.
	for i, v := range vertices {
		ps.Vertices[i] = v
	}

	// Compute normals. Ensure the edges have non-zero length.
	for i := 0; i < ps.VertexCount; i++ {
		i1 := i
		i2 := 0
		if i+1 < ps.VertexCount {
			i2 = i + 1
		}
		edge := SubVV(ps.Vertices[i2], ps.Vertices[i1])
		ps.Normals[i] = CrossVF(edge, 1.0)
		ps.Normals[i].Normalize()
	}
	/*
	   #ifdef _DEBUG
	   	// Ensure the polygon is convex and the interior
	   	// is to the left of each edge.
	   	for (int i = 0; i < m_vertexCount; ++i)
	   	{
	   		int i1 = i;
	   		int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
	   		b2Vec2 edge = m_vertices[i2] - m_vertices[i1];

	   		for (int j = 0; j < m_vertexCount; ++j)
	   		{
	   			// Don't check vertices on the current edge.
	   			if (j == i1 || j == i2)
	   			{
	   				continue;
	   			}

	   			b2Vec2 r = m_vertices[j] - m_vertices[i1];

	   			// If this crashes, your polygon is non-convex, has colinear edges,
	   			// or the winding order is wrong.
	   			float64 s = b2Cross(edge, r);
	   			b2Assert(s > 0.0f && "ERROR: Please ensure your polygon is convex and has a CCW winding order");
	   		}
	   	}
	   #endif
	*/

	// Compute the polygon centroid.
	ps.Centroid = computeCentroid(ps.Vertices[:], ps.VertexCount)
}

// Build vertices to represent an axis-aligned box.
// @param hx the half-width.
// @param hy the half-height.
func (ps *PolygonShape) SetAsBox(hx float64, hy float64) {
	ps.VertexCount = 4
	ps.Vertices[0].Set(-hx, -hy)
	ps.Vertices[1].Set(hx, -hy)
	ps.Vertices[2].Set(hx, hy)
	ps.Vertices[3].Set(-hx, hy)
	ps.Normals[0].Set(0.0, -1.0)
	ps.Normals[1].Set(1.0, 0.0)
	ps.Normals[2].Set(0.0, 1.0)
	ps.Normals[3].Set(-1.0, 0.0)
	ps.Centroid.SetZero()
}

// Build vertices to represent an oriented box.
// @param hx the half-width.
// @param hy the half-height.
// @param center the center of the box in local coordinates.
// @param angle the rotation of the box in local coordinates.
func (ps *PolygonShape) SetAsOrientedBox(hx float64, hy float64, center Vec2, angle float64) {
	ps.VertexCount = 4
	ps.Vertices[0].Set(-hx, -hy)
	ps.Vertices[1].Set(hx, -hy)
	ps.Vertices[2].Set(hx, hy)
	ps.Vertices[3].Set(-hx, hy)
	ps.Normals[0].Set(0.0, -1.0)
	ps.Normals[1].Set(1.0, 0.0)
	ps.Normals[2].Set(0.0, 1.0)
	ps.Normals[3].Set(-1.0, 0.0)
	ps.Centroid = center

	var xf Transform
	xf.P = center
	xf.Q.Set(angle)

	// Transform vertices and normals.
	for i := 0; i < ps.VertexCount; i++ {
		ps.Vertices[i] = MulX(xf, ps.Vertices[i])
		ps.Normals[i] = MulRV(xf.Q, ps.Normals[i])
	}
}

// @see b2Shape::TestPoint
func (ps *PolygonShape) TestPoint(xf Transform, p Vec2) bool {
	pLocal := MulTRV(xf.Q, SubVV(p, xf.P))

	for i := 0; i < ps.VertexCount; i++ {
		dot := DotVV(ps.Normals[i], SubVV(pLocal, ps.Vertices[i]))
		if dot > 0.0 {
			return false
		}
	}

	return true
}

// Implement b2Shape.
func (ps *PolygonShape) RayCast(input RayCastInput, xf Transform, childIndex int) (output RayCastOutput, ret bool) {
	// Put the ray into the polygon's frame of reference.
	p1 := MulTRV(xf.Q, SubVV(input.P1, xf.P))
	p2 := MulTRV(xf.Q, SubVV(input.P2, xf.P))
	d := SubVV(p2, p1)

	lower, upper := 0.0, input.MaxFraction

	index := -1

	for i := 0; i < ps.VertexCount; i++ {
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		numerator := DotVV(ps.Normals[i], SubVV(ps.Vertices[i], p1))
		denominator := DotVV(ps.Normals[i], d)

		if denominator == 0.0 {
			if numerator < 0.0 {
				return
			}
		} else {
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if denominator < 0.0 && numerator < lower*denominator {
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator
				index = i
			} else if denominator > 0.0 && numerator < upper*denominator {
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - b2_epsilon)
		if upper < lower {
			return
		}
	}

	if index >= 0 {
		output.Fraction = lower
		output.Normal = MulRV(xf.Q, ps.Normals[index])
		ret = true
		return
	}

	return
}

// @see b2Shape::ComputeAABB
func (ps *PolygonShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int) {

	lower := MulX(xf, ps.Vertices[0])
	upper := lower

	for i := 1; i < ps.VertexCount; i++ {
		v := MulX(xf, ps.Vertices[i])
		lower = MinV(lower, v)
		upper = MaxV(upper, v)
	}

	r := Vec2{ps.Radius, ps.Radius}
	aabb.LowerBound = SubVV(lower, r)
	aabb.UpperBound = AddVV(upper, r)
}

// @see b2Shape::ComputeMass
func (ps *PolygonShape) ComputeMass(massData *MassData, density float64) {
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	//b2Assert(m_vertexCount >= 3);
	center := Vec2{0.0, 0.0}
	area := 0.0
	I := 0.0

	// s is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	s := Vec2{0.0, 0.0}

	// This code would put the reference point inside the polygon.
	for i := 0; i < ps.VertexCount; i++ {
		s.Add(ps.Vertices[i])
	}
	s.Mul(1.0 / float64(ps.VertexCount))

	k_inv3 := 1.0 / 3.0

	for i := 0; i < ps.VertexCount; i++ {
		// Triangle vertices.
		e1 := SubVV(ps.Vertices[i], s)
		var e2 Vec2
		if i+1 < ps.VertexCount {
			e2 = SubVV(ps.Vertices[i+1], s)
		} else {
			e2 = SubVV(ps.Vertices[0], s)
		}

		D := CrossVV(e1, e2)

		triangleArea := 0.5 * D
		area += triangleArea

		// Area weighted centroid
		center.Add(MulFV(triangleArea*k_inv3, AddVV(e1, e2)))

		ex1, ey1 := e1.X, e1.Y
		ex2, ey2 := e2.X, e2.Y

		intx2 := ex1*ex1 + ex2*ex1 + ex2*ex2
		inty2 := ey1*ey1 + ey2*ey1 + ey2*ey2

		I += (0.25 * k_inv3 * D) * (intx2 + inty2)
	}

	// Total mass
	massData.Mass = density * area

	// Center of mass
	center.Mul(1.0 / area)
	massData.Center = AddVV(center, s)

	// Inertia tensor relative to the local origin (point s).
	massData.I = density * I

	// Shift to center of mass then to original body origin.
	massData.I += massData.Mass * (DotVV(massData.Center, massData.Center) - DotVV(center, center))
}

// Get the vertex count.
func (ps *PolygonShape) GetVertexCount() int {
	return ps.VertexCount
}

// Get a vertex by index.
func (ps *PolygonShape) GetVertex(index int) Vec2 {
	return ps.Vertices[index]
}

// A chain shape is a free form sequence of line segments.
// The chain has two-sided collision, so you can use inside and outside collision.
// Therefore, you may use any winding order.
// Since there may be many vertices, they are allocated using b2Alloc.
// Connectivity information is used to create smooth collisions.
// WARNING: The chain will not collide properly if there are self-intersections.
type ChainShape struct {
	Shape

	// The vertices. Owned by this class.
	Vertices []Vec2

	// The vertex count.
	Count int

	PrevVertex, NextVertex       Vec2
	HasPrevVertex, HasNextVertex bool
}

func MakeChainShape() ChainShape {
	cs := ChainShape{}
	cs.Type = Shape_e_chain
	cs.Radius = PolygonRadius
	return cs
}

func NewChainShape() *ChainShape {
	cs := MakeChainShape()
	return &cs
}

func (cs *ChainShape) Clone() IShape {
	clone := NewChainShape()
	clone.CreateChain(cs.Vertices[:cs.Count])
	clone.PrevVertex = cs.PrevVertex
	clone.NextVertex = cs.NextVertex
	clone.HasPrevVertex = cs.HasPrevVertex
	clone.HasNextVertex = cs.HasNextVertex
	return clone
}

// Create a loop. This automatically adjusts connectivity.
// @param vertices an array of vertices, these are copied
// @param count the vertex count
func (cs *ChainShape) CreateLoop(vertices []Vec2) {
	count := len(vertices)
	cs.Count = count + 1
	cs.Vertices = make([]Vec2, cs.Count)
	copy(cs.Vertices, vertices)
	cs.Vertices[count] = cs.Vertices[0]
	cs.PrevVertex = cs.Vertices[cs.Count-2]
	cs.NextVertex = cs.Vertices[1]
	cs.HasPrevVertex = true
	cs.HasNextVertex = true
}

// Create a chain with isolated end vertices.
// @param vertices an array of vertices, these are copied
// @param count the vertex count
func (cs *ChainShape) CreateChain(vertices []Vec2) {
	cs.Count = len(vertices)
	cs.Vertices = make([]Vec2, cs.Count)
	copy(cs.Vertices, vertices)
	cs.HasPrevVertex = false
	cs.HasNextVertex = false
}

// Establish connectivity to a vertex that precedes the first vertex.
// Don't call this for loops.
func (cs *ChainShape) SetPrevVertex(prevVertex Vec2) {
	cs.PrevVertex = prevVertex
	cs.HasPrevVertex = true
}

// Establish connectivity to a vertex that follows the last vertex.
// Don't call this for loops.
func (cs *ChainShape) SetNextVertex(nextVertex Vec2) {
	cs.NextVertex = nextVertex
	cs.HasNextVertex = true
}

// @see b2Shape::GetChildCount
func (cs *ChainShape) GetChildCount() int {
	return cs.Count - 1
}

// Get a child edge.
func (cs *ChainShape) GetChildEdge(index int) *EdgeShape {
	edge := NewEdgeShape()

	edge.Type = Shape_e_edge
	edge.Radius = cs.Radius

	edge.Vertex1 = cs.Vertices[index+0]
	edge.Vertex2 = cs.Vertices[index+1]

	if index > 0 {
		edge.Vertex0 = cs.Vertices[index-1]
		edge.HasVertex0 = true
	} else {
		edge.Vertex0 = cs.PrevVertex
		edge.HasVertex0 = cs.HasPrevVertex
	}

	if index < cs.Count-2 {
		edge.Vertex3 = cs.Vertices[index+2]
		edge.HasVertex3 = true
	} else {
		edge.Vertex3 = cs.NextVertex
		edge.HasVertex3 = cs.HasNextVertex
	}

	return edge
}

// This always return false.
// @see b2Shape::TestPoint
func (cs *ChainShape) TestPoint(xf Transform, p Vec2) bool {
	return false
}

// Implement b2Shape.
func (cs *ChainShape) RayCast(input RayCastInput, xf Transform, childIndex int) (output RayCastOutput, ret bool) {
	edgeShape := NewEdgeShape()

	i1 := childIndex
	i2 := childIndex + 1
	if i2 == cs.Count {
		i2 = 0
	}

	edgeShape.Vertex1 = cs.Vertices[i1]
	edgeShape.Vertex2 = cs.Vertices[i2]

	return edgeShape.RayCast(input, xf, 0)
}

// @see b2Shape::ComputeAABB
func (cs *ChainShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int) {
	i1 := childIndex
	i2 := childIndex + 1
	if i2 == cs.Count {
		i2 = 0
	}

	v1 := MulX(xf, cs.Vertices[i1])
	v2 := MulX(xf, cs.Vertices[i2])

	aabb.LowerBound = MinV(v1, v2)
	aabb.UpperBound = MaxV(v1, v2)
}

// Chains have zero mass.
// @see b2Shape::ComputeMass
func (cs *ChainShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = 0.0
	massData.Center.SetZero()
	massData.I = 0.0
}
