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
	Shape_e_circle    = 0
	Shape_e_edge      = 1
	Shape_e_polygon   = 2
	Shape_e_chain     = 3
	Shape_e_typeCount = 4
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
	GetChildCount() int32

	// Test a point for containment in this shape. This only works for convex shapes.
	// @param xf the shape world transform.
	// @param p a point in world coordinates.
	TestPoint(xf Transform, p Vec2) bool

	// Cast a ray against a child shape.
	// @param output the ray-cast results.
	// @param input the ray-cast input parameters.
	// @param transform the transform to be applied to the shape.
	// @param childIndex the child shape index
	RayCast(input RayCastInput, xf Transform, childIndex int32) (output RayCastOutput, ret bool)

	// Given a transform, compute the associated axis aligned bounding box for a child shape.
	// @param aabb returns the axis aligned box.
	// @param xf the world transform of the shape.
	// @param childIndex the child shape
	ComputeAABB(aabb *AABB, xf Transform, childIndex int32)

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
func (this *Shape) GetType() ShapeType {
	return this.Type
}

func (this *Shape) GetRadius() float64 {
	return this.Radius
}

// A circle shape.
type CircleShape struct {
	Shape
	P Vec2
}

func NewCircleShape() *CircleShape {
	this := new(CircleShape)
	this.Type = Shape_e_circle
	return this
}

func (this *CircleShape) Clone() IShape {
	clone := NewCircleShape()
	*clone = *this
	return clone
}

// @see b2Shape::GetChildCount
func (this *CircleShape) GetChildCount() int32 {
	return 1
}

// Implement b2Shape.
func (this *CircleShape) TestPoint(transform Transform, p Vec2) bool {
	center := AddVV(transform.P, MulRV(transform.Q, this.P))
	d := SubVV(p, center)
	return DotVV(d, d) <= this.Radius*this.Radius
}

// Implement b2Shape.
func (this *CircleShape) RayCast(input RayCastInput, transform Transform, childIndex int32) (output RayCastOutput, ret bool) {
	position := AddVV(transform.P, MulRV(transform.Q, this.P))
	s := SubVV(input.P1, position)
	b := DotVV(s, s) - this.Radius*this.Radius

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
func (this *CircleShape) ComputeAABB(aabb *AABB, transform Transform, childIndex int32) {
	p := AddVV(transform.P, MulRV(transform.Q, this.P))
	aabb.LowerBound.Set(p.X-this.Radius, p.Y-this.Radius)
	aabb.UpperBound.Set(p.X+this.Radius, p.Y+this.Radius)
}

// @see b2Shape::ComputeMass
func (this *CircleShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = density * Pi * this.Radius * this.Radius
	massData.Center = this.P

	// inertia about the local origin
	massData.I = massData.Mass * (0.5*this.Radius*this.Radius + DotVV(this.P, this.P))
}

// Get the supporting vertex index in the given direction.
func (this *CircleShape) GetSupport(d *Vec2) int32 {
	return 0
}

// Get the supporting vertex in the given direction.
func (this *CircleShape) GetSupportVertex(d Vec2) Vec2 {
	return this.P
}

// Get the vertex count.
func (this *CircleShape) GetVertexCount() int32 {
	return 1
}

// Get a vertex by index. Used by b2Distance.
func (this *CircleShape) GetVertex(index int32) Vec2 {
	return this.P
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

func (this *EdgeShape) Set(v1, v2 Vec2) {
	this.Vertex1 = v1
	this.Vertex2 = v2
	this.HasVertex0 = false
	this.HasVertex3 = false
}

func (this *EdgeShape) Clone() IShape {
	clone := NewEdgeShape()
	*clone = *this
	return clone
}

// @see b2Shape::GetChildCount
func (this *EdgeShape) GetChildCount() int32 {
	return 1
}

// @see b2Shape::TestPoint
func (this *EdgeShape) TestPoint(xf Transform, p Vec2) bool {
	return false
}

// Implement b2Shape.
func (this *EdgeShape) RayCast(input RayCastInput, xf Transform, childIndex int32) (output RayCastOutput, ret bool) {
	// Put the ray into the edge's frame of reference.
	p1 := MulTRV(xf.Q, SubVV(input.P1, xf.P))
	p2 := MulTRV(xf.Q, SubVV(input.P2, xf.P))
	d := SubVV(p2, p1)

	v1 := this.Vertex1
	v2 := this.Vertex2
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
func (this *EdgeShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int32) {
	v1 := MulX(xf, this.Vertex1)
	v2 := MulX(xf, this.Vertex2)

	lower := MinV(v1, v2)
	upper := MaxV(v1, v2)

	r := Vec2{this.Radius, this.Radius}
	aabb.LowerBound = SubVV(lower, r)
	aabb.UpperBound = AddVV(upper, r)
}

// @see b2Shape::ComputeMass
func (this *EdgeShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = 0.0
	massData.Center = MulFV(0.5, AddVV(this.Vertex1, this.Vertex2))
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
	VertexCount int32
}

func NewPolygonShape() *PolygonShape {
	this := new(PolygonShape)
	this.Type = Shape_e_polygon
	this.Radius = PolygonRadius
	return this
}

func (this *PolygonShape) Clone() IShape {
	clone := NewPolygonShape()
	*clone = *this
	return clone
}

// @see b2Shape::GetChildCount
func (this *PolygonShape) GetChildCount() int32 {
	return 1
}

func computeCentroid(vs []Vec2, count int32) Vec2 {
	c := Vec2{0.0, 0.0}
	area := 0.0

	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	pRef := Vec2{0.0, 0.0}

	/*
	   #if 0
	   	// This code would put the reference point inside the polygon.
	   	for (int32 i = 0; i < count; ++i)
	   	{
	   		pRef += vs[i];
	   	}
	   	pRef *= 1.0f / count;
	   #endif
	*/

	const inv3 float64 = 1.0 / 3.0

	for i := int32(0); i < count; i++ {
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
func (this *PolygonShape) Set(vertices []Vec2) {
	this.VertexCount = int32(len(vertices))

	// Copy vertices.
	for i, v := range vertices {
		this.Vertices[i] = v
	}

	// Compute normals. Ensure the edges have non-zero length.
	for i := int32(0); i < this.VertexCount; i++ {
		i1 := i
		i2 := int32(0)
		if i+1 < this.VertexCount {
			i2 = i + 1
		}
		edge := SubVV(this.Vertices[i2], this.Vertices[i1])
		this.Normals[i] = CrossVF(edge, 1.0)
		this.Normals[i].Normalize()
	}
	/*
	   #ifdef _DEBUG
	   	// Ensure the polygon is convex and the interior
	   	// is to the left of each edge.
	   	for (int32 i = 0; i < m_vertexCount; ++i)
	   	{
	   		int32 i1 = i;
	   		int32 i2 = i + 1 < m_vertexCount ? i + 1 : 0;
	   		b2Vec2 edge = m_vertices[i2] - m_vertices[i1];

	   		for (int32 j = 0; j < m_vertexCount; ++j)
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
	this.Centroid = computeCentroid(this.Vertices[:], this.VertexCount)
}

// Build vertices to represent an axis-aligned box.
// @param hx the half-width.
// @param hy the half-height.
func (this *PolygonShape) SetAsBox(hx float64, hy float64) {
	this.VertexCount = 4
	this.Vertices[0].Set(-hx, -hy)
	this.Vertices[1].Set(hx, -hy)
	this.Vertices[2].Set(hx, hy)
	this.Vertices[3].Set(-hx, hy)
	this.Normals[0].Set(0.0, -1.0)
	this.Normals[1].Set(1.0, 0.0)
	this.Normals[2].Set(0.0, 1.0)
	this.Normals[3].Set(-1.0, 0.0)
	this.Centroid.SetZero()
}

// Build vertices to represent an oriented box.
// @param hx the half-width.
// @param hy the half-height.
// @param center the center of the box in local coordinates.
// @param angle the rotation of the box in local coordinates.
func (this *PolygonShape) SetAsOrientedBox(hx float64, hy float64, center Vec2, angle float64) {
	this.VertexCount = 4
	this.Vertices[0].Set(-hx, -hy)
	this.Vertices[1].Set(hx, -hy)
	this.Vertices[2].Set(hx, hy)
	this.Vertices[3].Set(-hx, hy)
	this.Normals[0].Set(0.0, -1.0)
	this.Normals[1].Set(1.0, 0.0)
	this.Normals[2].Set(0.0, 1.0)
	this.Normals[3].Set(-1.0, 0.0)
	this.Centroid = center

	var xf Transform
	xf.P = center
	xf.Q.Set(angle)

	// Transform vertices and normals.
	for i := int32(0); i < this.VertexCount; i++ {
		this.Vertices[i] = MulX(xf, this.Vertices[i])
		this.Normals[i] = MulRV(xf.Q, this.Normals[i])
	}
}

// @see b2Shape::TestPoint
func (this *PolygonShape) TestPoint(xf Transform, p Vec2) bool {
	pLocal := MulTRV(xf.Q, SubVV(p, xf.P))

	for i := int32(0); i < this.VertexCount; i++ {
		dot := DotVV(this.Normals[i], SubVV(pLocal, this.Vertices[i]))
		if dot > 0.0 {
			return false
		}
	}

	return true
}

// Implement b2Shape.
func (this *PolygonShape) RayCast(input RayCastInput, xf Transform, childIndex int32) (output RayCastOutput, ret bool) {
	// Put the ray into the polygon's frame of reference.
	p1 := MulTRV(xf.Q, SubVV(input.P1, xf.P))
	p2 := MulTRV(xf.Q, SubVV(input.P2, xf.P))
	d := SubVV(p2, p1)

	lower, upper := 0.0, input.MaxFraction

	index := int32(-1)

	for i := int32(0); i < this.VertexCount; i++ {
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		numerator := DotVV(this.Normals[i], SubVV(this.Vertices[i], p1))
		denominator := DotVV(this.Normals[i], d)

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
		output.Normal = MulRV(xf.Q, this.Normals[index])
		ret = true
		return
	}

	return
}

// @see b2Shape::ComputeAABB
func (this *PolygonShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int32) {

	lower := MulX(xf, this.Vertices[0])
	upper := lower

	for i := int32(1); i < this.VertexCount; i++ {
		v := MulX(xf, this.Vertices[i])
		lower = MinV(lower, v)
		upper = MaxV(upper, v)
	}

	r := Vec2{this.Radius, this.Radius}
	aabb.LowerBound = SubVV(lower, r)
	aabb.UpperBound = AddVV(upper, r)
}

// @see b2Shape::ComputeMass
func (this *PolygonShape) ComputeMass(massData *MassData, density float64) {
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int32(dA)
	// centroid.x = (1/mass) * rho * int32(x * dA)
	// centroid.y = (1/mass) * rho * int32(y * dA)
	// I = rho * int32((x*x + y*y) * dA)
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
	for i := int32(0); i < this.VertexCount; i++ {
		s.Add(this.Vertices[i])
	}
	s.Mul(1.0 / float64(this.VertexCount))

	k_inv3 := 1.0 / 3.0

	for i := int32(0); i < this.VertexCount; i++ {
		// Triangle vertices.
		e1 := SubVV(this.Vertices[i], s)
		var e2 Vec2
		if i+1 < this.VertexCount {
			e2 = SubVV(this.Vertices[i+1], s)
		} else {
			e2 = SubVV(this.Vertices[0], s)
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
func (this *PolygonShape) GetVertexCount() int32 {
	return this.VertexCount
}

// Get a vertex by index.
func (this *PolygonShape) GetVertex(index int32) Vec2 {
	return this.Vertices[index]
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
	Count int32

	PrevVertex, NextVertex       Vec2
	HasPrevVertex, HasNextVertex bool
}

func NewChainShape() *ChainShape {
	this := new(ChainShape)
	this.Type = Shape_e_chain
	this.Radius = PolygonRadius
	return this
}

func (this *ChainShape) Clone() IShape {
	clone := NewChainShape()
	clone.CreateChain(this.Vertices[:this.Count])
	clone.PrevVertex = this.PrevVertex
	clone.NextVertex = this.NextVertex
	clone.HasPrevVertex = this.HasPrevVertex
	clone.HasNextVertex = this.HasNextVertex
	return clone
}

// Create a loop. This automatically adjusts connectivity.
// @param vertices an array of vertices, these are copied
// @param count the vertex count
func (this *ChainShape) CreateLoop(vertices []Vec2) {
	count := int32(len(vertices))
	this.Count = count + 1
	this.Vertices = make([]Vec2, this.Count, this.Count)
	copy(this.Vertices, vertices)
	this.Vertices[count] = this.Vertices[0]
	this.PrevVertex = this.Vertices[this.Count-2]
	this.NextVertex = this.Vertices[1]
	this.HasPrevVertex = true
	this.HasNextVertex = true
}

// Create a chain with isolated end vertices.
// @param vertices an array of vertices, these are copied
// @param count the vertex count
func (this *ChainShape) CreateChain(vertices []Vec2) {
	this.Count = int32(len(vertices))
	this.Vertices = make([]Vec2, this.Count, this.Count)
	copy(this.Vertices, vertices)
	this.HasPrevVertex = false
	this.HasNextVertex = false
}

// Establish connectivity to a vertex that precedes the first vertex.
// Don't call this for loops.
func (this *ChainShape) SetPrevVertex(prevVertex Vec2) {
	this.PrevVertex = prevVertex
	this.HasPrevVertex = true
}

// Establish connectivity to a vertex that follows the last vertex.
// Don't call this for loops.
func (this *ChainShape) SetNextVertex(nextVertex Vec2) {
	this.NextVertex = nextVertex
	this.HasNextVertex = true
}

// @see b2Shape::GetChildCount
func (this *ChainShape) GetChildCount() int32 {
	return this.Count - 1
}

// Get a child edge.
func (this *ChainShape) GetChildEdge(index int32) *EdgeShape {
	edge := NewEdgeShape()

	edge.Type = Shape_e_edge
	edge.Radius = this.Radius

	edge.Vertex1 = this.Vertices[index+0]
	edge.Vertex2 = this.Vertices[index+1]

	if index > 0 {
		edge.Vertex0 = this.Vertices[index-1]
		edge.HasVertex0 = true
	} else {
		edge.Vertex0 = this.PrevVertex
		edge.HasVertex0 = this.HasPrevVertex
	}

	if index < this.Count-2 {
		edge.Vertex3 = this.Vertices[index+2]
		edge.HasVertex3 = true
	} else {
		edge.Vertex3 = this.NextVertex
		edge.HasVertex3 = this.HasNextVertex
	}

	return edge
}

// This always return false.
// @see b2Shape::TestPoint
func (this *ChainShape) TestPoint(xf Transform, p Vec2) bool {
	return false
}

// Implement b2Shape.
func (this *ChainShape) RayCast(input RayCastInput, xf Transform, childIndex int32) (output RayCastOutput, ret bool) {
	edgeShape := NewEdgeShape()

	i1 := childIndex
	i2 := childIndex + 1
	if i2 == this.Count {
		i2 = 0
	}

	edgeShape.Vertex1 = this.Vertices[i1]
	edgeShape.Vertex2 = this.Vertices[i2]

	return edgeShape.RayCast(input, xf, 0)
}

// @see b2Shape::ComputeAABB
func (this *ChainShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int32) {
	i1 := childIndex
	i2 := childIndex + 1
	if i2 == this.Count {
		i2 = 0
	}

	v1 := MulX(xf, this.Vertices[i1])
	v2 := MulX(xf, this.Vertices[i2])

	aabb.LowerBound = MinV(v1, v2)
	aabb.UpperBound = MaxV(v1, v2)
}

// Chains have zero mass.
// @see b2Shape::ComputeMass
func (this *ChainShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = 0.0
	massData.Center.SetZero()
	massData.I = 0.0
}
