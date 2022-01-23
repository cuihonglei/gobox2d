package box2d

import (
	"math"
)

// Structures and functions used for computing contact points, distance
// queries, and TOI queries.

const NullFeature uint8 = 0xFF

// The features that intersect to form the contact point
// This must be 4 bytes or less.
type ContactFeature struct {
	IndexA uint8 // Feature index on shapeA
	IndexB uint8 // Feature index on shapeB
	TypeA  uint8 // The feature type on shapeA
	TypeB  uint8 // The feature type on shapeB
}

type ContactFeatureType byte

const (
	ContactFeature_e_vertex = 0
	ContactFeature_e_face   = 1
)

// Contact ids to facilitate warm starting.
type ContactID struct {
	Cf ContactFeature
	//Key uint // Used to quickly compare contact ids.
}

// A manifold point is a contact point belonging to a contact
// manifold. It holds details related to the geometry and dynamics
// of the contact points.
// The local point usage depends on the manifold type:
// -e_circles: the local center of circleB
// -e_faceA: the local center of cirlceB or the clip point of polygonB
// -e_faceB: the clip point of polygonA
// This structure is stored across time steps, so we keep it small.
// Note: the impulses are used for internal caching and may not
// provide reliable contact forces, especially for high speed collisions.
type ManifoldPoint struct {
	LocalPoint     Vec2      // usage depends on manifold type
	NormalImpulse  float64   // the non-penetration impulse
	TangentImpulse float64   // the friction impulse
	Id             ContactID // uniquely identifies a contact point between two shapes
}

// A manifold for two touching convex shapes.
// Box2D supports multiple types of contact:
// - clip point versus plane with radius
// - point versus point with radius (circles)
// The local point usage depends on the manifold type:
// -e_circles: the local center of circleA
// -e_faceA: the center of faceA
// -e_faceB: the center of faceB
// Similarly the local normal usage:
// -e_circles: not used
// -e_faceA: the normal on polygonA
// -e_faceB: the normal on polygonB
// We store contacts in this way so that position correction can
// account for movement, which is critical for continuous physics.
// All contact scenarios must be expressed in one of these types.
// This structure is stored across time steps, so we keep it small.
type Manifold struct {
	Points      [MaxManifoldPoints]ManifoldPoint // the points of contact
	LocalNormal Vec2                             // not use for Type::e_points
	LocalPoint  Vec2                             // usage depends on manifold type
	Type        ManifoldType
	PointCount  int // the number of manifold points
}

type ManifoldType byte

const (
	Manifold_e_circles = iota
	Manifold_e_faceA
	Manifold_e_faceB
)

// This is used to compute the current state of a contact manifold.
type WorldManifold struct {
	Normal Vec2                    // world vector pointing from A to B
	Points [MaxManifoldPoints]Vec2 // world contact point (point of intersection)
}

// Evaluate the manifold with supplied transforms. This assumes
// modest motion from the original state. This does not change the
// point count, impulses, etc. The radii must come from the shapes
// that generated the manifold.
func (this *WorldManifold) Initialize(manifold *Manifold,
	xfA Transform, radiusA float64,
	xfB Transform, radiusB float64) {
	if manifold.PointCount == 0 {
		return
	}

	switch manifold.Type {
	case Manifold_e_circles:
		this.Normal.Set(1.0, 0.0)
		pointA := MulX(xfA, manifold.LocalPoint)
		pointB := MulX(xfB, manifold.Points[0].LocalPoint)
		if DistanceSquaredVV(pointA, pointB) > Epsilon*Epsilon {
			this.Normal = SubVV(pointB, pointA)
			this.Normal.Normalize()
		}

		cA := AddVV(pointA, MulFV(radiusA, this.Normal))
		cB := SubVV(pointB, MulFV(radiusB, this.Normal))
		this.Points[0] = MulFV(0.5, AddVV(cA, cB))
	case Manifold_e_faceA:
		this.Normal = MulRV(xfA.Q, manifold.LocalNormal)
		planePoint := MulX(xfA, manifold.LocalPoint)

		for i := 0; i < manifold.PointCount; i++ {
			clipPoint := MulX(xfB, manifold.Points[i].LocalPoint)
			cA := AddVV(clipPoint, MulFV(radiusA-DotVV(SubVV(clipPoint, planePoint), this.Normal), this.Normal))
			cB := SubVV(clipPoint, MulFV(radiusB, this.Normal))
			this.Points[i] = MulFV(0.5, AddVV(cA, cB))
		}
	case Manifold_e_faceB:
		this.Normal = MulRV(xfB.Q, manifold.LocalNormal)
		planePoint := MulX(xfB, manifold.LocalPoint)

		for i := 0; i < manifold.PointCount; i++ {
			clipPoint := MulX(xfA, manifold.Points[i].LocalPoint)
			cB := AddVV(clipPoint, MulFV(radiusB-DotVV(SubVV(clipPoint, planePoint), this.Normal), this.Normal))
			cA := SubVV(clipPoint, MulFV(radiusA, this.Normal))
			this.Points[i] = MulFV(0.5, AddVV(cA, cB))
		}

		// Ensure normal points from A to B.
		this.Normal = this.Normal.Minus()
	}
}

// This is used for determining the state of contact points.
type PointState byte

const (
	NullState    = iota // point does not exist
	AddState            // point was added in the update
	PersistState        // point persisted across the update
	RemoveState         // point was removed in the update
)

// Compute the point states given two manifolds. The states pertain to the transition from manifold1
// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
func GetPointStates(state1 *[MaxManifoldPoints]PointState, state2 *[MaxManifoldPoints]PointState,
	manifold1 *Manifold, manifold2 *Manifold) {
	for i := 0; i < MaxManifoldPoints; i++ {
		state1[i] = NullState
		state2[i] = NullState
	}

	// Detect persists and removes.
	for i := 0; i < manifold1.PointCount; i++ {
		id := manifold1.Points[i].Id

		state1[i] = RemoveState

		for j := 0; j < manifold2.PointCount; j++ {
			if manifold2.Points[j].Id.Cf == id.Cf {
				state1[i] = PersistState
				break
			}
		}
	}

	// Detect persists and adds.
	for i := 0; i < manifold2.PointCount; i++ {
		id := manifold2.Points[i].Id

		state2[i] = AddState

		for j := 0; j < manifold1.PointCount; j++ {
			if manifold1.Points[j].Id.Cf == id.Cf {
				state2[i] = PersistState
				break
			}
		}
	}
}

/// Used for computing contact manifolds.
type ClipVertex struct {
	V  Vec2
	Id ContactID
}

// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
type RayCastInput struct {
	P1, P2      Vec2
	MaxFraction float64
}

// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
// come from b2RayCastInput.
type RayCastOutput struct {
	Normal   Vec2
	Fraction float64
}

// An axis aligned bounding box.
type AABB struct {
	LowerBound Vec2
	UpperBound Vec2
}

func MakeAABB() AABB {
	return AABB{}
}

func NewAABB() *AABB {
	return &AABB{}
}

// Verify that the bounds are sorted.
func (this *AABB) IsValid() bool {
	d := SubVV(this.UpperBound, this.LowerBound)
	valid := d.X >= 0.0 && d.Y >= 0.0
	valid = valid && this.LowerBound.IsValid() && this.UpperBound.IsValid()
	return valid
}

// Get the center of the AABB.
func (this *AABB) GetCenter() Vec2 {
	return MulFV(0.5, AddVV(this.LowerBound, this.UpperBound))
}

// Get the extents of the AABB (half-widths).
func (this *AABB) GetExtents() Vec2 {
	return MulFV(0.5, SubVV(this.UpperBound, this.LowerBound))
}

// Get the perimeter length
func (this *AABB) GetPerimeter() float64 {
	wx := this.UpperBound.X - this.LowerBound.X
	wy := this.UpperBound.Y - this.LowerBound.Y
	return 2.0 * (wx + wy)
}

// Combine an AABB into this one.
func (this *AABB) Combine(aabb AABB) {
	this.LowerBound = MinV(this.LowerBound, aabb.LowerBound)
	this.UpperBound = MaxV(this.UpperBound, aabb.UpperBound)
}

// Combine two AABBs into this one.
func (this *AABB) Combine2(aabb1 AABB, aabb2 AABB) {
	this.LowerBound = MinV(aabb1.LowerBound, aabb2.LowerBound)
	this.UpperBound = MaxV(aabb1.UpperBound, aabb2.UpperBound)
}

// Does this aabb contain the provided AABB.
func (this *AABB) Contains(aabb AABB) bool {
	result := true
	result = result && this.LowerBound.X <= aabb.LowerBound.X
	result = result && this.LowerBound.Y <= aabb.LowerBound.Y
	result = result && aabb.UpperBound.X <= this.UpperBound.X
	result = result && aabb.UpperBound.Y <= this.UpperBound.Y
	return result
}

func (this *AABB) RayCast(input RayCastInput) (output RayCastOutput, ret bool) {
	tmin := -MaxFloat
	tmax := MaxFloat

	p := input.P1
	d := SubVV(input.P2, input.P1)
	absD := AbsV(d)

	var normal Vec2

	for i := 0; i < 2; i++ {
		if absD.GetI(i) < Epsilon {
			// Parallel.
			if p.GetI(i) < this.LowerBound.GetI(i) || this.UpperBound.GetI(i) < p.GetI(i) {
				return
			}
		} else {
			inv_d := 1.0 / d.GetI(i)
			t1 := (this.LowerBound.GetI(i) - p.GetI(i)) * inv_d
			t2 := (this.UpperBound.GetI(i) - p.GetI(i)) * inv_d

			// Sign of the normal vector.
			s := -1.0

			if t1 > t2 {
				t1, t2 = t2, t1
				s = 1.0
			}

			// Push the min up
			if t1 > tmin {
				normal.SetZero()
				*normal.SetI(i) = s
				tmin = t1
			}

			// Pull the max down
			tmax = MinF(tmax, t2)

			if tmin > tmax {
				return
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if tmin < 0.0 || input.MaxFraction < tmin {
		return
	}

	// Intersection.
	output.Fraction = tmin
	output.Normal = normal
	ret = true
	return
}

// Compute the collision manifold between two circles.
func CollideCircles(manifold *Manifold, circleA *CircleShape, xfA Transform,
	circleB *CircleShape, xfB Transform) {
	manifold.PointCount = 0

	pA := MulX(xfA, circleA.P)
	pB := MulX(xfB, circleB.P)

	d := SubVV(pB, pA)
	distSqr := DotVV(d, d)
	rA, rB := circleA.Radius, circleB.Radius
	radius := rA + rB
	if distSqr > radius*radius {
		return
	}

	manifold.Type = Manifold_e_circles
	manifold.LocalPoint = circleA.P
	manifold.LocalNormal.SetZero()
	manifold.PointCount = 1

	manifold.Points[0].LocalPoint = circleB.P
	manifold.Points[0].Id.Cf = ContactFeature{0, 0, 0, 0}
}

// Compute the collision manifold between a polygon and a circle.
func CollidePolygonAndCircle(manifold *Manifold, polygonA *PolygonShape, xfA Transform,
	circleB *CircleShape, xfB Transform) {
	manifold.PointCount = 0

	// Compute circle position in the frame of the polygon.
	c := MulX(xfB, circleB.P)
	cLocal := MulXT(xfA, c)

	// Find the min separating edge.
	normalIndex := 0
	separation := -MaxFloat
	radius := polygonA.Radius + circleB.Radius
	vertexCount := polygonA.VertexCount
	vertices := &polygonA.Vertices
	normals := &polygonA.Normals

	for i := 0; i < vertexCount; i++ {
		s := DotVV(normals[i], SubVV(cLocal, vertices[i]))

		if s > radius {
			// Early out.
			return
		}

		if s > separation {
			separation = s
			normalIndex = i
		}
	}

	// Vertices that subtend the incident face.
	vertIndex1 := normalIndex
	vertIndex2 := 0
	if vertIndex1+1 < vertexCount {
		vertIndex2 = vertIndex1 + 1
	}
	v1 := vertices[vertIndex1]
	v2 := vertices[vertIndex2]

	// If the center is inside the polygon ...
	if separation < Epsilon {
		manifold.PointCount = 1
		manifold.Type = Manifold_e_faceA
		manifold.LocalNormal = normals[normalIndex]
		manifold.LocalPoint = MulFV(0.5, AddVV(v1, v2))
		manifold.Points[0].LocalPoint = circleB.P
		manifold.Points[0].Id.Cf = ContactFeature{0, 0, 0, 0}
		return
	}

	// Compute barycentric coordinates
	u1 := DotVV(SubVV(cLocal, v1), SubVV(v2, v1))
	u2 := DotVV(SubVV(cLocal, v2), SubVV(v1, v2))
	if u1 <= 0.0 {
		if DistanceSquaredVV(cLocal, v1) > radius*radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = Manifold_e_faceA
		manifold.LocalNormal = SubVV(cLocal, v1)
		manifold.LocalNormal.Normalize()
		manifold.LocalPoint = v1
		manifold.Points[0].LocalPoint = circleB.P
		manifold.Points[0].Id.Cf = ContactFeature{0, 0, 0, 0}
	} else if u2 <= 0.0 {
		if DistanceSquaredVV(cLocal, v2) > radius*radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = Manifold_e_faceA
		manifold.LocalNormal = SubVV(cLocal, v2)
		manifold.LocalNormal.Normalize()
		manifold.LocalPoint = v2
		manifold.Points[0].LocalPoint = circleB.P
		manifold.Points[0].Id.Cf = ContactFeature{0, 0, 0, 0}
	} else {
		faceCenter := MulFV(0.5, AddVV(v1, v2))
		separation := DotVV(SubVV(cLocal, faceCenter), normals[vertIndex1])
		if separation > radius {
			return
		}

		manifold.PointCount = 1
		manifold.Type = Manifold_e_faceA
		manifold.LocalNormal = normals[vertIndex1]
		manifold.LocalPoint = faceCenter
		manifold.Points[0].LocalPoint = circleB.P
		manifold.Points[0].Id.Cf = ContactFeature{0, 0, 0, 0}
	}
}

// Find the separation between poly1 and poly2 for a give edge normal on poly1.
func EdgeSeparation(poly1 *PolygonShape, xf1 Transform, edge1 int, poly2 *PolygonShape, xf2 Transform) float64 {
	vertices1 := &poly1.Vertices
	normals1 := &poly1.Normals

	count2 := poly2.VertexCount
	vertices2 := &poly2.Vertices

	// Convert normal from poly1's frame into poly2's frame.
	normal1World := MulRV(xf1.Q, normals1[edge1])
	normal1 := MulTRV(xf2.Q, normal1World)

	// Find support vertex on poly2 for -normal.
	index := 0
	minDot := MaxFloat

	for i := 0; i < count2; i++ {
		dot := DotVV(vertices2[i], normal1)
		if dot < minDot {
			minDot = dot
			index = i
		}
	}

	v1 := MulX(xf1, vertices1[edge1])
	v2 := MulX(xf2, vertices2[index])
	separation := DotVV(SubVV(v2, v1), normal1World)
	return separation
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
func FindMaxSeparation(edgeIndex *int, poly1 *PolygonShape, xf1 Transform, poly2 *PolygonShape, xf2 Transform) float64 {
	count1 := poly1.VertexCount
	normals1 := &poly1.Normals

	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	d := SubVV(MulX(xf2, poly2.Centroid), MulX(xf1, poly1.Centroid))
	dLocal1 := MulTRV(xf1.Q, d)

	// Find edge normal on poly1 that has the largest projection onto d.
	edge := 0
	maxDot := -MaxFloat
	for i := 0; i < count1; i++ {
		dot := DotVV(normals1[i], dLocal1)
		if dot > maxDot {
			maxDot = dot
			edge = i
		}
	}

	// Get the separation for the edge normal.
	s := EdgeSeparation(poly1, xf1, edge, poly2, xf2)

	// Check the separation for the previous edge normal.
	prevEdge := count1 - 1
	if edge-1 >= 0 {
		prevEdge = edge - 1
	}
	sPrev := EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2)

	// Check the separation for the next edge normal.
	nextEdge := 0
	if edge+1 < count1 {
		nextEdge = edge + 1
	}
	sNext := EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2)

	// Find the best edge and the search direction.
	var bestEdge int
	var bestSeparation float64
	var increment int
	if sPrev > s && sPrev > sNext {
		increment = -1
		bestEdge = prevEdge
		bestSeparation = sPrev
	} else if sNext > s {
		increment = 1
		bestEdge = nextEdge
		bestSeparation = sNext
	} else {
		*edgeIndex = edge
		return s
	}

	// Perform a local search for the best edge normal.
	for {
		if increment == -1 {
			edge = count1 - 1
			if bestEdge-1 >= 0 {
				edge = bestEdge - 1
			}
		} else {
			edge = 0
			if bestEdge+1 < count1 {
				edge = bestEdge + 1
			}
		}
		s = EdgeSeparation(poly1, xf1, edge, poly2, xf2)

		if s > bestSeparation {
			bestEdge = edge
			bestSeparation = s
		} else {
			break
		}
	}
	*edgeIndex = bestEdge
	return bestSeparation
}

func FindIncidentEdge(c []ClipVertex, poly1 *PolygonShape, xf1 Transform, edge1 int, poly2 *PolygonShape, xf2 Transform) {
	normals1 := &poly1.Normals

	count2 := poly2.VertexCount
	vertices2 := &poly2.Vertices
	normals2 := &poly2.Normals

	// Get the normal of the reference edge in poly2's frame.
	normal1 := MulTRV(xf2.Q, MulRV(xf1.Q, normals1[edge1]))

	// Find the incident edge on poly2.
	index := 0
	minDot := MaxFloat
	for i := 0; i < count2; i++ {
		dot := DotVV(normal1, normals2[i])
		if dot < minDot {
			minDot = dot
			index = i
		}
	}

	// Build the clip vertices for the incident edge.
	i1 := index
	i2 := 0
	if i1+1 < count2 {
		i2 = i1 + 1
	}

	c[0].V = MulX(xf2, vertices2[i1])
	c[0].Id.Cf.IndexA = uint8(edge1)
	c[0].Id.Cf.IndexB = uint8(i1)
	c[0].Id.Cf.TypeA = ContactFeature_e_face
	c[0].Id.Cf.TypeB = ContactFeature_e_vertex

	c[1].V = MulX(xf2, vertices2[i2])
	c[1].Id.Cf.IndexA = uint8(edge1)
	c[1].Id.Cf.IndexB = uint8(i2)
	c[1].Id.Cf.TypeA = ContactFeature_e_face
	c[1].Id.Cf.TypeB = ContactFeature_e_vertex
}

// Compute the collision manifold between two polygons.
func CollidePolygons(manifold *Manifold, polyA *PolygonShape, xfA Transform, polyB *PolygonShape, xfB Transform) {
	manifold.PointCount = 0
	totalRadius := polyA.Radius + polyB.Radius

	edgeA := 0
	separationA := FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB)
	if separationA > totalRadius {
		return
	}

	edgeB := 0
	separationB := FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA)
	if separationB > totalRadius {
		return
	}

	var poly1 *PolygonShape // reference polygon
	var poly2 *PolygonShape // incident polygon
	var xf1, xf2 Transform
	var edge1 int // reference edge
	var flip uint8
	const k_relativeTol float64 = 0.98
	const k_absoluteTol float64 = 0.001

	if separationB > k_relativeTol*separationA+k_absoluteTol {
		poly1 = polyB
		poly2 = polyA
		xf1 = xfB
		xf2 = xfA
		edge1 = edgeB
		manifold.Type = Manifold_e_faceB
		flip = 1
	} else {
		poly1 = polyA
		poly2 = polyB
		xf1 = xfA
		xf2 = xfB
		edge1 = edgeA
		manifold.Type = Manifold_e_faceA
		flip = 0
	}

	incidentEdge := make([]ClipVertex, 2, 2)
	FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2)

	count1 := poly1.VertexCount
	vertices1 := &poly1.Vertices

	iv1 := edge1
	iv2 := 0
	if edge1+1 < count1 {
		iv2 = edge1 + 1
	}

	v11 := vertices1[iv1]
	v12 := vertices1[iv2]

	localTangent := SubVV(v12, v11)
	localTangent.Normalize()

	localNormal := CrossVF(localTangent, 1.0)
	planePoint := MulFV(0.5, AddVV(v11, v12))

	tangent := MulRV(xf1.Q, localTangent)
	normal := CrossVF(tangent, 1.0)

	v11 = MulX(xf1, v11)
	v12 = MulX(xf1, v12)

	// Face offset.
	frontOffset := DotVV(normal, v11)

	// Side offsets, extended by polytope skin thickness.
	sideOffset1 := -DotVV(tangent, v11) + totalRadius
	sideOffset2 := DotVV(tangent, v12) + totalRadius

	// Clip incident edge against extruded edge1 side edges.
	clipPoints1 := make([]ClipVertex, 2, 2)
	clipPoints2 := make([]ClipVertex, 2, 2)
	var np int

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, tangent.Minus(), sideOffset1, iv1)

	if np < 2 {
		return
	}

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2)

	if np < 2 {
		return
	}

	// Now clipPoints2 contains the clipped points.
	manifold.LocalNormal = localNormal
	manifold.LocalPoint = planePoint

	pointCount := 0
	for i := 0; i < MaxManifoldPoints; i++ {
		separation := DotVV(normal, clipPoints2[i].V) - frontOffset

		if separation <= totalRadius {
			cp := &manifold.Points[pointCount]
			cp.LocalPoint = MulXT(xf2, clipPoints2[i].V)
			cp.Id = clipPoints2[i].Id
			if flip != 0 {
				// Swap features
				cf := cp.Id.Cf
				cp.Id.Cf.IndexA = cf.IndexB
				cp.Id.Cf.IndexB = cf.IndexA
				cp.Id.Cf.TypeA = cf.TypeB
				cp.Id.Cf.TypeB = cf.TypeA
			}
			pointCount++
		}
	}

	manifold.PointCount = pointCount
}

// Compute the collision manifold between an edge and a circle.
func CollideEdgeAndCircle(manifold *Manifold, edgeA *EdgeShape, xfA Transform,
	circleB *CircleShape, xfB Transform) {
	manifold.PointCount = 0

	// Compute circle in frame of edge
	Q := MulXT(xfA, MulX(xfB, circleB.P))

	A, B := edgeA.Vertex1, edgeA.Vertex2
	e := SubVV(B, A)

	// Barycentric coordinates
	u := DotVV(e, SubVV(B, Q))
	v := DotVV(e, SubVV(Q, A))

	radius := edgeA.Radius + circleB.Radius

	var cf ContactFeature
	cf.IndexB = 0
	cf.TypeB = ContactFeature_e_vertex

	// Region A
	if v <= 0.0 {
		P := A
		d := SubVV(Q, P)
		dd := DotVV(d, d)
		if dd > radius*radius {
			return
		}

		// Is there an edge connected to A?
		if edgeA.HasVertex0 {
			A1 := edgeA.Vertex0
			B1 := A
			e1 := SubVV(B1, A1)
			u1 := DotVV(e1, SubVV(B1, Q))

			// Is the circle in Region AB of the previous edge?
			if u1 > 0.0 {
				return
			}
		}

		cf.IndexA = 0
		cf.TypeA = ContactFeature_e_vertex
		manifold.PointCount = 1
		manifold.Type = Manifold_e_circles
		manifold.LocalNormal.SetZero()
		manifold.LocalPoint = P
		manifold.Points[0].Id.Cf = ContactFeature{0, 0, 0, 0}
		manifold.Points[0].Id.Cf = cf
		manifold.Points[0].LocalPoint = circleB.P
		return
	}

	// Region B
	if u <= 0.0 {
		P := B
		d := SubVV(Q, P)
		dd := DotVV(d, d)
		if dd > radius*radius {
			return
		}

		// Is there an edge connected to B?
		if edgeA.HasVertex3 {
			B2 := edgeA.Vertex3
			A2 := B
			e2 := SubVV(B2, A2)
			v2 := DotVV(e2, SubVV(Q, A2))

			// Is the circle in Region AB of the next edge?
			if v2 > 0.0 {
				return
			}
		}

		cf.IndexA = 1
		cf.TypeA = ContactFeature_e_vertex
		manifold.PointCount = 1
		manifold.Type = Manifold_e_circles
		manifold.LocalNormal.SetZero()
		manifold.LocalPoint = P
		manifold.Points[0].Id.Cf = ContactFeature{0, 0, 0, 0}
		manifold.Points[0].Id.Cf = cf
		manifold.Points[0].LocalPoint = circleB.P
		return
	}

	// Region AB
	den := DotVV(e, e)
	P := MulFV(1.0/den, AddVV(MulFV(u, A), MulFV(v, B)))
	d := SubVV(Q, P)
	dd := DotVV(d, d)
	if dd > radius*radius {
		return
	}

	n := Vec2{-e.Y, e.X}
	if DotVV(n, SubVV(Q, A)) < 0.0 {
		n.Set(-n.X, -n.Y)
	}
	n.Normalize()

	cf.IndexA = 0
	cf.TypeA = ContactFeature_e_face
	manifold.PointCount = 1
	manifold.Type = Manifold_e_faceA
	manifold.LocalNormal = n
	manifold.LocalPoint = A
	manifold.Points[0].Id.Cf = ContactFeature{0, 0, 0, 0}
	manifold.Points[0].Id.Cf = cf
	manifold.Points[0].LocalPoint = circleB.P
}

// This structure is used to keep track of the best separating axis.
type EPAxisType byte

const (
	EPAxis_e_unknown = iota
	EPAxis_e_edgeA
	EPAxis_e_edgeB
)

type EPAxis struct {
	Type       EPAxisType
	Index      int
	Separation float64
}

// This holds polygon B expressed in frame A.
type TempPolygon struct {
	Vertices [MaxPolygonVertices]Vec2
	Normals  [MaxPolygonVertices]Vec2
	Count    int
}

// Reference face used for clipping
type ReferenceFace struct {
	I1, I2 int

	V1, V2 Vec2

	Normal Vec2

	SideNormal1 Vec2
	SideOffset1 float64

	SideNormal2 Vec2
	SideOffset2 float64
}

// This class collides and edge and a polygon, taking into account edge adjacency.
type EPColliderType byte

const (
	EPCollider_e_isolated = iota
	EPCollider_e_concave
	EPCollider_e_convex
)

type EPCollider struct {
	PolygonB TempPolygon

	Xf                        Transform
	CentroidB                 Vec2
	V0, V1, V2, V3            Vec2
	Normal0, Normal1, Normal2 Vec2
	Normal                    Vec2
	Type1, Type2              EPColliderType
	LowerLimit, UpperLimit    Vec2
	Radius                    float64
	Front                     bool
}

// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
func (this *EPCollider) Collide(manifold *Manifold, edgeA *EdgeShape, xfA Transform, polygonB *PolygonShape, xfB Transform) {
	this.Xf = MulTTT(xfA, xfB)

	this.CentroidB = MulX(this.Xf, polygonB.Centroid)

	this.V0 = edgeA.Vertex0
	this.V1 = edgeA.Vertex1
	this.V2 = edgeA.Vertex2
	this.V3 = edgeA.Vertex3

	hasVertex0 := edgeA.HasVertex0
	hasVertex3 := edgeA.HasVertex3

	edge1 := SubVV(this.V2, this.V1)
	edge1.Normalize()
	this.Normal1.Set(edge1.Y, -edge1.X)
	offset1 := DotVV(this.Normal1, SubVV(this.CentroidB, this.V1))
	offset0, offset2 := 0.0, 0.0
	convex1, convex2 := false, false

	// Is there a preceding edge?
	if hasVertex0 {
		edge0 := SubVV(this.V1, this.V0)
		edge0.Normalize()
		this.Normal0.Set(edge0.Y, -edge0.X)
		convex1 = CrossVV(edge0, edge1) >= 0.0
		offset0 = DotVV(this.Normal0, SubVV(this.CentroidB, this.V0))
	}

	// Is there a following edge?
	if hasVertex3 {
		edge2 := SubVV(this.V3, this.V2)
		edge2.Normalize()
		this.Normal2.Set(edge2.Y, -edge2.X)
		convex2 = CrossVV(edge1, edge2) > 0.0
		offset2 = DotVV(this.Normal2, SubVV(this.CentroidB, this.V2))
	}

	// Determine front or back collision. Determine collision normal limits.
	if hasVertex0 && hasVertex3 {
		if convex1 && convex2 {
			this.Front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0
			if this.Front {
				this.Normal = this.Normal1
				this.LowerLimit = this.Normal0
				this.UpperLimit = this.Normal2
			} else {
				this.Normal = this.Normal1.Minus()
				this.LowerLimit = this.Normal1.Minus()
				this.UpperLimit = this.Normal1.Minus()
			}
		} else if convex1 {
			this.Front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0)
			if this.Front {
				this.Normal = this.Normal1
				this.LowerLimit = this.Normal0
				this.UpperLimit = this.Normal1
			} else {
				this.Normal = this.Normal1.Minus()
				this.LowerLimit = this.Normal2.Minus()
				this.UpperLimit = this.Normal1.Minus()
			}
		} else if convex2 {
			this.Front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0)
			if this.Front {
				this.Normal = this.Normal1
				this.LowerLimit = this.Normal1
				this.UpperLimit = this.Normal2
			} else {
				this.Normal = this.Normal1.Minus()
				this.LowerLimit = this.Normal1.Minus()
				this.UpperLimit = this.Normal0.Minus()
			}
		} else {
			this.Front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0
			if this.Front {
				this.Normal = this.Normal1
				this.LowerLimit = this.Normal1
				this.UpperLimit = this.Normal1
			} else {
				this.Normal = this.Normal1.Minus()
				this.LowerLimit = this.Normal2.Minus()
				this.UpperLimit = this.Normal0.Minus()
			}
		}
	} else if hasVertex0 {
		if convex1 {
			this.Front = offset0 >= 0.0 || offset1 >= 0.0
			if this.Front {
				this.Normal = this.Normal1
				this.LowerLimit = this.Normal0
				this.UpperLimit = this.Normal1.Minus()
			} else {
				this.Normal = this.Normal1.Minus()
				this.LowerLimit = this.Normal1
				this.UpperLimit = this.Normal1.Minus()
			}
		} else {
			this.Front = offset0 >= 0.0 && offset1 >= 0.0
			if this.Front {
				this.Normal = this.Normal1
				this.LowerLimit = this.Normal1
				this.UpperLimit = this.Normal1.Minus()
			} else {
				this.Normal = this.Normal1.Minus()
				this.LowerLimit = this.Normal1
				this.UpperLimit = this.Normal0.Minus()
			}
		}
	} else if hasVertex3 {
		if convex2 {
			this.Front = offset1 >= 0.0 || offset2 >= 0.0
			if this.Front {
				this.Normal = this.Normal1
				this.LowerLimit = this.Normal1.Minus()
				this.UpperLimit = this.Normal2
			} else {
				this.Normal = this.Normal1.Minus()
				this.LowerLimit = this.Normal1.Minus()
				this.UpperLimit = this.Normal1
			}
		} else {
			this.Front = offset1 >= 0.0 && offset2 >= 0.0
			if this.Front {
				this.Normal = this.Normal1
				this.LowerLimit = this.Normal1.Minus()
				this.UpperLimit = this.Normal1
			} else {
				this.Normal = this.Normal1.Minus()
				this.LowerLimit = this.Normal2.Minus()
				this.UpperLimit = this.Normal1
			}
		}
	} else {
		this.Front = offset1 >= 0.0
		if this.Front {
			this.Normal = this.Normal1
			this.LowerLimit = this.Normal1.Minus()
			this.UpperLimit = this.Normal1.Minus()
		} else {
			this.Normal = this.Normal1.Minus()
			this.LowerLimit = this.Normal1
			this.UpperLimit = this.Normal1
		}
	}

	// Get polygonB in frameA
	this.PolygonB.Count = polygonB.VertexCount
	for i := 0; i < polygonB.VertexCount; i++ {
		this.PolygonB.Vertices[i] = MulX(this.Xf, polygonB.Vertices[i])
		this.PolygonB.Normals[i] = MulRV(this.Xf.Q, polygonB.Normals[i])
	}

	this.Radius = 2.0 * PolygonRadius

	manifold.PointCount = 0

	edgeAxis := this.ComputeEdgeSeparation()

	// If no valid normal can be found than this edge should not collide.
	if edgeAxis.Type == EPAxis_e_unknown {
		return
	}

	if edgeAxis.Separation > this.Radius {
		return
	}

	polygonAxis := this.ComputePolygonSeparation()
	if polygonAxis.Type != EPAxis_e_unknown && polygonAxis.Separation > this.Radius {
		return
	}

	// Use hysteresis for jitter reduction.
	const k_relativeTol float64 = 0.98
	const k_absoluteTol float64 = 0.001

	var primaryAxis EPAxis
	if polygonAxis.Type == EPAxis_e_unknown {
		primaryAxis = edgeAxis
	} else if polygonAxis.Separation > k_relativeTol*edgeAxis.Separation+k_absoluteTol {
		primaryAxis = polygonAxis
	} else {
		primaryAxis = edgeAxis
	}

	ie := make([]ClipVertex, 2, 2)
	var rf ReferenceFace
	if primaryAxis.Type == EPAxis_e_edgeA {
		manifold.Type = Manifold_e_faceA

		// Search for the polygon normal that is most anti-parallel to the edge normal.
		bestIndex := 0
		bestValue := DotVV(this.Normal, this.PolygonB.Normals[0])
		for i := 1; i < this.PolygonB.Count; i++ {
			value := DotVV(this.Normal, this.PolygonB.Normals[i])
			if value < bestValue {
				bestValue = value
				bestIndex = i
			}
		}

		i1 := bestIndex
		i2 := 0
		if i1+1 < this.PolygonB.Count {
			i2 = i1 + 1
		}

		ie[0].V = this.PolygonB.Vertices[i1]
		ie[0].Id.Cf.IndexA = uint8(0)
		ie[0].Id.Cf.IndexB = uint8(i1)
		ie[0].Id.Cf.TypeA = ContactFeature_e_face
		ie[0].Id.Cf.TypeB = ContactFeature_e_vertex

		ie[1].V = this.PolygonB.Vertices[i2]
		ie[1].Id.Cf.IndexA = uint8(0)
		ie[1].Id.Cf.IndexB = uint8(i2)
		ie[1].Id.Cf.TypeA = ContactFeature_e_face
		ie[1].Id.Cf.TypeB = ContactFeature_e_vertex

		if this.Front {
			rf.I1 = 0
			rf.I2 = 1
			rf.V1 = this.V1
			rf.V2 = this.V2
			rf.Normal = this.Normal1
		} else {
			rf.I1 = 1
			rf.I2 = 0
			rf.V1 = this.V2
			rf.V2 = this.V1
			rf.Normal = this.Normal1.Minus()
		}
	} else {
		manifold.Type = Manifold_e_faceB

		ie[0].V = this.V1
		ie[0].Id.Cf.IndexA = 0
		ie[0].Id.Cf.IndexB = uint8(primaryAxis.Index)
		ie[0].Id.Cf.TypeA = ContactFeature_e_vertex
		ie[0].Id.Cf.TypeB = ContactFeature_e_face

		ie[1].V = this.V2
		ie[1].Id.Cf.IndexA = 0
		ie[1].Id.Cf.IndexB = uint8(primaryAxis.Index)
		ie[1].Id.Cf.TypeA = ContactFeature_e_vertex
		ie[1].Id.Cf.TypeB = ContactFeature_e_face

		rf.I1 = primaryAxis.Index
		rf.I2 = 0
		if rf.I1+1 < this.PolygonB.Count {
			rf.I2 = rf.I1 + 1
		}
		rf.V1 = this.PolygonB.Vertices[rf.I1]
		rf.V2 = this.PolygonB.Vertices[rf.I2]
		rf.Normal = this.PolygonB.Normals[rf.I1]
	}

	rf.SideNormal1.Set(rf.Normal.Y, -rf.Normal.X)
	rf.SideNormal2 = rf.SideNormal1.Minus()
	rf.SideOffset1 = DotVV(rf.SideNormal1, rf.V1)
	rf.SideOffset2 = DotVV(rf.SideNormal2, rf.V2)

	// Clip incident edge against extruded edge1 side edges.
	clipPoints1 := make([]ClipVertex, 2, 2)
	clipPoints2 := make([]ClipVertex, 2, 2)
	var np int

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, ie, rf.SideNormal1, rf.SideOffset1, rf.I1)

	if np < MaxManifoldPoints {
		return
	}

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, rf.SideNormal2, rf.SideOffset2, rf.I2)

	if np < MaxManifoldPoints {
		return
	}

	// Now clipPoints2 contains the clipped points.
	if primaryAxis.Type == EPAxis_e_edgeA {
		manifold.LocalNormal = rf.Normal
		manifold.LocalPoint = rf.V1
	} else {
		manifold.LocalNormal = polygonB.Normals[rf.I1]
		manifold.LocalPoint = polygonB.Vertices[rf.I1]
	}

	pointCount := 0
	for i := 0; i < MaxManifoldPoints; i++ {
		separation := DotVV(rf.Normal, SubVV(clipPoints2[i].V, rf.V1))

		if separation <= this.Radius {
			cp := &manifold.Points[pointCount]

			if primaryAxis.Type == EPAxis_e_edgeA {
				cp.LocalPoint = MulXT(this.Xf, clipPoints2[i].V)
				cp.Id = clipPoints2[i].Id
			} else {
				cp.LocalPoint = clipPoints2[i].V
				cp.Id.Cf.TypeA = clipPoints2[i].Id.Cf.TypeB
				cp.Id.Cf.TypeB = clipPoints2[i].Id.Cf.TypeA
				cp.Id.Cf.IndexA = clipPoints2[i].Id.Cf.IndexB
				cp.Id.Cf.IndexB = clipPoints2[i].Id.Cf.IndexA
			}

			pointCount++
		}
	}

	manifold.PointCount = pointCount
}

func (this *EPCollider) ComputeEdgeSeparation() EPAxis {
	var axis EPAxis
	axis.Type = EPAxis_e_edgeA
	axis.Index = 1
	if this.Front {
		axis.Index = 0
	}
	axis.Separation = math.MaxFloat64

	for i := 0; i < this.PolygonB.Count; i++ {
		s := DotVV(this.Normal, SubVV(this.PolygonB.Vertices[i], this.V1))
		if s < axis.Separation {
			axis.Separation = s
		}
	}

	return axis
}

func (this *EPCollider) ComputePolygonSeparation() EPAxis {
	var axis EPAxis
	axis.Type = EPAxis_e_unknown
	axis.Index = -1
	axis.Separation = -math.MaxFloat64

	perp := Vec2{-this.Normal.Y, this.Normal.X}

	for i := 0; i < this.PolygonB.Count; i++ {
		n := this.PolygonB.Normals[i].Minus()

		s1 := DotVV(n, SubVV(this.PolygonB.Vertices[i], this.V1))
		s2 := DotVV(n, SubVV(this.PolygonB.Vertices[i], this.V2))
		s := MinF(s1, s2)

		if s > this.Radius {
			// No collision
			axis.Type = EPAxis_e_edgeB
			axis.Index = i
			axis.Separation = s
			return axis
		}

		// Adjacency
		if DotVV(n, perp) >= 0.0 {
			if DotVV(SubVV(n, this.UpperLimit), this.Normal) < -AngularSlop {
				continue
			}
		} else {
			if DotVV(SubVV(n, this.LowerLimit), this.Normal) < -AngularSlop {
				continue
			}
		}

		if s > axis.Separation {
			axis.Type = EPAxis_e_edgeB
			axis.Index = i
			axis.Separation = s
		}
	}

	return axis
}

// Compute the collision manifold between an edge and a circle.
func CollideEdgeAndPolygon(manifold *Manifold, edgeA *EdgeShape, xfA Transform,
	polygonB *PolygonShape, xfB Transform) {
	var collider EPCollider
	collider.Collide(manifold, edgeA, xfA, polygonB, xfB)
}

// Clipping for contact manifolds.
func ClipSegmentToLine(vOut []ClipVertex, vIn []ClipVertex,
	normal Vec2, offset float64, vertexIndexA int) int {

	// Start with no output points
	numOut := 0

	// Calculate the distance of end points to the line
	distance0 := DotVV(normal, vIn[0].V) - offset
	distance1 := DotVV(normal, vIn[1].V) - offset

	// If the points are behind the plane
	if distance0 <= 0.0 {
		vOut[numOut] = vIn[0]
		numOut++
	}
	if distance1 <= 0.0 {
		vOut[numOut] = vIn[1]
		numOut++
	}

	// If the points are on different sides of the plane
	if distance0*distance1 < 0.0 {
		// Find intersection point of edge and plane
		interp := distance0 / (distance0 - distance1)
		vOut[numOut].V = AddVV(vIn[0].V, MulFV(interp, SubVV(vIn[1].V, vIn[0].V)))

		// VertexA is hitting edgeB.
		vOut[numOut].Id.Cf.IndexA = uint8(vertexIndexA)
		vOut[numOut].Id.Cf.IndexB = vIn[0].Id.Cf.IndexB
		vOut[numOut].Id.Cf.TypeA = ContactFeature_e_vertex
		vOut[numOut].Id.Cf.TypeB = ContactFeature_e_face
		numOut++
	}

	return numOut
}

// Determine if two generic shapes overlap.
func TestOverlap(shapeA IShape, indexA int, shapeB IShape, indexB int,
	xfA Transform, xfB Transform) bool {

	var input DistanceInput
	input.ProxyA.Set(shapeA, indexA)
	input.ProxyB.Set(shapeB, indexB)
	input.TransformA = xfA
	input.TransformB = xfB
	input.UseRadii = true

	var cache SimplexCache
	cache.Count = 0

	output := Distance(&cache, &input)

	return output.Distance < 10.0*Epsilon
}

func TestOverlapAABB(a AABB, b AABB) bool {
	var d1, d2 Vec2
	d1 = SubVV(b.LowerBound, a.UpperBound)
	d2 = SubVV(a.LowerBound, b.UpperBound)

	if d1.X > 0.0 || d1.Y > 0.0 {
		return false
	}

	if d2.X > 0.0 || d2.Y > 0.0 {
		return false
	}

	return true
}
