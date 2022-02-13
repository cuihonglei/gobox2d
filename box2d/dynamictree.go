package box2d

const NullNode = -1

// A node in the dynamic tree. The client does not interact with this directly.
type TreeNode struct {
	// Enlarged AABB
	AABB AABB

	UserData interface{}

	Parent int

	Child1 int
	Child2 int

	// leaf = 0, free node = -1
	Height int
}

func (tn *TreeNode) IsLeaf() bool {
	return tn.Child1 == NullNode
}

// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
// A dynamic tree arranges data in a binary tree to accelerate
// queries such as volume queries and ray casts. Leafs are proxies
// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
// so that the proxy AABB is bigger than the client object. This allows the client
// object to move by small amounts without triggering a tree update.
//
// Nodes are pooled and relocatable, so we use node indices rather than pointers.
type DynamicTree struct {
	root int

	nodes        []TreeNode
	nodeCount    int
	nodeCapacity int

	freeList int

	// This is used to incrementally traverse the tree for re-balancing.
	path uint

	insertionCount int
}

func NewDynamicTree() *DynamicTree {
	dt := new(DynamicTree)

	dt.root = NullNode

	dt.nodeCapacity = 16
	dt.nodes = make([]TreeNode, dt.nodeCapacity)

	// Build a linked list for the free list.
	for i := 0; i < dt.nodeCapacity-1; i++ {
		dt.nodes[i].Parent = i + 1
		dt.nodes[i].Height = -1
	}
	dt.nodes[dt.nodeCapacity-1].Parent = NullNode
	dt.nodes[dt.nodeCapacity-1].Height = -1

	return dt
}

// Create a proxy. Provide a tight fitting AABB and a userData pointer.
func (dt *DynamicTree) CreateProxy(aabb AABB, userData interface{}) int {
	proxyId := dt.allocateNode()

	// Fatten the aabb.
	r := Vec2{AABBExtension, AABBExtension}
	dt.nodes[proxyId].AABB.LowerBound = SubVV(aabb.LowerBound, r)
	dt.nodes[proxyId].AABB.UpperBound = AddVV(aabb.UpperBound, r)
	dt.nodes[proxyId].UserData = userData
	dt.nodes[proxyId].Height = 0

	dt.insertLeaf(proxyId)

	return proxyId
}

// Destroy a proxy. This asserts if the id is invalid.
func (dt *DynamicTree) DestroyProxy(proxyId int) {
	dt.removeLeaf(proxyId)
	dt.freeNode(proxyId)
}

// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
// then the proxy is removed from the tree and re-inserted. Otherwise
// the function returns immediately.
// @return true if the proxy was re-inserted.
func (dt *DynamicTree) MoveProxy(proxyId int, aabb AABB, displacement Vec2) bool {

	if dt.nodes[proxyId].AABB.Contains(aabb) {
		return false
	}

	dt.removeLeaf(proxyId)

	// Extend AABB.
	b := aabb
	r := Vec2{AABBExtension, AABBExtension}
	b.LowerBound = SubVV(b.LowerBound, r)
	b.UpperBound = AddVV(b.UpperBound, r)

	// Predict AABB displacement.
	d := MulFV(AABBMultiplier, displacement)

	if d.X < 0.0 {
		b.LowerBound.X += d.X
	} else {
		b.UpperBound.X += d.X
	}

	if d.Y < 0.0 {
		b.LowerBound.Y += d.Y
	} else {
		b.UpperBound.Y += d.Y
	}

	dt.nodes[proxyId].AABB = b

	dt.insertLeaf(proxyId)
	return true
}

// Get proxy user data.
// @return the proxy user data or 0 if the id is invalid.
func (dt *DynamicTree) GetUserData(proxyId int) interface{} {
	return dt.nodes[proxyId].UserData
}

// Get the fat AABB for a proxy.
func (dt *DynamicTree) GetFatAABB(proxyId int) AABB {
	return dt.nodes[proxyId].AABB
}

// Query an AABB for overlapping proxies. The callback class
// is called for each proxy that overlaps the supplied AABB.
func (dt *DynamicTree) Query(callback func(int) bool, aabb AABB) {
	stack := NewGrowableStack(256)
	stack.Push(dt.root)

	for stack.GetCount() > 0 {
		nodeId, _ := stack.Pop().(int)
		if nodeId == NullNode {
			continue
		}

		node := &dt.nodes[nodeId]

		if TestOverlapAABB(node.AABB, aabb) {
			if node.IsLeaf() {
				proceed := callback(nodeId)
				if !proceed {
					return
				}
			} else {
				stack.Push(node.Child1)
				stack.Push(node.Child2)
			}
		}
	}
}

// Ray-cast against the proxies in the tree. This relies on the callback
// to perform a exact ray-cast in the case were the proxy contains a shape.
// The callback also performs the any collision filtering. This has performance
// roughly equal to k * log(n), where k is the number of collisions and n is the
// number of proxies in the tree.
// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
// @param callback a callback class that is called for each proxy that is hit by the ray.
func (dt *DynamicTree) RayCast(callback func(RayCastInput, int) float64, input RayCastInput) {
	p1 := input.P1
	p2 := input.P2
	r := SubVV(p2, p1)
	r.Normalize()

	// v is perpendicular to the segment.
	v := CrossFV(1.0, r)
	abs_v := AbsV(v)

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	maxFraction := input.MaxFraction

	// Build a bounding box for the segment.
	var segmentAABB AABB
	{
		t := AddVV(p1, MulFV(maxFraction, SubVV(p2, p1)))
		segmentAABB.LowerBound = MinV(p1, t)
		segmentAABB.UpperBound = MaxV(p1, t)
	}

	stack := NewGrowableStack(256)
	stack.Push(dt.root)

	for stack.GetCount() > 0 {
		nodeId := stack.Pop().(int)
		if nodeId == NullNode {
			continue
		}

		node := &dt.nodes[nodeId]

		if !TestOverlapAABB(node.AABB, segmentAABB) {
			continue
		}

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		c := node.AABB.GetCenter()
		h := node.AABB.GetExtents()
		separation := AbsF(DotVV(v, SubVV(p1, c))) - DotVV(abs_v, h)
		if separation > 0.0 {
			continue
		}

		if node.IsLeaf() {
			var subInput RayCastInput
			subInput.P1 = input.P1
			subInput.P2 = input.P2
			subInput.MaxFraction = maxFraction

			value := callback(subInput, nodeId)

			if value == 0.0 {
				// The client has terminated the ray cast.
				return
			}

			if value > 0.0 {
				// Update segment bounding box.
				maxFraction = value
				t := AddVV(p1, MulFV(maxFraction, SubVV(p2, p1)))
				segmentAABB.LowerBound = MinV(p1, t)
				segmentAABB.UpperBound = MaxV(p1, t)
			}
		} else {
			stack.Push(node.Child1)
			stack.Push(node.Child2)
		}
	}
}

// Compute the height of the binary tree in O(N) time. Should not be
// called often.
func (dt *DynamicTree) GetHeight() int {
	if dt.root == NullNode {
		return 0
	}
	return dt.nodes[dt.root].Height
}

// Get the maximum balance of an node in the tree. The balance is the difference
// in height of the two children of a node.
func (dt *DynamicTree) GetMaxBalance() int {
	maxBalance := 0
	for i := 0; i < dt.nodeCapacity; i++ {
		node := &dt.nodes[i]
		if node.Height <= 1 {
			continue
		}

		child1 := node.Child1
		child2 := node.Child2
		balance := AbsI(dt.nodes[child2].Height - dt.nodes[child1].Height)
		maxBalance = MaxI(maxBalance, balance)
	}

	return maxBalance
}

// Get the ratio of the sum of the node areas to the root area.
func (dt *DynamicTree) GetAreaRatio() float64 {
	if dt.root == NullNode {
		return 0.0
	}

	root := &dt.nodes[dt.root]
	rootArea := root.AABB.GetPerimeter()

	totalArea := 0.0
	for i := 0; i < dt.nodeCapacity; i++ {
		node := &dt.nodes[i]
		if node.Height < 0 {
			// Free node in pool
			continue
		}

		totalArea += node.AABB.GetPerimeter()
	}

	return totalArea / rootArea
}

// Allocate a node from the pool. Grow the pool if necessary.
func (dt *DynamicTree) allocateNode() int {
	// Expand the node pool as needed.
	if dt.freeList == NullNode {

		// The free list is empty. Rebuild a bigger pool.
		oldNodes := dt.nodes
		dt.nodeCapacity *= 2
		dt.nodes = make([]TreeNode, dt.nodeCapacity)
		copy(dt.nodes, oldNodes)

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for i := dt.nodeCount; i < dt.nodeCapacity-1; i++ {
			dt.nodes[i].Parent = i + 1
			dt.nodes[i].Height = -1
		}
		dt.nodes[dt.nodeCapacity-1].Parent = NullNode
		dt.nodes[dt.nodeCapacity-1].Height = -1
		dt.freeList = dt.nodeCount
	}

	// Peel a node off the free list.
	nodeId := dt.freeList
	dt.freeList = dt.nodes[nodeId].Parent
	dt.nodes[nodeId].Parent = NullNode
	dt.nodes[nodeId].Child1 = NullNode
	dt.nodes[nodeId].Child2 = NullNode
	dt.nodes[nodeId].Height = 0
	dt.nodes[nodeId].UserData = nil
	dt.nodeCount++
	return nodeId
}

// Return a node to the pool.
func (dt *DynamicTree) freeNode(nodeId int) {
	dt.nodes[nodeId].Parent = dt.freeList
	dt.nodes[nodeId].Height = -1
	dt.freeList = nodeId
	dt.nodeCount--
}

func (dt *DynamicTree) insertLeaf(leaf int) {
	dt.insertionCount++

	if dt.root == NullNode {
		dt.root = leaf
		dt.nodes[dt.root].Parent = NullNode
		return
	}

	// Find the best sibling for this node
	leafAABB := dt.nodes[leaf].AABB
	index := dt.root
	for !dt.nodes[index].IsLeaf() {
		child1 := dt.nodes[index].Child1
		child2 := dt.nodes[index].Child2

		area := dt.nodes[index].AABB.GetPerimeter()

		var combinedAABB AABB
		combinedAABB.Combine2(dt.nodes[index].AABB, leafAABB)
		combinedArea := combinedAABB.GetPerimeter()

		// Cost of creating a new parent for this node and the new leaf
		cost := 2.0 * combinedArea

		// Minimum cost of pushing the leaf further down the tree
		inheritanceCost := 2.0 * (combinedArea - area)

		// Cost of descending into child1
		var cost1 float64
		if dt.nodes[child1].IsLeaf() {
			var aabb AABB
			aabb.Combine2(leafAABB, dt.nodes[child1].AABB)
			cost1 = aabb.GetPerimeter() + inheritanceCost
		} else {
			var aabb AABB
			aabb.Combine2(leafAABB, dt.nodes[child1].AABB)
			oldArea := dt.nodes[child1].AABB.GetPerimeter()
			newArea := aabb.GetPerimeter()
			cost1 = (newArea - oldArea) + inheritanceCost
		}

		// Cost of descending into child2
		var cost2 float64
		if dt.nodes[child2].IsLeaf() {
			var aabb AABB
			aabb.Combine2(leafAABB, dt.nodes[child2].AABB)
			cost2 = aabb.GetPerimeter() + inheritanceCost
		} else {
			var aabb AABB
			aabb.Combine2(leafAABB, dt.nodes[child2].AABB)
			oldArea := dt.nodes[child2].AABB.GetPerimeter()
			newArea := aabb.GetPerimeter()
			cost2 = newArea - oldArea + inheritanceCost
		}

		// Descend according to the minimum cost.
		if cost < cost1 && cost < cost2 {
			break
		}

		// Descend
		if cost1 < cost2 {
			index = child1
		} else {
			index = child2
		}
	}

	sibling := index

	// Create a new parent.
	oldParent := dt.nodes[sibling].Parent
	newParent := dt.allocateNode()
	dt.nodes[newParent].Parent = oldParent
	dt.nodes[newParent].UserData = nil
	dt.nodes[newParent].AABB.Combine2(leafAABB, dt.nodes[sibling].AABB)
	dt.nodes[newParent].Height = dt.nodes[sibling].Height + 1

	if oldParent != NullNode {
		// The sibling was not the root.
		if dt.nodes[oldParent].Child1 == sibling {
			dt.nodes[oldParent].Child1 = newParent
		} else {
			dt.nodes[oldParent].Child2 = newParent
		}

		dt.nodes[newParent].Child1 = sibling
		dt.nodes[newParent].Child2 = leaf
		dt.nodes[sibling].Parent = newParent
		dt.nodes[leaf].Parent = newParent
	} else {
		// The sibling was the root.
		dt.nodes[newParent].Child1 = sibling
		dt.nodes[newParent].Child2 = leaf
		dt.nodes[sibling].Parent = newParent
		dt.nodes[leaf].Parent = newParent
		dt.root = newParent
	}

	// Walk back up the tree fixing heights and AABBs
	index = dt.nodes[leaf].Parent
	for index != NullNode {
		index = dt.balance(index)

		child1 := dt.nodes[index].Child1
		child2 := dt.nodes[index].Child2

		dt.nodes[index].Height = 1 + MaxI(dt.nodes[child1].Height, dt.nodes[child2].Height)
		dt.nodes[index].AABB.Combine2(dt.nodes[child1].AABB, dt.nodes[child2].AABB)

		index = dt.nodes[index].Parent
	}

	//Validate();
}

func (dt *DynamicTree) removeLeaf(leaf int) {
	if leaf == dt.root {
		dt.root = NullNode
		return
	}

	parent := dt.nodes[leaf].Parent
	grandParent := dt.nodes[parent].Parent
	var sibling int
	if dt.nodes[parent].Child1 == leaf {
		sibling = dt.nodes[parent].Child2
	} else {
		sibling = dt.nodes[parent].Child1
	}

	if grandParent != NullNode {
		// Destroy parent and connect sibling to grandParent.
		if dt.nodes[grandParent].Child1 == parent {
			dt.nodes[grandParent].Child1 = sibling
		} else {
			dt.nodes[grandParent].Child2 = sibling
		}
		dt.nodes[sibling].Parent = grandParent
		dt.freeNode(parent)

		// Adjust ancestor bounds.
		index := grandParent
		for index != NullNode {
			index = dt.balance(index)

			child1 := dt.nodes[index].Child1
			child2 := dt.nodes[index].Child2

			dt.nodes[index].AABB.Combine2(dt.nodes[child1].AABB, dt.nodes[child2].AABB)
			dt.nodes[index].Height = 1 + MaxI(dt.nodes[child1].Height, dt.nodes[child2].Height)

			index = dt.nodes[index].Parent
		}
	} else {
		dt.root = sibling
		dt.nodes[sibling].Parent = NullNode
		dt.freeNode(parent)
	}
}

func (dt *DynamicTree) balance(iA int) int {
	A := &dt.nodes[iA]
	if A.IsLeaf() || A.Height < 2 {
		return iA
	}

	iB := A.Child1
	iC := A.Child2

	B := &dt.nodes[iB]
	C := &dt.nodes[iC]

	balance := C.Height - B.Height

	// Rotate C up
	if balance > 1 {
		iF := C.Child1
		iG := C.Child2
		F := &dt.nodes[iF]
		G := &dt.nodes[iG]

		// Swap A and C
		C.Child1 = iA
		C.Parent = A.Parent
		A.Parent = iC

		// A's old parent should point to C
		if C.Parent != NullNode {
			if dt.nodes[C.Parent].Child1 == iA {
				dt.nodes[C.Parent].Child1 = iC
			} else {
				dt.nodes[C.Parent].Child2 = iC
			}
		} else {
			dt.root = iC
		}

		// Rotate
		if F.Height > G.Height {
			C.Child2 = iF
			A.Child2 = iG
			G.Parent = iA
			A.AABB.Combine2(B.AABB, G.AABB)
			C.AABB.Combine2(A.AABB, F.AABB)

			A.Height = 1 + MaxI(B.Height, G.Height)
			C.Height = 1 + MaxI(A.Height, F.Height)
		} else {
			C.Child2 = iG
			A.Child2 = iF
			F.Parent = iA
			A.AABB.Combine2(B.AABB, F.AABB)
			C.AABB.Combine2(A.AABB, G.AABB)

			A.Height = 1 + MaxI(B.Height, F.Height)
			C.Height = 1 + MaxI(A.Height, G.Height)
		}

		return iC
	}

	// Rotate B up
	if balance < -1 {
		iD := B.Child1
		iE := B.Child2
		D := &dt.nodes[iD]
		E := &dt.nodes[iE]

		// Swap A and B
		B.Child1 = iA
		B.Parent = A.Parent
		A.Parent = iB

		// A's old parent should point to B
		if B.Parent != NullNode {
			if dt.nodes[B.Parent].Child1 == iA {
				dt.nodes[B.Parent].Child1 = iB
			} else {
				dt.nodes[B.Parent].Child2 = iB
			}
		} else {
			dt.root = iB
		}

		// Rotate
		if D.Height > E.Height {
			B.Child2 = iD
			A.Child1 = iE
			E.Parent = iA
			A.AABB.Combine2(C.AABB, E.AABB)
			B.AABB.Combine2(A.AABB, D.AABB)

			A.Height = 1 + MaxI(C.Height, E.Height)
			B.Height = 1 + MaxI(A.Height, D.Height)
		} else {
			B.Child2 = iE
			A.Child1 = iD
			D.Parent = iA
			A.AABB.Combine2(C.AABB, D.AABB)
			B.AABB.Combine2(A.AABB, E.AABB)

			A.Height = 1 + MaxI(C.Height, D.Height)
			B.Height = 1 + MaxI(A.Height, E.Height)
		}

		return iB
	}

	return iA
}
