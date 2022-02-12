package box2d

import (
	"sort"
)

type Pair struct {
	ProxyIdA int
	ProxyIdB int
	Next     int
}

// The broad-phase is used for computing pairs and performing volume queries and ray casts.
// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
// It is up to the client to consume the new pairs and to track subsequent overlap.
type BroadPhase struct {
	tree       *DynamicTree
	proxyCount int

	moveBuffer   []int
	moveCapacity int
	moveCount    int

	pairBuffer   []Pair
	pairCapacity int
	pairCount    int

	queryProxyId int
}

const BroadPhase_e_nullProxy = -1

func NewBroadPhase() *BroadPhase {
	bp := new(BroadPhase)
	bp.tree = NewDynamicTree()

	bp.pairCapacity = 16
	bp.pairBuffer = make([]Pair, bp.pairCapacity)

	bp.moveCapacity = 16
	bp.moveBuffer = make([]int, bp.moveCapacity)

	return bp
}

// Create a proxy with an initial AABB. Pairs are not reported until
// UpdatePairs is called.
func (bp *BroadPhase) CreateProxy(aabb AABB, userData interface{}) int {
	proxyId := bp.tree.CreateProxy(aabb, userData)
	bp.proxyCount++
	bp.BufferMove(proxyId)
	return proxyId
}

// Destroy a proxy. It is up to the client to remove any pairs.
func (bp *BroadPhase) DestroyProxy(proxyId int) {
	bp.UnBufferMove(proxyId)
	bp.proxyCount--
	bp.tree.DestroyProxy(proxyId)
}

// Call MoveProxy as many times as you like, then when you are done
// call UpdatePairs to finalized the proxy pairs (for your time step).
func (bp *BroadPhase) MoveProxy(proxyId int, aabb AABB, displacement Vec2) {
	buffer := bp.tree.MoveProxy(proxyId, aabb, displacement)
	if buffer {
		bp.BufferMove(proxyId)
	}
}

// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
func (bp *BroadPhase) TouchProxy(proxyId int) {
	bp.BufferMove(proxyId)
}

func (bp *BroadPhase) BufferMove(proxyId int) {
	if bp.moveCount == bp.moveCapacity {
		oldBuffer := bp.moveBuffer
		bp.moveCapacity *= 2
		bp.moveBuffer = make([]int, bp.moveCapacity)
		copy(bp.moveBuffer, oldBuffer)
	}

	bp.moveBuffer[bp.moveCount] = proxyId
	bp.moveCount++
}

func (bp *BroadPhase) UnBufferMove(proxyId int) {
	for i := 0; i < bp.moveCount; i++ {
		if bp.moveBuffer[i] == proxyId {
			bp.moveBuffer[i] = BroadPhase_e_nullProxy
			return
		}
	}
}

// This is called from DynamicTree.Query when we are gathering pairs.
func (bp *BroadPhase) QueryCallback(proxyId int) bool {
	// A proxy cannot form a pair with itself.
	if proxyId == bp.queryProxyId {
		return true
	}

	// Grow the pair buffer as needed.
	if bp.pairCount == bp.pairCapacity {
		oldBuffer := bp.pairBuffer
		bp.pairCapacity *= 2
		bp.pairBuffer = make([]Pair, bp.pairCapacity)
		copy(bp.pairBuffer, oldBuffer)
	}

	bp.pairBuffer[bp.pairCount].ProxyIdA = MinI(proxyId, bp.queryProxyId)
	bp.pairBuffer[bp.pairCount].ProxyIdB = MaxI(proxyId, bp.queryProxyId)
	bp.pairCount++

	return true
}

// Get the fat AABB for a proxy.
func (bp *BroadPhase) GetFatAABB(proxyId int) AABB {
	return bp.tree.GetFatAABB(proxyId)
}

// Get user data from a proxy. Returns NULL if the id is invalid.
func (bp *BroadPhase) GetUserData(proxyId int) interface{} {
	return bp.tree.GetUserData(proxyId)
}

// Test overlap of fat AABBs.
func (bp *BroadPhase) TestOverlap(proxyIdA int, proxyIdB int) bool {
	aabbA := bp.tree.GetFatAABB(proxyIdA)
	aabbB := bp.tree.GetFatAABB(proxyIdB)
	return TestOverlapAABB(aabbA, aabbB)
}

// Get the number of proxies.
func (bp *BroadPhase) GetProxyCount() int {
	return bp.proxyCount
}

type PairSorter struct {
	pair  []Pair
	count int
}

// This is used to sort pairs.
func (bp PairSorter) Len() int {
	return bp.count
}

func (bp PairSorter) Less(i, j int) bool {
	pair1 := &bp.pair[i]
	pair2 := &bp.pair[j]

	if pair1.ProxyIdA < pair2.ProxyIdA {
		return true
	}

	if pair1.ProxyIdA == pair2.ProxyIdA {
		return pair1.ProxyIdB < pair2.ProxyIdB
	}

	return false
}

func (bp PairSorter) Swap(i, j int) {
	bp.pair[i], bp.pair[j] = bp.pair[j], bp.pair[i]
}

// Update the pairs. This results in pair callbacks. This can only add pairs.
func (bp *BroadPhase) UpdatePairs(callback func(interface{}, interface{})) {
	// Reset pair buffer
	bp.pairCount = 0

	// Perform tree queries for all moving proxies.
	for i := 0; i < bp.moveCount; i++ {
		bp.queryProxyId = bp.moveBuffer[i]
		if bp.queryProxyId == BroadPhase_e_nullProxy {
			continue
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		fatAABB := bp.tree.GetFatAABB(bp.queryProxyId)

		// Query tree, create pairs and add them pair buffer.
		bp.tree.Query(bp.QueryCallback, fatAABB)
	}

	// Reset move buffer
	bp.moveCount = 0

	// Sort the pair buffer to expose duplicates.
	sort.Sort(PairSorter{bp.pairBuffer, bp.pairCount})

	// Send the pairs back to the client.
	for i := 0; i < bp.pairCount; {
		primaryPair := &bp.pairBuffer[i]
		userDataA := bp.tree.GetUserData(primaryPair.ProxyIdA)
		userDataB := bp.tree.GetUserData(primaryPair.ProxyIdB)

		callback(userDataA, userDataB)
		i++

		// Skip any duplicate pairs.
		for i < bp.pairCount {
			pair := &bp.pairBuffer[i]
			if pair.ProxyIdA != primaryPair.ProxyIdA || pair.ProxyIdB != primaryPair.ProxyIdB {
				break
			}
			i++
		}
	}

	// Try to keep the tree balanced.
	//m_tree.Rebalance(4);
}

// Query an AABB for overlapping proxies. The callback class
// is called for each proxy that overlaps the supplied AABB.
func (bp *BroadPhase) Query(callback func(int) bool, aabb AABB) {
	bp.tree.Query(callback, aabb)
}

// Ray-cast against the proxies in the tree. This relies on the callback
// to perform a exact ray-cast in the case were the proxy contains a shape.
// The callback also performs the any collision filtering. This has performance
// roughly equal to k * log(n), where k is the number of collisions and n is the
// number of proxies in the tree.
// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
// @param callback a callback class that is called for each proxy that is hit by the ray.
func (bp *BroadPhase) RayCast(callback func(RayCastInput, int) float64, input RayCastInput) {
	bp.tree.RayCast(callback, input)
}

// Get the height of the embedded tree.
func (bp *BroadPhase) GetTreeHeight() int {
	return bp.tree.GetHeight()
}

// Get the balance of the embedded tree.
func (bp *BroadPhase) GetTreeBalance() int {
	return bp.tree.GetMaxBalance()
}

// Get the quality metric of the embedded tree.
func (bp *BroadPhase) GetTreeQuality() float64 {
	return bp.tree.GetAreaRatio()
}
