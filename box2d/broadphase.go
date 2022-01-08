package box2d

import (
	"sort"
)

type Pair struct {
	ProxyIdA int32
	ProxyIdB int32
	Next     int32
}

// The broad-phase is used for computing pairs and performing volume queries and ray casts.
// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
// It is up to the client to consume the new pairs and to track subsequent overlap.
type BroadPhase struct {
	tree       *DynamicTree
	proxyCount int32

	moveBuffer   []int32
	moveCapacity int32
	moveCount    int32

	pairBuffer   []Pair
	pairCapacity int32
	pairCount    int32

	queryProxyId int32
}

const BroadPhase_e_nullProxy = -1

func NewBroadPhase() *BroadPhase {
	this := new(BroadPhase)
	this.tree = NewDynamicTree()

	this.pairCapacity = 16
	this.pairBuffer = make([]Pair, this.pairCapacity, this.pairCapacity)

	this.moveCapacity = 16
	this.moveBuffer = make([]int32, this.moveCapacity, this.moveCapacity)

	return this
}

// Create a proxy with an initial AABB. Pairs are not reported until
// UpdatePairs is called.
func (this *BroadPhase) CreateProxy(aabb AABB, userData interface{}) int32 {
	proxyId := this.tree.CreateProxy(aabb, userData)
	this.proxyCount++
	this.BufferMove(proxyId)
	return proxyId
}

// Destroy a proxy. It is up to the client to remove any pairs.
func (this *BroadPhase) DestroyProxy(proxyId int32) {
	this.UnBufferMove(proxyId)
	this.proxyCount--
	this.tree.DestroyProxy(proxyId)
}

// Call MoveProxy as many times as you like, then when you are done
// call UpdatePairs to finalized the proxy pairs (for your time step).
func (this *BroadPhase) MoveProxy(proxyId int32, aabb AABB, displacement Vec2) {
	buffer := this.tree.MoveProxy(proxyId, aabb, displacement)
	if buffer {
		this.BufferMove(proxyId)
	}
}

// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
func (this *BroadPhase) TouchProxy(proxyId int32) {
	this.BufferMove(proxyId)
}

func (this *BroadPhase) BufferMove(proxyId int32) {
	if this.moveCount == this.moveCapacity {
		oldBuffer := this.moveBuffer
		this.moveCapacity *= 2
		this.moveBuffer = make([]int32, this.moveCapacity, this.moveCapacity)
		copy(this.moveBuffer, oldBuffer)
	}

	this.moveBuffer[this.moveCount] = proxyId
	this.moveCount++
}

func (this *BroadPhase) UnBufferMove(proxyId int32) {
	for i := int32(0); i < this.moveCount; i++ {
		if this.moveBuffer[i] == proxyId {
			this.moveBuffer[i] = BroadPhase_e_nullProxy
			return
		}
	}
}

// This is called from DynamicTree.Query when we are gathering pairs.
func (this *BroadPhase) QueryCallback(proxyId int32) bool {
	// A proxy cannot form a pair with itself.
	if proxyId == this.queryProxyId {
		return true
	}

	// Grow the pair buffer as needed.
	if this.pairCount == this.pairCapacity {
		oldBuffer := this.pairBuffer
		this.pairCapacity *= 2
		this.pairBuffer = make([]Pair, this.pairCapacity, this.pairCapacity)
		copy(this.pairBuffer, oldBuffer)
	}

	this.pairBuffer[this.pairCount].ProxyIdA = MinI(proxyId, this.queryProxyId)
	this.pairBuffer[this.pairCount].ProxyIdB = MaxI(proxyId, this.queryProxyId)
	this.pairCount++

	return true
}

// Get the fat AABB for a proxy.
func (this *BroadPhase) GetFatAABB(proxyId int32) AABB {
	return this.tree.GetFatAABB(proxyId)
}

// Get user data from a proxy. Returns NULL if the id is invalid.
func (this *BroadPhase) GetUserData(proxyId int32) interface{} {
	return this.tree.GetUserData(proxyId)
}

// Test overlap of fat AABBs.
func (this *BroadPhase) TestOverlap(proxyIdA int32, proxyIdB int32) bool {
	aabbA := this.tree.GetFatAABB(proxyIdA)
	aabbB := this.tree.GetFatAABB(proxyIdB)
	return TestOverlapAABB(aabbA, aabbB)
}

// Get the number of proxies.
func (this *BroadPhase) GetProxyCount() int32 {
	return this.proxyCount
}

type PairSorter struct {
	pair  []Pair
	count int32
}

// This is used to sort pairs.
func (this PairSorter) Len() int {
	return int(this.count)
}

func (this PairSorter) Less(i, j int) bool {
	pair1 := &this.pair[i]
	pair2 := &this.pair[j]

	if pair1.ProxyIdA < pair2.ProxyIdA {
		return true
	}

	if pair1.ProxyIdA == pair2.ProxyIdA {
		return pair1.ProxyIdB < pair2.ProxyIdB
	}

	return false
}

func (this PairSorter) Swap(i, j int) {
	this.pair[i], this.pair[j] = this.pair[j], this.pair[i]
}

// Update the pairs. This results in pair callbacks. This can only add pairs.
func (this *BroadPhase) UpdatePairs(callback func(interface{}, interface{})) {
	// Reset pair buffer
	this.pairCount = 0

	// Perform tree queries for all moving proxies.
	for i := int32(0); i < this.moveCount; i++ {
		this.queryProxyId = this.moveBuffer[i]
		if this.queryProxyId == BroadPhase_e_nullProxy {
			continue
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		fatAABB := this.tree.GetFatAABB(this.queryProxyId)

		// Query tree, create pairs and add them pair buffer.
		this.tree.Query(this.QueryCallback, fatAABB)
	}

	// Reset move buffer
	this.moveCount = 0

	// Sort the pair buffer to expose duplicates.
	sort.Sort(PairSorter{this.pairBuffer, this.pairCount})

	// Send the pairs back to the client.
	for i := int32(0); i < this.pairCount; {
		primaryPair := &this.pairBuffer[i]
		userDataA := this.tree.GetUserData(primaryPair.ProxyIdA)
		userDataB := this.tree.GetUserData(primaryPair.ProxyIdB)

		callback(userDataA, userDataB)
		i++

		// Skip any duplicate pairs.
		for i < this.pairCount {
			pair := &this.pairBuffer[i]
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
func (this *BroadPhase) Query(callback func(int32) bool, aabb AABB) {
	this.tree.Query(callback, aabb)
}

// Ray-cast against the proxies in the tree. This relies on the callback
// to perform a exact ray-cast in the case were the proxy contains a shape.
// The callback also performs the any collision filtering. This has performance
// roughly equal to k * log(n), where k is the number of collisions and n is the
// number of proxies in the tree.
// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
// @param callback a callback class that is called for each proxy that is hit by the ray.
func (this *BroadPhase) RayCast(callback func(RayCastInput, int32) float64, input RayCastInput) {
	this.tree.RayCast(callback, input)
}

// Get the height of the embedded tree.
func (this *BroadPhase) GetTreeHeight() int32 {
	return this.tree.GetHeight()
}

// Get the balance of the embedded tree.
func (this *BroadPhase) GetTreeBalance() int32 {
	return this.tree.GetMaxBalance()
}

// Get the quality metric of the embedded tree.
func (this *BroadPhase) GetTreeQuality() float64 {
	return this.tree.GetAreaRatio()
}
