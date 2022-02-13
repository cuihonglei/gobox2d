package box2d

type GrowableStack struct {
	Stack    []interface{}
	Count    int
	Capacity int
}

func NewGrowableStack(capacity int) *GrowableStack {
	this := new(GrowableStack)
	this.Capacity = capacity
	this.Stack = make([]interface{}, capacity)
	return this
}

func (gs *GrowableStack) Push(element interface{}) {
	if gs.Count == gs.Capacity {
		old := gs.Stack
		gs.Capacity *= 2
		gs.Stack = make([]interface{}, gs.Capacity)
		copy(gs.Stack, old)
	}

	gs.Stack[gs.Count] = element
	gs.Count++
}

func (gs *GrowableStack) Pop() interface{} {
	gs.Count--
	return gs.Stack[gs.Count]
}

func (gs *GrowableStack) GetCount() int {
	return gs.Count
}
