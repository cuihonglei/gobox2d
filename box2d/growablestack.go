package box2d

type GrowableStack struct {
	Stack    []interface{}
	Count    int
	Capacity int
}

func NewGrowableStack(capacity int) *GrowableStack {
	this := new(GrowableStack)
	this.Capacity = capacity
	this.Stack = make([]interface{}, capacity, capacity)
	return this
}

func (this *GrowableStack) Push(element interface{}) {
	if this.Count == this.Capacity {
		old := this.Stack
		this.Capacity *= 2
		this.Stack = make([]interface{}, this.Capacity, this.Capacity)
		copy(this.Stack, old)
	}

	this.Stack[this.Count] = element
	this.Count++
}

func (this *GrowableStack) Pop() interface{} {
	this.Count--
	return this.Stack[this.Count]
}

func (this *GrowableStack) GetCount() int {
	return this.Count
}
