package box2d

import (
	"time"
)

// Timer for profiling.
type Timer struct {
	start int64
}

func NewTimer() *Timer {
	this := new(Timer)
	return this
}

// Reset the timer.
func (this *Timer) Reset() {
	this.start = time.Now().UnixNano()
}

// Get the time since construction or the last reset.
func (this *Timer) GetMilliseconds() float64 {
	return float64((time.Now().UnixNano() - this.start) / 1000000)
}
