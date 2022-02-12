package box2d

import (
	"time"
)

// Timer for profiling.
type Timer struct {
	start int64
}

func MakeTimer() Timer {
	t := Timer{}
	t.Reset()
	return t
}

func NewTimer() *Timer {
	t := MakeTimer()
	return &t
}

// Reset the timer.
func (t *Timer) Reset() {
	t.start = time.Now().UnixNano()
}

// Get the time since construction or the last reset.
func (t *Timer) GetMilliseconds() float64 {
	return float64(time.Now().UnixNano()-t.start) / float64(time.Millisecond)
}
