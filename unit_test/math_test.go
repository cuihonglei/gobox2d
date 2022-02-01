package main

import (
	"math"
	"testing"

	"github.com/cuihonglei/gobox2d/box2d"
)

func Test_math(t *testing.T) {

	// From issue #447
	var sweep box2d.Sweep
	sweep.LocalCenter.SetZero()
	sweep.C0.Set(-2.0, 4.0)
	sweep.C.Set(3.0, 8.0)
	sweep.A = 5.0
	sweep.Alpha0 = 0.0

	var transform box2d.Transform

	transform = sweep.GetTransform(0.0)
	if transform.P.X != sweep.C0.X {
		t.Fail()
	}
	if transform.P.Y != sweep.C0.Y {
		t.Fail()
	}
	if transform.Q.C != math.Cos(sweep.A0) {
		t.Fail()
	}
	if transform.Q.S != math.Sin(sweep.A0) {
		t.Fail()
	}

	transform = sweep.GetTransform(1.0)
	if transform.P.X != sweep.C.X {
		t.Fail()
	}
	if transform.P.Y != sweep.C.Y {
		t.Fail()
	}
	if transform.Q.C != math.Cos(sweep.A) {
		t.Fail()
	}
	if transform.Q.S != math.Sin(sweep.A) {
		t.Fail()
	}
}
