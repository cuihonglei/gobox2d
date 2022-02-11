package main

type Rope struct {
	Test

	// TODO
	//rope1 *box2d.Rope
	//tuning1 *box2d.RopeTuning
}

func CreateRope() ITest {
	r := &Rope{}
	r.Test.init(r)

	return r
}

var _ = RegisterTest("Rope", "Bending", CreateRope)
