package main

type BodyTypes struct {
	Test
}

func CreateBodyTypes() ITest {
	return new(BodyTypes)
}

func (bt *BodyTypes) step(settings *Settings) {

	// TODO

	bt.Test.step(settings)
}

var _ = RegisterTest("Examples", "Body Types", CreateBodyTypes)
