package main

import (
	"fmt"

	"github.com/cuihonglei/gobox2d/box2d"
)

//
type Camera struct {
	center box2d.Vec2
	zoom   float64
	width  int
	height int
}

func MakeCamera() Camera {
	c := Camera{}
	c.center.Set(0.0, 20.0)
	c.zoom = 1.0
	c.width = 1280
	c.height = 800
	return c
}

func (c *Camera) convertScreenToWorld(screenPoint box2d.Vec2) box2d.Vec2 {
	// TODO
	return screenPoint
}

func (c *Camera) convertWorldToScreen(worldPoint box2d.Vec2) box2d.Vec2 {
	// TODO
	return worldPoint
}

func (c *Camera) buildProjectionMatrix(m *float64, zBias float64) {
	// TODO
}

// This class implements debug drawing callbacks that are invoked
// inside b2World::Step.
type DebugDraw struct {
	box2d.Draw

	showUI bool
}

func MakeDebugDraw() DebugDraw {
	return DebugDraw{
		showUI: true,
	}
}

func (dd *DebugDraw) create() {
	fmt.Println("DebugDraw.create")
}

func (dd *DebugDraw) destroy() {
	fmt.Println("DebugDraw.destroy")
}

func (dd *DebugDraw) DrawPolygon(vertices []box2d.Vec2, color box2d.Color) {
	fmt.Println("DebugDraw.DrawPolygon")
}

func (dd *DebugDraw) DrawSolidPolygon(vertices []box2d.Vec2, color box2d.Color) {
	//fmt.Println("DebugDraw.DrawSolidPolygon")
}

func (dd *DebugDraw) DrawCircle(center box2d.Vec2, radius float64, color box2d.Color) {
	fmt.Println("DebugDraw.DrawCircle")
}

func (dd *DebugDraw) DrawSolidCircle(center box2d.Vec2, radius float64, axis box2d.Vec2, color box2d.Color) {
	fmt.Println("DebugDraw.DrawSolidCircle")
}

func (dd *DebugDraw) DrawSegment(p1 box2d.Vec2, p2 box2d.Vec2, color box2d.Color) {
	//fmt.Println("DebugDraw.DrawSegment")
}

func (dd *DebugDraw) DrawTransform(xf box2d.Transform) {
	fmt.Println("DebugDraw.DrawTransform")
}

func (dd *DebugDraw) DrawString(x, y int, istring string, a ...interface{}) {
	fmt.Println("DebugDraw.DrawString")
}

func (dd *DebugDraw) DrawString2(pw *box2d.Vec2, istring string, a ...interface{}) {
	fmt.Println("DebugDraw.DrawString2")
}

func (dd *DebugDraw) Flush() {
	//fmt.Println("DebugDraw.Flush")
	// TODO
}

var g_debugDraw = MakeDebugDraw()
var g_camera = MakeCamera()
