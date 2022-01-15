package main

import (
	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/glfw/v3.3/glfw"
)

type Camera struct {
	center box2d.Vec2
	zoom   float64
	width  int
	height int
}

// This class implements debug drawing callbacks that are invoked
// inside b2World::Step.
type DebugDraw struct {
	box2d.Draw
}

func (dd *DebugDraw) Create() {

}

func (dd *DebugDraw) Destroy() {

}

func (dd *DebugDraw) DrawPolygon(vertices []box2d.Vec2, color box2d.Color) {

}

func (dd *DebugDraw) DrawSolidPolygon(vertices []box2d.Vec2, color box2d.Color) {

}

func (dd *DebugDraw) DrawCircle(center box2d.Vec2, radius float64, color box2d.Color) {

}

func (dd *DebugDraw) DrawSolidCircle(center box2d.Vec2, radius float64, axis box2d.Vec2, color box2d.Color) {

}

func (dd *DebugDraw) DrawSegment(p1 box2d.Vec2, p2 box2d.Vec2, color box2d.Color) {

}

func (dd *DebugDraw) DrawTransform(xf box2d.Transform) {

}

var g_debugDraw DebugDraw
var g_camera Camera
var g_mainWindow *glfw.Window
