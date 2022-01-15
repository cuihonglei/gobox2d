package main

import (
	"fmt"
	"os"
	"unsafe"

	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/gl/v2.1/gl"
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

func (c *Camera) buildProjectionMatrix(m []float32, zBias float32) {

	w := float64(c.width)
	h := float64(c.height)
	ratio := w / h
	extents := box2d.Vec2{X: ratio * 25.0, Y: 25.0}
	extents.Mul(c.zoom)

	lower := box2d.SubVV(c.center, extents)
	upper := box2d.AddVV(c.center, extents)

	m[0] = float32(2.0 / (upper.X - lower.X))
	m[1] = 0.0
	m[2] = 0.0
	m[3] = 0.0

	m[4] = 0.0
	m[5] = float32(2.0 / (upper.Y - lower.Y))
	m[6] = 0.0
	m[7] = 0.0

	m[8] = 0.0
	m[9] = 0.0
	m[10] = 1.0
	m[11] = 0.0

	m[12] = float32(-(upper.X + lower.X) / (upper.X - lower.X))
	m[13] = float32(-(upper.Y + lower.Y) / (upper.Y - lower.Y))
	m[14] = zBias
	m[15] = 1.0
}

//
func sCheckGLError() {

	errCode := gl.GetError()
	if errCode != gl.NO_ERROR {
		fmt.Fprintf(os.Stderr, "OpenGL error = %d\n", errCode)
		//assert(false)
	}
}

//
func sPrintLog(object uint32) {
	// TODO
}

//
func sCreateShaderFromString(source string, xtype uint32) uint32 {

	res := gl.CreateShader(xtype)
	csources, free := gl.Strs(source)
	gl.ShaderSource(res, 1, csources, nil)
	free()
	gl.CompileShader(res)
	var compile_ok int32
	gl.GetShaderiv(res, gl.COMPILE_STATUS, &compile_ok)
	if compile_ok == gl.FALSE {
		fmt.Fprintf(os.Stderr, "Error compiling shader of type %d!\n", xtype)
		sPrintLog(res)
		gl.DeleteShader(res)
		return 0
	}

	return res
}

//
func sCreateShaderProgram(vs, fs string) uint32 {

	vsId := sCreateShaderFromString(vs, gl.VERTEX_SHADER)
	fsId := sCreateShaderFromString(fs, gl.FRAGMENT_SHADER)
	//assert(vsId != 0 && fsId != 0)

	programId := gl.CreateProgram()
	gl.AttachShader(programId, vsId)
	gl.AttachShader(programId, fsId)
	gl.BindFragDataLocationEXT(programId, 0, gl.Str("color\x00"))
	gl.LinkProgram(programId)

	gl.DeleteShader(vsId)
	gl.DeleteShader(fsId)

	var status int32
	gl.GetProgramiv(programId, gl.LINK_STATUS, &status)
	//assert(status != GL_FALSE)

	return programId
}

//
const GLRenderPoints_e_maxVertices = 512

type GLRenderPoints struct {
	vertices [GLRenderPoints_e_maxVertices]box2d.Vec2
	colors   [GLRenderPoints_e_maxVertices]box2d.Color
	sizes    [GLRenderPoints_e_maxVertices]float64

	count int
}

func NewGLRenderPoints() *GLRenderPoints {
	rp := new(GLRenderPoints)

	return rp
}

func (rp *GLRenderPoints) create() {

}

func (rl *GLRenderPoints) destroy() {

}

func (rl *GLRenderPoints) vertex(v box2d.Vec2, c box2d.Color, size float64) {

}

func (rl *GLRenderPoints) flush() {

}

//
const GLRenderLines_e_maxVertices = 2 * 512

type GLRenderLines struct {
	vertices [GLRenderLines_e_maxVertices]box2d.Vec2
	colors   [GLRenderLines_e_maxVertices]box2d.Color

	count int

	vaoId             uint32
	vboIds            [2]uint32
	programId         uint32
	projectionUniform int32
	vertexAttribute   int32
	colorAttribute    int32
}

func NewGLRenderLines() *GLRenderLines {
	rl := new(GLRenderLines)

	return rl
}

const GLRenderLines_vs = `
#version 330
uniform mat4 projectionMatrix;
layout(location = 0) in vec2 v_position;
layout(location = 1) in vec4 v_color;
out vec4 f_color;
void main(void)
{
	f_color = v_color;
	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);
}
` + "\x00"

const GLRenderLines_fs = `
#version 330
in vec4 f_color;
out vec4 color;
void main(void)
{
	color = f_color;
}
` + "\x00"

func (rl *GLRenderLines) create() {
	rl.programId = sCreateShaderProgram(GLRenderLines_vs, GLRenderLines_fs)
	rl.projectionUniform = gl.GetUniformLocation(rl.programId, gl.Str("projectionMatrix\x00"))
	rl.vertexAttribute = 0
	rl.colorAttribute = 1

	// Generate
	gl.GenVertexArrays(1, &rl.vaoId)
	gl.GenBuffers(2, &rl.vboIds[0])

	gl.BindVertexArray(rl.vaoId)
	gl.EnableVertexAttribArray(uint32(rl.vertexAttribute))
	gl.EnableVertexAttribArray(uint32(rl.colorAttribute))

	// Vertex buffer
	gl.BindBuffer(gl.ARRAY_BUFFER, rl.vboIds[0])
	gl.VertexAttribPointer(uint32(rl.vertexAttribute), 2, gl.DOUBLE, false, 0, nil)
	gl.BufferData(gl.ARRAY_BUFFER, 16*1024, unsafe.Pointer(&rl.vertices[0]), gl.DYNAMIC_DRAW)

	gl.BindBuffer(gl.ARRAY_BUFFER, rl.vboIds[1])
	gl.VertexAttribPointer(uint32(rl.colorAttribute), 4, gl.DOUBLE, false, 0, nil)
	gl.BufferData(gl.ARRAY_BUFFER, 32*1024, unsafe.Pointer(&rl.colors[0]), gl.DYNAMIC_DRAW)

	sCheckGLError()

	// Cleanup
	gl.BindBuffer(gl.ARRAY_BUFFER, 0)
	gl.BindVertexArray(0)

	rl.count = 0
}

func (rl *GLRenderLines) destroy() {

	if rl.vaoId > 0 {
		gl.DeleteVertexArrays(1, &rl.vaoId)
		gl.DeleteBuffers(2, &rl.vboIds[0])
		rl.vaoId = 0
	}

	if rl.programId > 0 {
		gl.DeleteProgram(rl.programId)
		rl.programId = 0
	}
}

func (rl *GLRenderLines) vertex(v box2d.Vec2, c box2d.Color) {
	if rl.count == GLRenderLines_e_maxVertices {
		rl.flush()
	}

	rl.vertices[rl.count] = v
	rl.colors[rl.count] = c
	rl.count += 1
}

func (rl *GLRenderLines) flush() {

	if rl.count == 0 {
		return
	}

	gl.UseProgram(rl.programId)

	var proj [16]float32
	g_camera.buildProjectionMatrix(proj[:], 0.1)

	gl.UniformMatrix4fv(rl.projectionUniform, 1, false, &proj[0])

	gl.BindVertexArray(rl.vaoId)

	gl.BindBuffer(gl.ARRAY_BUFFER, rl.vboIds[0])
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, 16, unsafe.Pointer(&rl.vertices[0]))

	gl.BindBuffer(gl.ARRAY_BUFFER, rl.vboIds[1])
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, 32, unsafe.Pointer(&rl.colors[0]))

	gl.DrawArrays(gl.LINES, 0, int32(rl.count))

	sCheckGLError()

	gl.BindBuffer(gl.ARRAY_BUFFER, 0)
	gl.BindVertexArray(0)
	gl.UseProgram(0)

	rl.count = 0
}

//
const GLRenderTriangles_e_maxVertices = 3 * 512

type GLRenderTriangles struct {
	vertices [GLRenderTriangles_e_maxVertices]box2d.Vec2
	colors   [GLRenderTriangles_e_maxVertices]box2d.Color

	count int
}

func NewGLRenderTriangles() *GLRenderTriangles {
	rr := new(GLRenderTriangles)

	return rr
}

func (rr *GLRenderTriangles) create() {

}

func (rr *GLRenderTriangles) destroy() {

}

func (rr *GLRenderTriangles) vertex(v box2d.Vec2, c box2d.Color) {

}

func (rr *GLRenderTriangles) flush() {

}

// This class implements debug drawing callbacks that are invoked
// inside b2World::Step.
type DebugDraw struct {
	box2d.Draw

	showUI    bool
	points    *GLRenderPoints
	lines     *GLRenderLines
	triangles *GLRenderTriangles
}

func MakeDebugDraw() DebugDraw {
	return DebugDraw{
		showUI: true,
	}
}

//
func (dd *DebugDraw) create() {

	dd.points = NewGLRenderPoints()
	dd.points.create()
	dd.lines = NewGLRenderLines()
	dd.lines.create()
	dd.triangles = NewGLRenderTriangles()
	dd.triangles.create()
}

//
func (dd *DebugDraw) destroy() {

	dd.points.destroy()
	dd.points = nil

	dd.lines.destroy()
	dd.lines = nil

	dd.triangles.destroy()
	dd.triangles = nil
}

//
func (dd *DebugDraw) DrawPolygon(vertices []box2d.Vec2, color box2d.Color) {

	vertexCount := len(vertices)
	p1 := vertices[vertexCount-1]
	for i := 0; i < vertexCount; i++ {
		p2 := vertices[i]
		dd.lines.vertex(p1, color)
		dd.lines.vertex(p2, color)
		p1 = p2
	}
}

//
func (dd *DebugDraw) DrawSolidPolygon(vertices []box2d.Vec2, color box2d.Color) {

	vertexCount := len(vertices)
	fillColor := box2d.MakeColor2(0.5*color.R, 0.5*color.G, 0.5*color.B, 0.5)

	for i := 1; i < vertexCount-1; i++ {
		dd.triangles.vertex(vertices[0], fillColor)
		dd.triangles.vertex(vertices[i], fillColor)
		dd.triangles.vertex(vertices[i+1], fillColor)
	}

	p1 := vertices[vertexCount-1]
	for i := 0; i < vertexCount; i++ {
		p2 := vertices[i]
		dd.lines.vertex(p1, color)
		dd.lines.vertex(p2, color)
		p1 = p2
	}
}

//
func (dd *DebugDraw) DrawCircle(center box2d.Vec2, radius float64, color box2d.Color) {
	fmt.Println("DebugDraw.DrawCircle")
}

//
func (dd *DebugDraw) DrawSolidCircle(center box2d.Vec2, radius float64, axis box2d.Vec2, color box2d.Color) {
	fmt.Println("DebugDraw.DrawSolidCircle")
}

//
func (dd *DebugDraw) DrawSegment(p1 box2d.Vec2, p2 box2d.Vec2, color box2d.Color) {

	dd.lines.vertex(p1, color)
	dd.lines.vertex(p2, color)
}

//
func (dd *DebugDraw) DrawTransform(xf box2d.Transform) {
	fmt.Println("DebugDraw.DrawTransform")
}

//
func (dd *DebugDraw) DrawPoint(p box2d.Vec2, size float64, color box2d.Color) {

	dd.points.vertex(p, color, size)
}

//
func (dd *DebugDraw) DrawString(x, y int, xstring string, a ...interface{}) {
	fmt.Println("DebugDraw.DrawString")

	if !dd.showUI {
		return
	}
}

//
func (dd *DebugDraw) DrawString2(pw *box2d.Vec2, xstring string, a ...interface{}) {
	fmt.Println("DebugDraw.DrawString2")
}

//
func (dd *DebugDraw) flush() {
	dd.triangles.flush()
	dd.lines.flush()
	dd.points.flush()
}

var g_debugDraw = MakeDebugDraw()
var g_camera = MakeCamera()
