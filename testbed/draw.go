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

	vaoId             uint32
	vboIds            [3]uint32
	programId         uint32
	projectionUniform int32
	vertexAttribute   int32
	colorAttribute    int32
	sizeAttribute     int32
}

const GLRenderPoints_vs = `
#version 330
uniform mat4 projectionMatrix;
layout(location = 0) in vec2 v_position;
layout(location = 1) in vec4 v_color;
layout(location = 2) in float v_size;
out vec4 f_color;
void main(void)
{
	f_color = v_color;
	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);
   	gl_PointSize = v_size;
}
` + "\x00"

const GLRenderPoints_fs = `
#version 330
in vec4 f_color;
out vec4 color;
void main(void)
{
	color = f_color;
}
` + "\x00"

func (rp *GLRenderPoints) create() {

	rp.programId = sCreateShaderProgram(GLRenderPoints_vs, GLRenderPoints_fs)
	rp.projectionUniform = gl.GetUniformLocation(rp.programId, gl.Str("projectionMatrix\x00"))
	rp.vertexAttribute = 0
	rp.colorAttribute = 1
	rp.sizeAttribute = 2

	// Generate
	gl.GenVertexArrays(1, &rp.vaoId)
	gl.GenBuffers(3, &rp.vboIds[0])

	gl.BindVertexArray(rp.vaoId)
	gl.EnableVertexAttribArray(uint32(rp.vertexAttribute))
	gl.EnableVertexAttribArray(uint32(rp.colorAttribute))
	gl.EnableVertexAttribArray(uint32(rp.sizeAttribute))

	// Vertex buffer
	gl.BindBuffer(gl.ARRAY_BUFFER, rp.vboIds[0])
	gl.VertexAttribPointer(uint32(rp.vertexAttribute), 2, gl.DOUBLE, false, 0, nil)
	gl.BufferData(gl.ARRAY_BUFFER, int(unsafe.Sizeof(rp.vertices)), unsafe.Pointer(&rp.vertices[0]), gl.DYNAMIC_DRAW)

	gl.BindBuffer(gl.ARRAY_BUFFER, rp.vboIds[1])
	gl.VertexAttribPointer(uint32(rp.colorAttribute), 4, gl.DOUBLE, false, 0, nil)
	gl.BufferData(gl.ARRAY_BUFFER, int(unsafe.Sizeof(rp.colors)), unsafe.Pointer(&rp.colors[0]), gl.DYNAMIC_DRAW)

	gl.BindBuffer(gl.ARRAY_BUFFER, rp.vboIds[2])
	gl.VertexAttribPointer(uint32(rp.sizeAttribute), 1, gl.DOUBLE, false, 0, nil)
	gl.BufferData(gl.ARRAY_BUFFER, int(unsafe.Sizeof(rp.sizes)), unsafe.Pointer(&rp.sizes[0]), gl.DYNAMIC_DRAW)

	sCheckGLError()

	// Cleanup
	gl.BindBuffer(gl.ARRAY_BUFFER, 0)
	gl.BindVertexArray(0)

	rp.count = 0
}

func (rp *GLRenderPoints) destroy() {

	if rp.vaoId > 0 {
		gl.DeleteVertexArrays(1, &rp.vaoId)
		gl.DeleteBuffers(3, &rp.vboIds[0])
		rp.vaoId = 0
	}

	if rp.programId > 0 {
		gl.DeleteProgram(rp.programId)
		rp.programId = 0
	}
}

func (rp *GLRenderPoints) vertex(v box2d.Vec2, c box2d.Color, size float64) {

	if rp.count == GLRenderPoints_e_maxVertices {
		rp.flush()
	}

	rp.vertices[rp.count] = v
	rp.colors[rp.count] = c
	rp.sizes[rp.count] = size
	rp.count += 1
}

func (rp *GLRenderPoints) flush() {

	if rp.count == 0 {
		return
	}

	gl.UseProgram(rp.programId)

	var proj [16]float32
	g_camera.buildProjectionMatrix(proj[:], 0.1)

	gl.UniformMatrix4fv(rp.projectionUniform, 1, false, &proj[0])

	gl.BindVertexArray(rp.vaoId)

	gl.BindBuffer(gl.ARRAY_BUFFER, rp.vboIds[0])
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, rp.count*int(unsafe.Sizeof(box2d.Vec2{})), unsafe.Pointer(&rp.vertices[0]))

	gl.BindBuffer(gl.ARRAY_BUFFER, rp.vboIds[1])
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, rp.count*int(unsafe.Sizeof(box2d.Color{})), unsafe.Pointer(&rp.colors[0]))

	gl.BindBuffer(gl.ARRAY_BUFFER, rp.vboIds[2])
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, rp.count*int(unsafe.Sizeof(float64(0))), unsafe.Pointer(&rp.sizes[0]))

	gl.Enable(gl.PROGRAM_POINT_SIZE_EXT)
	gl.DrawArrays(gl.POINTS, 0, int32(rp.count))
	gl.Disable(gl.PROGRAM_POINT_SIZE_EXT)

	sCheckGLError()

	gl.BindBuffer(gl.ARRAY_BUFFER, 0)
	gl.BindVertexArray(0)
	gl.UseProgram(0)

	rp.count = 0
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
	gl.BufferData(gl.ARRAY_BUFFER, int(unsafe.Sizeof(rl.vertices)), unsafe.Pointer(&rl.vertices[0]), gl.DYNAMIC_DRAW)

	gl.BindBuffer(gl.ARRAY_BUFFER, rl.vboIds[1])
	gl.VertexAttribPointer(uint32(rl.colorAttribute), 4, gl.DOUBLE, false, 0, nil)
	gl.BufferData(gl.ARRAY_BUFFER, int(unsafe.Sizeof(rl.colors)), unsafe.Pointer(&rl.colors[0]), gl.DYNAMIC_DRAW)

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
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, rl.count*int(unsafe.Sizeof(box2d.Vec2{})), unsafe.Pointer(&rl.vertices[0]))

	gl.BindBuffer(gl.ARRAY_BUFFER, rl.vboIds[1])
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, rl.count*int(unsafe.Sizeof(box2d.Color{})), unsafe.Pointer(&rl.colors[0]))

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

	vaoId             uint32
	vboIds            [2]uint32
	programId         uint32
	projectionUniform int32
	vertexAttribute   int32
	colorAttribute    int32
}

const GLRenderTriangles_vs = `
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

const GLRenderTriangles_fs = `
#version 330
in vec4 f_color;
out vec4 color;
void main(void)
{
	color = f_color;
}
` + "\x00"

func (rt *GLRenderTriangles) create() {

	rt.programId = sCreateShaderProgram(GLRenderTriangles_vs, GLRenderTriangles_fs)
	rt.projectionUniform = gl.GetUniformLocation(rt.programId, gl.Str("projectionMatrix\x00"))
	rt.vertexAttribute = 0
	rt.colorAttribute = 1

	// Generate
	gl.GenVertexArrays(1, &rt.vaoId)
	gl.GenBuffers(2, &rt.vboIds[0])

	gl.BindVertexArray(rt.vaoId)
	gl.EnableVertexAttribArray(uint32(rt.vertexAttribute))
	gl.EnableVertexAttribArray(uint32(rt.colorAttribute))

	// Vertex buffer
	gl.BindBuffer(gl.ARRAY_BUFFER, rt.vboIds[0])
	gl.VertexAttribPointer(uint32(rt.vertexAttribute), 2, gl.DOUBLE, false, 0, nil)
	gl.BufferData(gl.ARRAY_BUFFER, int(unsafe.Sizeof(rt.vertices)), unsafe.Pointer(&rt.vertices[0]), gl.DYNAMIC_DRAW)

	gl.BindBuffer(gl.ARRAY_BUFFER, rt.vboIds[1])
	gl.VertexAttribPointer(uint32(rt.colorAttribute), 4, gl.DOUBLE, false, 0, nil)
	gl.BufferData(gl.ARRAY_BUFFER, int(unsafe.Sizeof(rt.colors)), unsafe.Pointer(&rt.colors[0]), gl.DYNAMIC_DRAW)

	sCheckGLError()

	// Cleanup
	gl.BindBuffer(gl.ARRAY_BUFFER, 0)
	gl.BindVertexArray(0)

	rt.count = 0
}

func (rt *GLRenderTriangles) destroy() {

	if rt.vaoId > 0 {
		gl.DeleteVertexArrays(1, &rt.vaoId)
		gl.DeleteBuffers(2, &rt.vboIds[0])
		rt.vaoId = 0
	}

	if rt.programId > 0 {
		gl.DeleteProgram(rt.programId)
		rt.programId = 0
	}
}

func (rt *GLRenderTriangles) vertex(v box2d.Vec2, c box2d.Color) {

	if rt.count == GLRenderTriangles_e_maxVertices {
		rt.flush()
	}

	rt.vertices[rt.count] = v
	rt.colors[rt.count] = c
	rt.count += 1
}

func (rt *GLRenderTriangles) flush() {

	if rt.count == 0 {
		return
	}

	gl.UseProgram(rt.programId)

	var proj [16]float32
	g_camera.buildProjectionMatrix(proj[:], 0.2)

	gl.UniformMatrix4fv(rt.projectionUniform, 1, false, &proj[0])

	gl.BindVertexArray(rt.vaoId)

	gl.BindBuffer(gl.ARRAY_BUFFER, rt.vboIds[0])
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, rt.count*int(unsafe.Sizeof(box2d.Vec2{})), unsafe.Pointer(&rt.vertices[0]))

	gl.BindBuffer(gl.ARRAY_BUFFER, rt.vboIds[1])
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, rt.count*int(unsafe.Sizeof(box2d.Color{})), unsafe.Pointer(&rt.colors[0]))

	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
	gl.DrawArrays(gl.TRIANGLES, 0, int32(rt.count))
	gl.Disable(gl.BLEND)

	sCheckGLError()

	gl.BindBuffer(gl.ARRAY_BUFFER, 0)
	gl.BindVertexArray(0)
	gl.UseProgram(0)

	rt.count = 0
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

	dd.points = &GLRenderPoints{}
	dd.points.create()
	dd.lines = &GLRenderLines{}
	dd.lines.create()
	dd.triangles = &GLRenderTriangles{}
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
