package main

import (
	"fmt"
	"log"
	"os"
	"runtime"

	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/inkyblackness/imgui-go/v4"
)

var g_mainWindow *glfw.Window = nil
var s_settings = MakeSettings()
var s_testSelection = 0
var s_test ITest = nil
var s_rightMouseDown = false
var s_clickPointWS = box2d.Vec2_zero

func sortTests() {
	// TODO
}

func createUI(window *glfw.Window) {

	imgui.CreateContext(nil)

	// Search for font file
	// fontPath1 := "data/droid_sans.ttf"
	// fontPath2 := "../data/droid_sans.ttf"
	// fontPath := ""
}

func resizeWindowCallback(window *glfw.Window, width int, height int) {
	g_camera.width = width
	g_camera.height = height
	s_settings.windowWidth = width
	s_settings.windowHeight = height
}

func keyCallback(window *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {

	// TODO
	//ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods)
	if imgui.CurrentIO().WantCaptureKeyboard() {
		return
	}

	if action == glfw.Press {
		switch key {
		case glfw.KeyEscape:
			// Quit
			g_mainWindow.SetShouldClose(true)

		case glfw.KeyLeft:
			// Pan left
			if mods == glfw.ModControl {
				newOrigin := box2d.MakeVec2(2.0, 0.0)
				s_test.shiftOrigin(newOrigin)
			} else {
				g_camera.center.X -= 0.5
			}

		case glfw.KeyRight:
			// Pan right
			if mods == glfw.ModControl {
				newOrigin := box2d.MakeVec2(-2.0, 0.0)
				s_test.shiftOrigin(newOrigin)
			} else {
				g_camera.center.X += 0.5
			}

		case glfw.KeyDown:
			// Pan down
			if mods == glfw.ModControl {
				newOrigin := box2d.MakeVec2(0.0, 2.0)
				s_test.shiftOrigin(newOrigin)
			} else {
				g_camera.center.Y -= 0.5
			}

		case glfw.KeyUp:
			// Pan  up
			if mods == glfw.ModControl {
				newOrigin := box2d.MakeVec2(0.0, -2.0)
				s_test.shiftOrigin(newOrigin)
			} else {
				g_camera.center.Y += 0.5
			}

		case glfw.KeyHome:
			// Reset view
			g_camera.zoom = 1.0
			g_camera.center.Set(0.0, 20.0)

		case glfw.KeyZ:
			// Zoom out
			g_camera.zoom = box2d.MinF(1.1*g_camera.zoom, 20.0)

		case glfw.KeyX:
			// Zoom in
			g_camera.zoom = box2d.MaxF(0.9*g_camera.zoom, 0.02)

		case glfw.KeyR:
			// Reset test
			s_test.destroy()
			s_test = g_testEntries[s_settings.testIndex].createFcn()

		case glfw.KeySpace:
			// Launch a bomb.
			if s_test != nil {
				s_test.launchBomb()
			}

		case glfw.KeyO:
			s_settings.singleStep = true

		case glfw.KeyP:
			s_settings.pause = !s_settings.pause

		case glfw.KeyLeftBracket:
			// Switch to previous test
			s_testSelection--
			if s_testSelection < 0 {
				s_testSelection = g_testCount - 1
			}

		case glfw.KeyRightBracket:
			// Switch to next test
			s_testSelection++
			if s_testSelection == g_testCount {
				s_testSelection = 0
			}

		case glfw.KeyTab:
			g_debugDraw.showUI = !g_debugDraw.showUI

		default:
			if s_test != nil {
				s_test.keyboard(key)
			}
		}
	} else if action == glfw.Release {
		s_test.keyboardUp(key)
	}
	// else GLFW_REPEAT
}

func charCallback(window *glfw.Window, char rune) {
	// TODO
}

func mouseButtonCallback(window *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {

	// TODO
	//ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

	xd, yd := g_mainWindow.GetCursorPos()
	ps := box2d.MakeVec2(xd, yd)

	// Use the mouse to move things around.
	if button == glfw.MouseButton1 {

		//
		//ps.Set(0, 0)
		pw := g_camera.convertScreenToWorld(ps)
		if action == glfw.Press {
			if mods == glfw.ModShift {
				s_test.shiftMouseDown(pw)
			} else {
				s_test.mouseDown(pw)
			}
		}

		if action == glfw.Release {
			s_test.mouseUp(pw)
		}

	} else if button == glfw.MouseButton2 {
		if action == glfw.Press {
			s_clickPointWS = g_camera.convertScreenToWorld(ps)
			s_rightMouseDown = true
		}

		if action == glfw.Release {
			s_rightMouseDown = false
		}
	}
}

func mouseMotionCallback(window *glfw.Window, xd float64, yd float64) {

	ps := box2d.MakeVec2(xd, yd)

	pw := g_camera.convertScreenToWorld(ps)
	s_test.mouseMove(pw)

	if s_rightMouseDown {
		diff := box2d.SubVV(pw, s_clickPointWS)
		g_camera.center.X -= diff.X
		g_camera.center.Y -= diff.Y
		s_clickPointWS = g_camera.convertScreenToWorld(ps)
	}
}

func scrollCallback(window *glfw.Window, dx float64, dy float64) {
	// TODO
	//ImGui_ImplGlfw_ScrollCallback(window, dx, dy);
	if imgui.CurrentIO().WantCaptureMouse() {
		return
	}

	if dy > 0 {
		g_camera.zoom /= 1.1
	} else {
		g_camera.zoom *= 1.1
	}
}

func restartTest() {
	s_test.destroy()
	s_test = g_testEntries[s_settings.testIndex].createFcn()
}

func updateUI() {
	// TODO
}

func main() {
	runtime.LockOSThread()

	s_settings.load()
	sortTests()

	g_camera.width = s_settings.windowWidth
	g_camera.height = s_settings.windowHeight

	err := glfw.Init()
	if err != nil {
		fmt.Fprintf(os.Stderr, "Failed to initialize GLFW\n")
		return
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.ContextVersionMajor, 3)
	glfw.WindowHint(glfw.ContextVersionMinor, 3)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)

	buffer := fmt.Sprintf("Box2D Testbed Version %d.%d.%d", box2d.B2_version.Major, box2d.B2_version.Minor, box2d.B2_version.Revision)

	fullscreen := false
	if fullscreen {
		g_mainWindow, err = glfw.CreateWindow(1920, 1080, buffer, glfw.GetPrimaryMonitor(), nil)
	} else {
		g_mainWindow, err = glfw.CreateWindow(g_camera.width, g_camera.height, buffer, nil, nil)
	}
	if err != nil {
		fmt.Fprintf(os.Stderr, "Failed to open GLFW g_mainWindow.\n")
		return
	}

	// TODO Why call this twice?
	//g_mainWindow.SetScrollCallback(scrollCallback)
	g_mainWindow.SetSizeCallback(resizeWindowCallback)
	g_mainWindow.SetKeyCallback(keyCallback)
	g_mainWindow.SetCharCallback(charCallback)
	g_mainWindow.SetMouseButtonCallback(mouseButtonCallback)
	g_mainWindow.SetCursorPosCallback(mouseMotionCallback)
	g_mainWindow.SetScrollCallback(scrollCallback)

	g_mainWindow.MakeContextCurrent()

	// Load OpenGL functions using glad
	err = gl.Init()
	if err != nil {
		log.Fatalf("Error initializing GL: %v", err)
	}

	// set vsync on, enable multisample (if available)
	//glfw.SwapInterval(1)
	gl.Enable(gl.MULTISAMPLE)

	// load GL backend
	//backend, err := goglbackend.New(0, 0, 0, 0, nil)
	//if err != nil {
	//	log.Fatalf("Error loading canvas GL assets: %v", err)
	//}

	g_debugDraw.create()

	createUI(g_mainWindow)

	s_settings.testIndex = box2d.ClampI(s_settings.testIndex, 0, g_testCount-1)
	s_testSelection = s_settings.testIndex
	s_test = g_testEntries[s_settings.testIndex].createFcn()

	gl.ClearColor(0.2, 0.2, 0.2, 1.0)

	// initialize canvas with zero size, since size is set in main loop
	//cv := canvas.New(backend)

	for !g_mainWindow.ShouldClose() {
		g_mainWindow.MakeContextCurrent()

		g_camera.width, g_camera.height = g_mainWindow.GetSize()

		bufferWidth, bufferHeight := g_mainWindow.GetFramebufferSize()
		gl.Viewport(0, 0, int32(bufferWidth), int32(bufferHeight))

		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		//fbw, fbh := g_mainWindow.GetFramebufferSize()
		//backend.SetBounds(0, 0, fbw, fbh)

		s_test.step(&s_settings)

		updateUI()

		g_mainWindow.SwapBuffers()

		if s_testSelection != s_settings.testIndex {
			s_settings.testIndex = s_testSelection
			s_test.destroy()
			s_test = g_testEntries[s_settings.testIndex].createFcn()
			g_camera.zoom = 1.0
			g_camera.center.Set(0.0, 20.0)
		}

		glfw.PollEvents()
	}

	s_test = nil
	g_debugDraw.destroy()
	s_settings.save()
}
