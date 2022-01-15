package main

import (
	"fmt"
	"log"
	"os"
	"runtime"

	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
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
			s_test = g_testEntries[s_settings.testIndex].createFcn()
			g_camera.zoom = 1.0
			g_camera.center.Set(0.0, 0.0)
		}

		glfw.PollEvents()
	}

	s_test = nil
	g_debugDraw.destroy()
	s_settings.save()
}
