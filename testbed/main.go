package main

import (
	"fmt"
	"log"
	"os"
	"runtime"
	"time"

	"github.com/cuihonglei/gobox2d/box2d"
	"github.com/go-gl/gl/v3.2-core/gl"
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

func createUI(window *glfw.Window, glslVersion string) {

	imgui.CreateContext(nil)

	var success bool
	success = ImGui_ImplGlfw_InitForOpenGL(window, false)
	if !success {
		fmt.Printf("ImGui_ImplGlfw_InitForOpenGL failed\n")
	}

	success = ImGui_ImplOpenGL3_Init(glslVersion)
	if !success {
		fmt.Printf("ImGui_ImplOpenGL3_Init failed\n")
	}

	// Search for font file
	fontPath1 := "data/droid_sans.ttf"
	fontPath2 := "testbed/data/droid_sans.ttf"
	fontPath := ""
	file1, err1 := os.Open(fontPath1)
	file2, err2 := os.Open(fontPath2)
	if err1 == nil {
		fontPath = fontPath1
		file1.Close()
	}

	if err2 == nil {
		fontPath = fontPath2
		file2.Close()
	}

	if len(fontPath) > 0 {
		imgui.CurrentIO().Fonts().AddFontFromFileTTF(fontPath, 13.0)
	}
}

func resizeWindowCallback(window *glfw.Window, width int, height int) {
	g_camera.width = width
	g_camera.height = height
	s_settings.windowWidth = width
	s_settings.windowHeight = height
}

func keyCallback(window *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {

	ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods)
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

func charCallback(window *glfw.Window, c rune) {
	ImGui_ImplGlfw_CharCallback(window, c)
}

func mouseButtonCallback(window *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {

	ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods)

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
	ImGui_ImplGlfw_ScrollCallback(window, dx, dy)
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
	menuWidth := 180
	if g_debugDraw.showUI {

		imgui.SetNextWindowPos(imgui.Vec2{X: float32(g_camera.width - menuWidth - 10), Y: 10})
		imgui.SetNextWindowSize(imgui.Vec2{X: float32(menuWidth), Y: float32(g_camera.height - 20)})

		imgui.BeginV("Tools", &g_debugDraw.showUI, imgui.WindowFlagsNoMove|imgui.WindowFlagsNoResize|imgui.WindowFlagsNoCollapse)

		if imgui.BeginTabBarV("ControlTabs", imgui.TabBarFlagsNone) {
			if imgui.BeginTabItem("Controls") {

				imgui.SliderInt("Vel Iters", &s_settings.velocityIterations, 0, 50)
				imgui.SliderInt("Pos Iters", &s_settings.positionIterations, 0, 50)
				imgui.SliderFloatV("Hertz", &s_settings.hertz, 5.0, 120.0, "%.0f hz", imgui.SliderFlagsNone)

				imgui.Separator()

				imgui.Checkbox("Sleep", &s_settings.enableSleep)
				imgui.Checkbox("Warm Starting", &s_settings.enableWarmStarting)
				imgui.Checkbox("Time of Impact", &s_settings.enableContinuous)
				imgui.Checkbox("Sub-Stepping", &s_settings.enableSubStepping)

				imgui.Separator()

				imgui.Checkbox("Shapes", &s_settings.drawShapes)
				imgui.Checkbox("Joints", &s_settings.drawJoints)
				imgui.Checkbox("AABBs", &s_settings.drawAABBs)
				imgui.Checkbox("Contact Points", &s_settings.drawContactPoints)
				imgui.Checkbox("Contact Normals", &s_settings.drawContactNormals)
				imgui.Checkbox("Contact Impulses", &s_settings.drawContactImpulse)
				imgui.Checkbox("Friction Impulses", &s_settings.drawFrictionImpulse)
				imgui.Checkbox("Center of Masses", &s_settings.drawCOMs)
				imgui.Checkbox("Statistics", &s_settings.drawStats)
				imgui.Checkbox("Profile", &s_settings.drawProfile)

				button_sz := imgui.Vec2{X: -1, Y: 0}
				if imgui.ButtonV("Pause (P)", button_sz) {
					s_settings.pause = !s_settings.pause
				}

				if imgui.ButtonV("Single Step (O)", button_sz) {
					s_settings.singleStep = !s_settings.singleStep
				}

				if imgui.ButtonV("Restart (R)", button_sz) {
					restartTest()
				}

				if imgui.ButtonV("Quit", button_sz) {
					g_Window.SetShouldClose(true)
				}

				imgui.EndTabItem()
			}

			leafNodeFlags := imgui.TreeNodeFlagsOpenOnArrow | imgui.TreeNodeFlagsOpenOnDoubleClick
			leafNodeFlags |= imgui.TreeNodeFlagsLeaf | imgui.TreeNodeFlagsNoTreePushOnOpen

			nodeFlags := imgui.TreeNodeFlagsOpenOnArrow | imgui.TreeNodeFlagsOpenOnDoubleClick

			if imgui.BeginTabItem("Tests") {

				categoryIndex := 0
				category := g_testEntries[categoryIndex].category
				for i := 0; i < g_testCount; {
					categorySelected := category == g_testEntries[s_settings.testIndex].category
					var nodeSelectionFlags imgui.TreeNodeFlags
					if categorySelected {
						nodeSelectionFlags = imgui.TreeNodeFlagsSelected
					}
					nodeOpen := imgui.TreeNodeV(category, nodeFlags|nodeSelectionFlags)

					if nodeOpen {
						for i < g_testCount && category == g_testEntries[i].category {

							var selectionFlags imgui.TreeNodeFlags
							if s_settings.testIndex == i {
								selectionFlags = imgui.TreeNodeFlagsSelected
							}
							// TODO
							//ImGui::TreeNodeEx((void*)(intptr_t)i, leafNodeFlags | selectionFlags, "%s", g_testEntries[i].name);
							imgui.TreeNodeV(g_testEntries[i].name, leafNodeFlags|selectionFlags)
							if imgui.IsItemClicked() {
								s_test.destroy()
								s_settings.testIndex = i
								s_test = g_testEntries[i].createFcn()
								s_testSelection = i
							}
							i++
						}
						imgui.TreePop()
					} else {
						for i < g_testCount && category == g_testEntries[i].category {
							i++
						}
					}

					if i < g_testCount {
						category = g_testEntries[i].category
						categoryIndex = i
					}
				}

				imgui.EndTabItem()
			}
			imgui.EndTabBar()
		}

		imgui.End()

		s_test.updateUI()
	}
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

	glslVersion := "#version 150"

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
	// TODO glad?
	// int version = gladLoadGL(glfwGetProcAddress);
	// printf("GL %d.%d\n", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));
	// printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

	// TODO Why call this twice?
	//g_mainWindow.SetScrollCallback(scrollCallback)
	g_mainWindow.SetSizeCallback(resizeWindowCallback)
	g_mainWindow.SetKeyCallback(keyCallback)
	g_mainWindow.SetCharCallback(charCallback)
	g_mainWindow.SetMouseButtonCallback(mouseButtonCallback)
	g_mainWindow.SetCursorPosCallback(mouseMotionCallback)
	g_mainWindow.SetScrollCallback(scrollCallback)

	g_debugDraw.create()

	createUI(g_mainWindow, glslVersion)

	s_settings.testIndex = box2d.ClampI(s_settings.testIndex, 0, g_testCount-1)
	s_testSelection = s_settings.testIndex
	s_test = g_testEntries[s_settings.testIndex].createFcn()

	// Control the frame rate. One draw per monitor refresh.
	//glfw.SwapInterval(1)

	gl.ClearColor(0.2, 0.2, 0.2, 1.0)

	var frameTime time.Duration
	var sleepAdjust time.Duration

	for !g_mainWindow.ShouldClose() {

		t1 := time.Now()

		g_camera.width, g_camera.height = g_mainWindow.GetSize()

		bufferWidth, bufferHeight := g_mainWindow.GetFramebufferSize()
		gl.Viewport(0, 0, int32(bufferWidth), int32(bufferHeight))

		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		ImGui_ImplOpenGL3_NewFrame()
		ImGui_ImplGlfw_NewFrame()

		imgui.NewFrame()

		if g_debugDraw.showUI {

			imgui.SetNextWindowPos(imgui.Vec2{})
			imgui.SetNextWindowSize(imgui.Vec2{X: float32(g_camera.width), Y: float32(g_camera.height)})
			imgui.SetNextWindowBgAlpha(0.0)
			imgui.BeginV("Overlay", nil, imgui.WindowFlagsNoTitleBar|imgui.WindowFlagsNoInputs|imgui.WindowFlagsAlwaysAutoResize|imgui.WindowFlagsNoScrollbar)
			imgui.End()

			entry := g_testEntries[s_settings.testIndex]
			buffer = fmt.Sprintf("%s : %s", entry.category, entry.name)
			s_test.drawTitle(buffer)
		}

		s_test.step(&s_settings)

		updateUI()

		// ImGui::ShowDemoWindow();

		if g_debugDraw.showUI {
			buffer = fmt.Sprintf("%.1f ms", 1000.0*frameTime.Seconds())
			g_debugDraw.DrawString(5, g_camera.height-20, buffer)
		}

		imgui.Render()
		ImGui_ImplOpenGL3_RenderDrawData(imgui.RenderedDrawData())

		g_mainWindow.SwapBuffers()

		if s_testSelection != s_settings.testIndex {
			s_settings.testIndex = s_testSelection
			s_test.destroy()
			s_test = g_testEntries[s_settings.testIndex].createFcn()
			g_camera.zoom = 1.0
			g_camera.center.Set(0.0, 20.0)
		}

		glfw.PollEvents()

		// Throttle to cap at 60Hz. This adaptive using a sleep adjustment. This could be improved by
		// using mm_pause or equivalent for the last millisecond.
		t2 := time.Now()
		target := time.Duration(time.Second / 60)
		timeUsed := t2.Sub(t1)
		sleepTime := target - timeUsed + sleepAdjust
		if sleepTime > 0 {
			time.Sleep(sleepTime)
		}

		t3 := time.Now()
		frameTime = t3.Sub(t1)

		// Compute the sleep adjustment using a low pass filter
		sleepAdjust = time.Duration(0.9*float64(sleepAdjust) + 0.9*float64(target-frameTime))
	}

	s_test.destroy()
	s_test = nil

	g_debugDraw.destroy()
	ImGui_ImplOpenGL3_Shutdown()
	ImGui_ImplGlfw_Shutdown()
	//glfw.Terminate()

	s_settings.save()
}
