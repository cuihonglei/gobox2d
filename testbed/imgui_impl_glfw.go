package main

import (
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/inkyblackness/imgui-go/v4"
)

var g_Window *glfw.Window = nil
var g_Time float64 = 0.0
var g_MouseJustPressed = [5]bool{false, false, false, false, false}
var g_MouseCursors = [imgui.MouseCursorCount]*glfw.Cursor{nil}

// Chain GLFW callbacks: our callbacks will call the user's previously installed callbacks, if any.
var g_PrevUserCallbackMousebutton glfw.MouseButtonCallback = nil
var g_PrevUserCallbackScroll glfw.ScrollCallback = nil
var g_PrevUserCallbackKey glfw.KeyCallback = nil
var g_PrevUserCallbackChar glfw.CharCallback = nil

func ImGui_ImplGlfw_MouseButtonCallback(whidow *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {
	if g_PrevUserCallbackMousebutton != nil {
		g_PrevUserCallbackMousebutton(whidow, button, action, mods)
	}

	if action == glfw.Press && button >= 0 && int(button) < len(g_MouseJustPressed) {
		g_MouseJustPressed[button] = true
	}
}

func ImGui_ImplGlfw_ScrollCallback(whidow *glfw.Window, xoffset float64, yoffset float64) {
	// TODO
}

func ImGui_ImplGlfw_CharCallback(whidow *glfw.Window, c rune) {
	// TODO
}

func ImGui_ImplGlfw_KeyCallback(whidow *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
	// TODO
}

func ImGui_ImplGlfw_InitForOpenGL(window *glfw.Window, install_callbacks bool) bool {
	return ImGui_ImplGlfw_Init(window, install_callbacks)
}

func ImGui_ImplGlfw_Shutdown() {
	// TODO
}

func ImGui_ImplGlfw_UpdateMousePosAndButtons() {

}

func ImGui_ImplGlfw_UpdateMouseCursor() {

}

func ImGui_ImplGlfw_Init(window *glfw.Window, install_callbacks bool) bool {
	g_Window = window
	g_Time = 0.0

	// Setup back-end capabilities flags
	io := imgui.CurrentIO()
	io.SetBackendFlags(imgui.BackendFlagsHasMouseCursors | imgui.BackendFlagsHasSetMousePos)

	// Keyboard mapping. ImGui will use those indices to peek into the io.KeysDown[] array.
	io.KeyMap(imgui.KeyTab, int(glfw.KeyTab))
	io.KeyMap(imgui.KeyLeftArrow, int(glfw.KeyLeft))

	g_MouseCursors[imgui.MouseCursorArrow] = glfw.CreateStandardCursor(glfw.ArrowCursor)
	g_MouseCursors[imgui.MouseCursorTextInput] = glfw.CreateStandardCursor(glfw.IBeamCursor)
	g_MouseCursors[imgui.MouseCursorResizeAll] = glfw.CreateStandardCursor(glfw.ArrowCursor)
	g_MouseCursors[imgui.MouseCursorResizeNS] = glfw.CreateStandardCursor(glfw.VResizeCursor)
	g_MouseCursors[imgui.MouseCursorResizeEW] = glfw.CreateStandardCursor(glfw.HResizeCursor)
	g_MouseCursors[imgui.MouseCursorResizeNESW] = glfw.CreateStandardCursor(glfw.ArrowCursor)
	g_MouseCursors[imgui.MouseCursorResizeNWSE] = glfw.CreateStandardCursor(glfw.ArrowCursor)
	g_MouseCursors[imgui.MouseCursorHand] = glfw.CreateStandardCursor(glfw.HandCursor)

	// Chain GLFW callbacks: our callbacks will call the user's previously installed callbacks, if any.
	g_PrevUserCallbackMousebutton = nil
	g_PrevUserCallbackScroll = nil
	g_PrevUserCallbackKey = nil
	g_PrevUserCallbackChar = nil
	if install_callbacks {
		g_PrevUserCallbackMousebutton = window.SetMouseButtonCallback(ImGui_ImplGlfw_MouseButtonCallback)
		g_PrevUserCallbackScroll = window.SetScrollCallback(ImGui_ImplGlfw_ScrollCallback)
		g_PrevUserCallbackKey = window.SetKeyCallback(ImGui_ImplGlfw_KeyCallback)
		g_PrevUserCallbackChar = window.SetCharCallback(ImGui_ImplGlfw_CharCallback)
	}

	return true
}

func ImGui_ImplGlfw_NewFrame() {
	io := imgui.CurrentIO()

	// Setup display size (every frame to accommodate for window resizing)
	w, h := g_Window.GetSize()
	display_w, display_h := g_Window.GetFramebufferSize()
	io.SetDisplaySize(imgui.Vec2{X: float32(w), Y: float32(h)})
	framebufferScale := imgui.Vec2{}
	if w > 0 {
		framebufferScale.X = float32(display_w) / float32(w)
	}
	if h > 0 {
		framebufferScale.Y = float32(display_h) / float32(h)
	}
	io.SetDisplayFrameBufferScale(framebufferScale)

	// Setup time step
	current_time := glfw.GetTime()
	if g_Time > 0.0 {
		io.SetDeltaTime(float32(current_time - g_Time))
	} else {
		io.SetDeltaTime(1.0 / 60.0)
	}
	g_Time = current_time

	ImGui_ImplGlfw_UpdateMousePosAndButtons()
	ImGui_ImplGlfw_UpdateMouseCursor()

	// TODO
	// Gamepad navigation mapping [BETA]
	//if (io.ConfigFlags & ImGuiConfigFlags_NavEnableGamepad)
}
