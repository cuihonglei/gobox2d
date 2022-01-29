package main

import (
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/inkyblackness/imgui-go/v4"
)

var g_Window *glfw.Window = nil
var g_Time float64 = 0.0

func ImGui_ImplGlfw_InitForOpenGL(window *glfw.Window, install_callbacks bool) bool {
	return ImGui_ImplGlfw_Init(window, install_callbacks)
}

func ImGui_ImplGlfw_Shutdown() {
	// TODO
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

	// TODO
	//ImGui_ImplGlfw_UpdateMousePosAndButtons();
	//ImGui_ImplGlfw_UpdateMouseCursor();

	// TODO
	// Gamepad navigation mapping [BETA]
	//if (io.ConfigFlags & ImGuiConfigFlags_NavEnableGamepad)
}
