package main

import (
	"github.com/go-gl/gl/v3.2-core/gl"
	"github.com/inkyblackness/imgui-go/v4"
)

// OpenGL Data
var g_GlslVersionString string = ""
var g_FontTexture uint32 = 0
var g_ShaderHandle, g_VertHandle, g_FragHandle uint32 = 0, 0, 0

func ImGui_ImplOpenGL3_Init(glsl_version string) bool {

	//io := imgui.CurrentIO()
	//io.BackendRendererName = "imgui_impl_opengl3"

	if len(glsl_version) == 0 {
		glsl_version = "#version 130"
	}

	g_GlslVersionString = glsl_version

	return true
}

func ImGui_ImplOpenGL3_Shutdown() {
	//ImGui_ImplOpenGL3_DestroyDeviceObjects();
}

func ImGui_ImplOpenGL3_NewFrame() {
	if g_FontTexture == 0 {
		ImGui_ImplOpenGL3_CreateDeviceObjects()
	}
}

// OpenGL3 Render function.
// (this used to be set in io.RenderDrawListsFn and called by ImGui::Render(), but you can now call this directly from your main loop)
// Note that this implementation is little overcomplicated because we are saving/setting up/restoring every OpenGL state explicitly, in order to be able to run within any OpenGL engine that doesn't do so.
func ImGui_ImplOpenGL3_RenderDrawData(draw_data imgui.DrawData) {

}

func ImGui_ImplOpenGL3_CreateFontsTexture() bool {

	// Build texture atlas
	io := imgui.CurrentIO()
	image := io.Fonts().TextureDataRGBA32() // Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) because it is more likely to be compatible with user's existing shaders. If your ImTextureId represent a higher-level concept than just a GL texture id, consider calling GetTexDataAsAlpha8() instead to save on GPU memory.
	pixels, width, height := image.Pixels, int32(image.Width), int32(image.Height)

	// Upload texture to graphics system
	var last_texture int32
	gl.GetIntegerv(gl.TEXTURE_BINDING_2D, &last_texture)
	gl.GenTextures(1, &g_FontTexture)
	gl.BindTexture(gl.TEXTURE_2D, g_FontTexture)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
	gl.PixelStorei(gl.UNPACK_ROW_LENGTH, 0)
	gl.TexImage2D(gl.TEXTURE_2D, 0, gl.RGBA, width, height, 0, gl.RGBA, gl.UNSIGNED_BYTE, pixels)

	// Store our identifier
	io.Fonts().SetTextureID(imgui.TextureID(g_FontTexture))

	// Restore state
	gl.BindTexture(gl.TEXTURE_2D, uint32(last_texture))

	return true
}

// If you get an error please report on github. You may try different GL context version or GLSL version. See GL<>GLSL version table at the top of this file.
func CheckShader(handle uint32, desc string) bool {

	// TODO

	return true
}

// If you get an error please report on GitHub. You may try different GL context version or GLSL version.
func CheckProgram(handle uint32, desc string) bool {

	// TODO

	return true
}

func ImGui_ImplOpenGL3_CreateDeviceObjects() bool {

	// Backup GL state
	var last_texture, last_array_buffer, last_vertex_array int32
	gl.GetIntegerv(gl.TEXTURE_BINDING_2D, &last_texture)
	gl.GetIntegerv(gl.ARRAY_BUFFER_BINDING, &last_array_buffer)
	gl.GetIntegerv(gl.VERTEX_ARRAY_BINDING, &last_vertex_array)

	const vertex_shader_glsl_130 = `
uniform mat4 ProjMtx;
in vec2 Position;
in vec2 UV;
in vec4 Color;
out vec2 Frag_UV;
out vec4 Frag_Color;
void main()
{
    Frag_UV = UV;
    Frag_Color = Color;
    gl_Position = ProjMtx * vec4(Position.xy,0,1);
}
` + "\x00"

	const fragment_shader_glsl_130 = `
uniform sampler2D Texture;
in vec2 Frag_UV;
in vec4 Frag_Color;
out vec4 Out_Color;
void main()
{
    Out_Color = Frag_Color * texture(Texture, Frag_UV.st);
}
` + "\x00"

	// Select shaders matching our GLSL versions
	vertex_shader := vertex_shader_glsl_130
	//fragment_shader := fragment_shader_glsl_130

	// Create shaders
	vertex_shader_with_version := []string{g_GlslVersionString, vertex_shader}
	g_VertHandle = gl.CreateShader(gl.VERTEX_SHADER)
	csources, free := gl.Strs(vertex_shader_with_version...)
	gl.ShaderSource(g_VertHandle, 2, csources, nil)
	free()
	gl.CompileShader(g_VertHandle)
	CheckShader(g_VertHandle, "vertex shader")

	ImGui_ImplOpenGL3_CreateFontsTexture()

	return true
}
