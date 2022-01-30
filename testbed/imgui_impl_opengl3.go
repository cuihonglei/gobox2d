package main

import (
	"fmt"
	"os"

	"github.com/go-gl/gl/v3.2-core/gl"
	"github.com/inkyblackness/imgui-go/v4"
)

// OpenGL Data
var g_GlslVersionString string = ""
var g_FontTexture uint32 = 0
var g_ShaderHandle, g_VertHandle, g_FragHandle uint32 = 0, 0, 0
var g_AttribLocationTex, g_AttribLocationProjMtx int32 = 0, 0
var g_AttribLocationPosition, g_AttribLocationUV, g_AttribLocationColor int32 = 0, 0, 0
var g_VboHandle, g_ElementsHandle uint32 = 0, 0

func ImGui_ImplOpenGL3_Init(glsl_version string) bool {

	//io := imgui.CurrentIO()
	//io.BackendRendererName = "imgui_impl_opengl3"

	if len(glsl_version) == 0 {
		glsl_version = "#version 130"
	}

	g_GlslVersionString = glsl_version + "\x00"

	return true
}

func ImGui_ImplOpenGL3_Shutdown() {
	ImGui_ImplOpenGL3_DestroyDeviceObjects()
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

	// Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
	io := imgui.CurrentIO()
	display_size := draw_data.DisplaySize()
	display_framebuffer_scale := io.DisplayFrameBufferScale()
	fb_width := int32(display_size.X * display_framebuffer_scale.X)
	fb_height := int32(display_size.Y * display_framebuffer_scale.Y)
	if fb_width <= 0 || fb_height <= 0 {
		return
	}
	draw_data.ScaleClipRects(display_framebuffer_scale)

	// Backup GL state
	var last_active_texture int32
	gl.GetIntegerv(gl.ACTIVE_TEXTURE, &last_active_texture)
	gl.ActiveTexture(gl.TEXTURE0)
	var last_program int32
	gl.GetIntegerv(gl.CURRENT_PROGRAM, &last_program)
	var last_texture int32
	gl.GetIntegerv(gl.TEXTURE_BINDING_2D, &last_texture)

	var last_array_buffer int32
	gl.GetIntegerv(gl.ARRAY_BUFFER_BINDING, &last_array_buffer)
	var last_vertex_array int32
	gl.GetIntegerv(gl.VERTEX_ARRAY_BINDING, &last_vertex_array)

	var last_viewport [4]int32
	gl.GetIntegerv(gl.VIEWPORT, &last_viewport[0])
	var last_scissor_box [4]int32
	gl.GetIntegerv(gl.SCISSOR_BOX, &last_scissor_box[0])
	var last_blend_src_rgb int32
	gl.GetIntegerv(gl.BLEND_SRC_RGB, &last_blend_src_rgb)
	var last_blend_dst_rgb int32
	gl.GetIntegerv(gl.BLEND_DST_RGB, &last_blend_dst_rgb)
	var last_blend_src_alpha int32
	gl.GetIntegerv(gl.BLEND_SRC_ALPHA, &last_blend_src_alpha)
	var last_blend_dst_alpha int32
	gl.GetIntegerv(gl.BLEND_DST_ALPHA, &last_blend_dst_alpha)
	var last_blend_equation_rgb int32
	gl.GetIntegerv(gl.BLEND_EQUATION_RGB, &last_blend_equation_rgb)
	var last_blend_equation_alpha int32
	gl.GetIntegerv(gl.BLEND_EQUATION_ALPHA, &last_blend_equation_alpha)
	last_enable_blend := gl.IsEnabled(gl.BLEND)
	last_enable_cull_face := gl.IsEnabled(gl.CULL_FACE)
	last_enable_depth_test := gl.IsEnabled(gl.DEPTH_TEST)
	last_enable_scissor_test := gl.IsEnabled(gl.SCISSOR_TEST)

	clip_origin_lower_left := true

	// Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, polygon fill
	gl.Enable(gl.BLEND)
	gl.BlendEquation(gl.FUNC_ADD)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
	gl.Disable(gl.CULL_FACE)
	gl.Disable(gl.DEPTH_TEST)
	gl.Enable(gl.SCISSOR_TEST)

	// Setup viewport, orthographic projection matrix
	// Our visible imgui space lies from draw_data->DisplayPos (top left) to draw_data->DisplayPos+data_data->DisplaySize (bottom right). DisplayMin is typically (0,0) for single viewport apps.
	gl.Viewport(0, 0, fb_width, fb_height)
	L := draw_data.DisplayPos().X
	R := draw_data.DisplayPos().X + draw_data.DisplaySize().X
	T := draw_data.DisplayPos().Y
	B := draw_data.DisplayPos().Y + draw_data.DisplaySize().Y
	ortho_projection := []float32{
		2.0 / (R - L), 0.0, 0.0, 0.0,
		0.0, 2.0 / (T - B), 0.0, 0.0,
		0.0, 0.0, -1.0, 0.0,
		(R + L) / (L - R), (T + B) / (B - T), 0.0, 1.0,
	}
	gl.UseProgram(g_ShaderHandle)
	gl.Uniform1i(g_AttribLocationTex, 0)
	gl.UniformMatrix4fv(g_AttribLocationProjMtx, 1, false, &ortho_projection[0])

	// Recreate the VAO every time
	// (This is to easily allow multiple GL contexts. VAO are not shared among GL contexts, and we don't track creation/deletion of windows so we don't have an obvious key to use to cache them.)
	var vao_handle uint32
	gl.GenVertexArrays(1, &vao_handle)
	gl.BindVertexArray(vao_handle)
	gl.BindBuffer(gl.ARRAY_BUFFER, g_VboHandle)
	gl.EnableVertexAttribArray(uint32(g_AttribLocationPosition))
	gl.EnableVertexAttribArray(uint32(g_AttribLocationUV))
	gl.EnableVertexAttribArray(uint32(g_AttribLocationColor))
	vertexSize, vertexOffsetPos, vertexOffsetUv, vertexOffsetCol := imgui.VertexBufferLayout()
	gl.VertexAttribPointerWithOffset(uint32(g_AttribLocationPosition), 2, gl.FLOAT, false, int32(vertexSize), uintptr(vertexOffsetPos))
	gl.VertexAttribPointerWithOffset(uint32(g_AttribLocationUV), 2, gl.FLOAT, false, int32(vertexSize), uintptr(vertexOffsetUv))
	gl.VertexAttribPointerWithOffset(uint32(g_AttribLocationColor), 4, gl.UNSIGNED_BYTE, true, int32(vertexSize), uintptr(vertexOffsetCol))

	// Draw
	pos := draw_data.DisplayPos()
	commandLists := draw_data.CommandLists()
	for n := 0; n < len(commandLists); n++ {
		cmd_list := commandLists[n]
		var idx_buffer_offset uintptr

		vtx_buffer_data, vtx_buffer_size := cmd_list.VertexBuffer()
		gl.BindBuffer(gl.ARRAY_BUFFER, g_VboHandle)
		gl.BufferData(gl.ARRAY_BUFFER, vtx_buffer_size, vtx_buffer_data, gl.STREAM_DRAW)

		idx_buffer_data, idx_buffer_size := cmd_list.IndexBuffer()
		gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, g_ElementsHandle)
		gl.BufferData(gl.ELEMENT_ARRAY_BUFFER, idx_buffer_size, idx_buffer_data, gl.STREAM_DRAW)

		commands := cmd_list.Commands()
		for cmd_i := 0; cmd_i < len(commands); cmd_i++ {
			pcmd := commands[cmd_i]
			if pcmd.HasUserCallback() {
				// User callback (registered via ImDrawList::AddCallback)
				pcmd.CallUserCallback(cmd_list)
			} else {
				clip_rect := imgui.Vec4{X: pcmd.ClipRect().X - pos.X, Y: pcmd.ClipRect().Y - pos.Y, Z: pcmd.ClipRect().Z - pos.X, W: pcmd.ClipRect().W - pos.Y}
				if clip_rect.X < float32(fb_width) && clip_rect.Y < float32(fb_height) && clip_rect.Z >= 0.0 && clip_rect.W >= 0.0 {
					// Apply scissor/clipping rectangle
					if clip_origin_lower_left {
						gl.Scissor(int32(clip_rect.X), fb_height-int32(clip_rect.W), int32(clip_rect.Z-clip_rect.X), int32(clip_rect.W-clip_rect.Y))
					} else {
						gl.Scissor(int32(clip_rect.X), int32(clip_rect.Y), int32(clip_rect.Z), int32(clip_rect.W))
					}

					// Bind texture, Draw
					gl.BindTexture(gl.TEXTURE_2D, uint32(pcmd.TextureID()))
					gl.DrawElementsWithOffset(gl.TRIANGLES, int32(pcmd.ElementCount()), gl.UNSIGNED_SHORT, idx_buffer_offset)
				}
			}
			idx_buffer_offset += uintptr(pcmd.ElementCount() * imgui.IndexBufferLayout())
		}
	}
	gl.DeleteVertexArrays(1, &vao_handle)

	// Restore modified GL state
	gl.UseProgram(uint32(last_program))
	gl.BindTexture(gl.TEXTURE_2D, uint32(last_texture))

	gl.ActiveTexture(uint32(last_active_texture))
	gl.BindVertexArray(uint32(last_vertex_array))
	gl.BindBuffer(gl.ARRAY_BUFFER, uint32(last_array_buffer))
	gl.BlendEquationSeparate(uint32(last_blend_equation_rgb), uint32(last_blend_equation_alpha))
	gl.BlendFuncSeparate(uint32(last_blend_src_rgb), uint32(last_blend_dst_rgb), uint32(last_blend_src_alpha), uint32(last_blend_dst_alpha))
	if last_enable_blend {
		gl.Enable(gl.BLEND)
	} else {
		gl.Disable(gl.BLEND)
	}
	if last_enable_cull_face {
		gl.Enable(gl.CULL_FACE)
	} else {
		gl.Disable(gl.CULL_FACE)
	}
	if last_enable_depth_test {
		gl.Enable(gl.DEPTH_TEST)
	} else {
		gl.Disable(gl.DEPTH_TEST)
	}
	if last_enable_scissor_test {
		gl.Enable(gl.SCISSOR_TEST)
	} else {
		gl.Disable(gl.SCISSOR_BOX)
	}

	gl.Viewport(last_viewport[0], last_viewport[1], last_viewport[2], last_viewport[3])
	gl.Scissor(last_scissor_box[0], last_scissor_box[1], last_scissor_box[2], last_scissor_box[3])
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

func ImGui_ImplOpenGL3_DestroyFontsTexture() {

	if g_FontTexture > 0 {
		io := imgui.CurrentIO()
		gl.DeleteTextures(1, &g_FontTexture)
		io.Fonts().SetTextureID(0)
		g_FontTexture = 0
	}
}

// If you get an error please report on github. You may try different GL context version or GLSL version. See GL<>GLSL version table at the top of this file.
func CheckShader(handle uint32, desc string) bool {

	var status, log_length int32 = 0, 0
	gl.GetShaderiv(handle, gl.COMPILE_STATUS, &status)
	gl.GetShaderiv(handle, gl.INFO_LOG_LENGTH, &log_length)
	if status == gl.FALSE {
		fmt.Fprintf(os.Stderr, "ERROR: ImGui_ImplOpenGL3_CreateDeviceObjects: failed to compile %s!\n", desc)
	}
	if log_length > 0 {
		var buf uint8
		gl.GetShaderInfoLog(handle, log_length, nil, &buf)
		fmt.Fprintf(os.Stderr, "%s\n", gl.GoStr(&buf))
	}

	return status == gl.TRUE
}

// If you get an error please report on GitHub. You may try different GL context version or GLSL version.
func CheckProgram(handle uint32, desc string) bool {

	var status, log_length int32 = 0, 0
	gl.GetProgramiv(handle, gl.LINK_STATUS, &status)
	gl.GetProgramiv(handle, gl.INFO_LOG_LENGTH, &log_length)
	if status == gl.FALSE {
		fmt.Fprintf(os.Stderr, "ERROR: ImGui_ImplOpenGL3_CreateDeviceObjects: failed to link %s! (with GLSL '%s')\n", desc, g_GlslVersionString)
	}
	if log_length > 0 {
		var buf uint8
		gl.GetProgramInfoLog(handle, log_length, nil, &buf)
		fmt.Fprintf(os.Stderr, "%s\n", gl.GoStr(&buf))
	}

	return status == gl.TRUE
}

func ImGui_ImplOpenGL3_CreateDeviceObjects() bool {

	// Backup GL state
	var last_texture, last_array_buffer, last_vertex_array int32
	gl.GetIntegerv(gl.TEXTURE_BINDING_2D, &last_texture)
	gl.GetIntegerv(gl.ARRAY_BUFFER_BINDING, &last_array_buffer)
	gl.GetIntegerv(gl.VERTEX_ARRAY_BINDING, &last_vertex_array)

	// Parse GLSL version string

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
	fragment_shader := fragment_shader_glsl_130

	// Create shaders
	vertex_shader_with_version := []string{g_GlslVersionString, vertex_shader}
	g_VertHandle = gl.CreateShader(gl.VERTEX_SHADER)
	csources, free := gl.Strs(vertex_shader_with_version...)
	gl.ShaderSource(g_VertHandle, 2, csources, nil)
	free()
	gl.CompileShader(g_VertHandle)
	CheckShader(g_VertHandle, "vertex shader")

	fragment_shader_with_version := []string{g_GlslVersionString, fragment_shader}
	g_FragHandle = gl.CreateShader(gl.FRAGMENT_SHADER)
	csources, free = gl.Strs(fragment_shader_with_version...)
	gl.ShaderSource(g_FragHandle, 2, csources, nil)
	free()
	gl.CompileShader(g_FragHandle)
	CheckShader(g_VertHandle, "fragment shader")

	g_ShaderHandle = gl.CreateProgram()
	gl.AttachShader(g_ShaderHandle, g_VertHandle)
	gl.AttachShader(g_ShaderHandle, g_FragHandle)
	gl.LinkProgram(g_ShaderHandle)
	CheckProgram(g_ShaderHandle, "shader program")

	g_AttribLocationTex = gl.GetUniformLocation(g_ShaderHandle, gl.Str("Texture\x00"))
	g_AttribLocationProjMtx = gl.GetUniformLocation(g_ShaderHandle, gl.Str("ProjMtx\x00"))
	g_AttribLocationPosition = gl.GetAttribLocation(g_ShaderHandle, gl.Str("Position\x00"))
	g_AttribLocationUV = gl.GetAttribLocation(g_ShaderHandle, gl.Str("UV\x00"))
	g_AttribLocationColor = gl.GetAttribLocation(g_ShaderHandle, gl.Str("Color\x00"))

	// Create buffers
	gl.GenBuffers(1, &g_VboHandle)
	gl.GenBuffers(1, &g_ElementsHandle)

	ImGui_ImplOpenGL3_CreateFontsTexture()

	// Restore modified GL state
	gl.BindTexture(gl.TEXTURE_2D, uint32(last_texture))
	gl.BindBuffer(gl.ARRAY_BUFFER, uint32(last_array_buffer))
	gl.BindVertexArray(uint32(last_vertex_array))

	return true
}

func ImGui_ImplOpenGL3_DestroyDeviceObjects() {
	if g_VboHandle > 0 {
		gl.DeleteBuffers(1, &g_VboHandle)
		g_VboHandle = 0
	}
	if g_ElementsHandle > 0 {
		gl.DeleteBuffers(1, &g_ElementsHandle)
		g_ElementsHandle = 0
	}

	if g_ShaderHandle > 0 && g_VertHandle > 0 {
		gl.DetachShader(g_ShaderHandle, g_VertHandle)
	}
	if g_VertHandle > 0 {
		gl.DeleteShader(g_VertHandle)
		g_VertHandle = 0
	}

	if g_ShaderHandle > 0 && g_FragHandle > 0 {
		gl.DetachShader(g_ShaderHandle, g_FragHandle)
	}
	if g_FragHandle > 0 {
		gl.DeleteShader(g_FragHandle)
		g_FragHandle = 0
	}

	if g_ShaderHandle > 0 {
		gl.DeleteProgram(g_ShaderHandle)
		g_ShaderHandle = 0
	}

	ImGui_ImplOpenGL3_DestroyFontsTexture()
}
