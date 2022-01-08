package box2d

// Color for debug drawing. Each value has the range [0,1].
type Color struct {
	R, G, B float64
}

func (this *Color) Set(r, g, b float64) {
	this.R, this.G, this.B = r, g, b
}

// Implement and register this class with a b2World to provide debug drawing of physics
// entities in your game.
const (
	Draw_e_shapeBit        = 0x0001 // draw shapes
	Draw_e_jointBit        = 0x0002 // draw joint connections
	Draw_e_aabbBit         = 0x0004 // draw axis aligned bounding boxes
	Draw_e_pairBit         = 0x0008 // draw broad-phase pairs
	Draw_e_centerOfMassBit = 0x0010 // draw center of mass frame
)

type IDraw interface {
	// Set the drawing flags.
	SetFlags(flags uint32)

	// Get the drawing flags.
	GetFlags() uint32

	// Append flags to the current flags.
	AppendFlags(flags uint32)

	// Clear flags from the current flags.
	ClearFlags(flags uint32)

	// Draw a closed polygon provided in CCW order.
	DrawPolygon(vertices []Vec2, color Color)

	// Draw a solid closed polygon provided in CCW order.
	DrawSolidPolygon(vertices []Vec2, color Color)

	// Draw a circle.
	DrawCircle(center Vec2, radius float64, color Color)

	// Draw a solid circle.
	DrawSolidCircle(center Vec2, radius float64, axis Vec2, color Color)

	// Draw a line segment.
	DrawSegment(p1 Vec2, p2 Vec2, color Color)

	// Draw a transform. Choose your own length scale.
	// @param xf a transform.
	DrawTransform(xf Transform)
}

type Draw struct {
	DrawFlags uint32
}

func (this *Draw) SetFlags(flags uint32) {
	this.DrawFlags = flags
}

func (this *Draw) GetFlags() uint32 {
	return this.DrawFlags
}

func (this *Draw) AppendFlags(flags uint32) {
	this.DrawFlags |= flags
}

func (this *Draw) ClearFlags(flags uint32) {
	this.DrawFlags &= ^flags
}
