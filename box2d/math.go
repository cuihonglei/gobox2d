package box2d

import (
	"math"
)

// This function is used to ensure that a floating point number is
// not a NaN or infinity.
func IsValid(x float64) bool {
	if x != x {
		// NaN.
		return false
	}
	return -math.MaxFloat64 < x && x < math.MaxFloat64
}

func Sqrt(x float64) float64 {
	return math.Sqrt(x)
}

func Atan2(y float64, x float64) float64 {
	return math.Atan2(y, x)
}

func Sin(x float64) float64 {
	return math.Sin(x)
}

func Cos(x float64) float64 {
	return math.Cos(x)
}

func Floor(x float64) float64 {
	return math.Floor(x)
}

// A 2D column vector.
type Vec2 struct {
	X, Y float64
}

func MakeVec2(x, y float64) Vec2 {
	return Vec2{X: x, Y: y}
}

func NewVec2(x, y float64) *Vec2 {
	return &Vec2{X: x, Y: y}
}

func (v *Vec2) GetI(i int) float64 {
	if i == 0 {
		return v.X
	}
	return v.Y
}

func (v *Vec2) SetI(i int) *float64 {
	if i == 0 {
		return &v.X
	}
	return &v.Y
}

// Set this vector to all zeros.
func (v *Vec2) SetZero() {
	v.X, v.Y = 0.0, 0.0
}

// Set this vector to some specified coordinates.
func (this *Vec2) Set(x float64, y float64) {
	this.X, this.Y = x, y
}

// Add a vector to this vector.
func (this *Vec2) Add(v1 Vec2) {
	this.X += v1.X
	this.Y += v1.Y
}

// Subtract a vector from this vector.
func (this *Vec2) Sub(v1 Vec2) {
	this.X -= v1.X
	this.Y -= v1.Y
}

// Multiply this vector by a scalar.
func (this *Vec2) Mul(a float64) {
	this.X *= a
	this.Y *= a
}

// Get the length of this vector (the norm).
func (this *Vec2) Length() float64 {
	return Sqrt(this.X*this.X + this.Y*this.Y)
}

// Get the length squared. For performance, use this instead of
// Vec2::Length (if possible).
func (this *Vec2) LengthSquared() float64 {
	return this.X*this.X + this.Y*this.Y
}

// Convert this vector into a unit vector. Returns the length.
func (this *Vec2) Normalize() float64 {
	length := this.Length()
	if length < Epsilon {
		return 0.0
	}
	invLength := 1.0 / length
	this.X *= invLength
	this.Y *= invLength
	return length
}

// Does this vector contain finite coordinates?
func (this *Vec2) IsValid() bool {
	return IsValid(this.X) && IsValid(this.Y)
}

// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
func (this *Vec2) Skew() Vec2 {
	return Vec2{-this.Y, this.X}
}

func (this *Vec2) Minus() Vec2 {
	return Vec2{-this.X, -this.Y}
}

// A 2D column vector with 3 elements.
type Vec3 struct {
	X, Y, Z float64
}

// Set this vector to all zeros.
func (this *Vec3) SetZero() {
	this.X, this.Y, this.Z = 0.0, 0.0, 0.0
}

// Set this vector to some specified coordinates.
func (this *Vec3) Set(x, y, z float64) {
	this.X, this.Y, this.Z = x, y, z
}

// Add a vector to this vector.
func (this *Vec3) Add(v1 Vec3) {
	this.X += v1.X
	this.Y += v1.Y
	this.Z += v1.Z
}

// Subtract a vector from this vector.
func (this *Vec3) Sub(v1 Vec3) {
	this.X -= v1.X
	this.Y -= v1.Y
	this.Z -= v1.Z
}

// Multiply this vector by a scalar.
func (this *Vec3) Mul(a float64) {
	this.X *= a
	this.Y *= a
	this.Z *= a
}

func (this *Vec3) Minus() Vec3 {
	return Vec3{-this.X, -this.Y, -this.Z}
}

// A 2-by-2 matrix. Stored in column-major order.
type Mat22 struct {
	Ex, Ey Vec2
}

// Initialize this matrix using columns.
func (this *Mat22) Set(c1, c2 Vec2) {
	this.Ex, this.Ey = c1, c2
}

// Set this to the identity matrix.
func (this *Mat22) SetIdentity() {
	this.Ex.X, this.Ey.X = 1.0, 0.0
	this.Ex.Y, this.Ey.Y = 0.0, 1.0
}

// Set this matrix to all zeros.
func (this *Mat22) SetZero() {
	this.Ex.X, this.Ey.X = 0.0, 0.0
	this.Ex.Y, this.Ey.Y = 0.0, 0.0
}

func (this *Mat22) GetInverse() (B Mat22) {
	a, b, c, d := this.Ex.X, this.Ey.X, this.Ex.Y, this.Ey.Y
	det := a*d - b*c
	if det != 0.0 {
		det = 1.0 / det
	}
	B.Ex.X = det * d
	B.Ey.X = -det * b
	B.Ex.Y = -det * c
	B.Ey.Y = det * a
	return
}

// Solve A * x = b, where b is a column vector. This is more efficient
// than computing the inverse in one-shot cases.
func (this *Mat22) Solve(b Vec2) (x Vec2) {
	a11, a12, a21, a22 := this.Ex.X, this.Ey.X, this.Ex.Y, this.Ey.Y
	det := a11*a22 - a12*a21
	if det != 0.0 {
		det = 1.0 / det
	}
	x.X = det * (a22*b.X - a12*b.Y)
	x.Y = det * (a11*b.Y - a21*b.X)
	return
}

// A 3-by-3 matrix. Stored in column-major order.
type Mat33 struct {
	Ex, Ey, Ez Vec3
}

// Set this matrix to all zeros.
func (this *Mat33) SetZero() {
	this.Ex.SetZero()
	this.Ey.SetZero()
	this.Ez.SetZero()
}

// Solve A * x = b, where b is a column vector. This is more efficient
// than computing the inverse in one-shot cases.
func (this *Mat33) Solve33(b Vec3) (x Vec3) {
	det := DotV3V3(this.Ex, CrossV3V3(this.Ey, this.Ez))
	if det != 0.0 {
		det = 1.0 / det
	}
	x.X = det * DotV3V3(b, CrossV3V3(this.Ey, this.Ez))
	x.Y = det * DotV3V3(this.Ex, CrossV3V3(b, this.Ez))
	x.Z = det * DotV3V3(this.Ex, CrossV3V3(this.Ey, b))
	return
}

// Solve A * x = b, where b is a column vector. This is more efficient
// than computing the inverse in one-shot cases. Solve only the upper
// 2-by-2 matrix equation.
func (this *Mat33) Solve22(b Vec2) (x Vec2) {
	a11, a12, a21, a22 := this.Ex.X, this.Ey.X, this.Ex.Y, this.Ey.Y
	det := a11*a22 - a12*a21
	if det != 0.0 {
		det = 1.0 / det
	}
	x.X = det * (a22*b.X - a12*b.Y)
	x.Y = det * (a11*b.Y - a21*b.X)
	return
}

// Get the inverse of this matrix as a 2-by-2.
// Returns the zero matrix if singular.
func (this *Mat33) GetInverse22(M *Mat33) {
	a, b, c, d := this.Ex.X, this.Ey.X, this.Ex.Y, this.Ey.Y
	det := a*d - b*c
	if det != 0.0 {
		det = 1.0 / det
	}
	M.Ex.X, M.Ey.X, M.Ex.Z = det*d, -det*b, 0.0
	M.Ex.Y, M.Ey.Y, M.Ey.Z = -det*c, det*a, 0.0
	M.Ez.X, M.Ez.Y, M.Ez.Z = 0.0, 0.0, 0.0
}

// Get the symmetric inverse of this matrix as a 3-by-3.
// Returns the zero matrix if singular.
func (this *Mat33) GetSymInverse33(M *Mat33) {
	det := DotV3V3(this.Ex, CrossV3V3(this.Ey, this.Ez))
	if det != 0.0 {
		det = 1.0 / det
	}

	a11, a12, a13 := this.Ex.X, this.Ey.X, this.Ez.X
	a22, a23 := this.Ey.Y, this.Ez.Y
	a33 := this.Ez.Z

	M.Ex.X = det * (a22*a33 - a23*a23)
	M.Ex.Y = det * (a13*a23 - a12*a33)
	M.Ex.Z = det * (a12*a23 - a13*a22)

	M.Ey.X = M.Ex.Y
	M.Ey.Y = det * (a11*a33 - a13*a13)
	M.Ey.Z = det * (a13*a12 - a11*a23)

	M.Ez.X = M.Ex.Z
	M.Ez.Y = M.Ey.Z
	M.Ez.Z = det * (a11*a22 - a12*a12)
}

// Rotation
type Rot struct {
	// Sine and cosine
	S, C float64
}

func NewRot(angle float64) Rot {
	var this Rot
	this.S = Sin(angle)
	this.C = Cos(angle)
	return this
}

// Set using an angle in radians.
func (this *Rot) Set(angle float64) {
	this.S = Sin(angle)
	this.C = Cos(angle)
}

// Set to the identity rotation
func (this *Rot) SetIdentity() {
	this.S = 0.0
	this.C = 1.0
}

// Get the angle in radians
func (this *Rot) GetAngle() float64 {
	return Atan2(this.S, this.C)
}

// Get the x-axis
func (this *Rot) GetXAxis() Vec2 {
	return Vec2{this.C, this.S}
}

// Get the u-axis
func (this *Rot) GetYAxis() Vec2 {
	return Vec2{-this.S, this.C}
}

// A transform contains translation and rotation. It is used to represent
// the position and orientation of rigid frames.
type Transform struct {
	P Vec2
	Q Rot
}

// Set this to the identity transform.
func (this *Transform) SetIdentity() {
	this.P.SetZero()
	this.Q.SetIdentity()
}

// Set this based on the position and angle.
func (this *Transform) Set(position Vec2, angle float64) {
	this.P = position
	this.Q.Set(angle)
}

// This describes the motion of a body/shape for TOI computation.
// Shapes are defined with respect to the body origin, which may
// no coincide with the center of mass. However, to support dynamics
// we must interpolate the center of mass position.
type Sweep struct {
	LocalCenter Vec2    // local center of mass position
	C0, C       Vec2    // center world positions
	A0, A       float64 // world angles

	// Fraction of the current time step in the range [0,1]
	// c0 and a0 are the positions at alpha0.
	Alpha0 float64
}

// Get the interpolated transform at a specific time.
// @param beta is a factor in [0,1], where 0 indicates alpha0.
func (this *Sweep) GetTransform(beta float64) (xf Transform) {
	xf.P = AddVV(MulFV(1.0-beta, this.C0), MulFV(beta, this.C))
	angle := (1.0-beta)*this.A0 + beta*this.A
	xf.Q.Set(angle)

	// Shift to origin
	xf.P.Sub(MulRV(xf.Q, this.LocalCenter))
	return
}

// Advance the sweep forward, yielding a new initial state.
// @param alpha the new initial time.
func (this *Sweep) Advance(alpha float64) {
	beta := (alpha - this.Alpha0) / (1.0 - this.Alpha0)
	this.C0 = AddVV(MulFV(1.0-beta, this.C0), MulFV(beta, this.C))
	this.A0 = (1.0-beta)*this.A0 + beta*this.A
	this.Alpha0 = alpha
}

// Normalize the angles.
func (this *Sweep) Normalize() {
	twoPi := 2.0 * Pi
	d := twoPi * Floor(this.A0/twoPi)
	this.A0 -= d
	this.A -= d
}

// Useful constant
var Vec2_zero Vec2 = Vec2{0.0, 0.0}

// Perform the dot product on two vectors.
func DotVV(a, b Vec2) float64 {
	return a.X*b.X + a.Y*b.Y
}

// Perform the cross product on two vectors. In 2D this produces a scalar.
func CrossVV(a, b Vec2) float64 {
	return a.X*b.Y - a.Y*b.X
}

// Perform the cross product on a vector and a scalar. In 2D this produces
// a vector.
func CrossVF(a Vec2, s float64) Vec2 {
	return Vec2{s * a.Y, -s * a.X}
}

// Perform the cross product on a scalar and a vector. In 2D this produces
// a vector.
func CrossFV(s float64, a Vec2) Vec2 {
	return Vec2{-s * a.Y, s * a.X}
}

// Multiply a matrix times a vector. If a rotation matrix is provided,
// then this transforms the vector from one frame to another.
func MulMV(A Mat22, v Vec2) Vec2 {
	return Vec2{A.Ex.X*v.X + A.Ey.X*v.Y, A.Ex.Y*v.X + A.Ey.Y*v.Y}
}

// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
// then this transforms the vector from one frame to another (inverse transform).
func MulTMV(A Mat22, v Vec2) Vec2 {
	return Vec2{DotVV(v, A.Ex), DotVV(v, A.Ey)}
}

// Add two vectors component-wise.
func AddVV(a, b Vec2) Vec2 {
	return Vec2{a.X + b.X, a.Y + b.Y}
}

// Subtract two vectors component-wise.
func SubVV(a, b Vec2) Vec2 {
	return Vec2{a.X - b.X, a.Y - b.Y}
}

func MulFV(s float64, a Vec2) Vec2 {
	return Vec2{s * a.X, s * a.Y}
}

func DistanceVV(a, b Vec2) float64 {
	c := SubVV(a, b)
	return c.Length()
}

func DistanceSquaredVV(a, b Vec2) float64 {
	c := SubVV(a, b)
	return DotVV(c, c)
}

func MulFV3(s float64, a Vec3) Vec3 {
	return Vec3{s * a.X, s * a.Y, s * a.Z}
}

// Add two vectors component-wise.
func AddV3V3(a, b Vec3) Vec3 {
	return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z}
}

// Subtract two vectors component-wise.
func SubV3V3(a, b Vec3) Vec3 {
	return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z}
}

// Perform the dot product on two vectors.
func DotV3V3(a, b Vec3) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

// Perform the cross product on two vectors.
func CrossV3V3(a, b Vec3) Vec3 {
	return Vec3{a.Y*b.Z - a.Z*b.Y, a.Z*b.X - a.X*b.Z, a.X*b.Y - a.Y*b.X}
}

func AddMM(A, B Mat22) Mat22 {
	return Mat22{AddVV(A.Ex, B.Ex), AddVV(A.Ey, B.Ey)}
}

// A * B
func MulMM(A, B Mat22) Mat22 {
	return Mat22{MulMV(A, B.Ex), MulMV(A, B.Ey)}
}

// A^T * B
func MulTMM(A, B Mat22) Mat22 {
	c1 := Vec2{DotVV(A.Ex, B.Ex), DotVV(A.Ey, B.Ex)}
	c2 := Vec2{DotVV(A.Ex, B.Ey), DotVV(A.Ey, B.Ey)}
	return Mat22{c1, c2}
}

// Multiply a matrix times a vector.
func MulM3V3(A Mat33, v Vec3) Vec3 {
	return AddV3V3(AddV3V3(MulFV3(v.X, A.Ex), MulFV3(v.Y, A.Ey)), MulFV3(v.Z, A.Ez))
}

/// Multiply a matrix times a vector.
func MulM3V(A Mat33, v Vec2) Vec2 {
	return Vec2{A.Ex.X*v.X + A.Ey.X*v.Y, A.Ex.Y*v.X + A.Ey.Y*v.Y}
}

// Multiply two rotations: q * r
func MulRR(q, r Rot) Rot {
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	var qr Rot
	qr.S = q.S*r.C + q.C*r.S
	qr.C = q.C*r.C - q.S*r.S
	return qr
}

// Transpose multiply two rotations: qT * r
func MulTRR(q, r Rot) Rot {
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	var qr Rot
	qr.S = q.C*r.S - q.S*r.C
	qr.C = q.C*r.C + q.S*r.S
	return qr
}

// Rotate a vector
func MulRV(q Rot, v Vec2) Vec2 {
	return Vec2{q.C*v.X - q.S*v.Y, q.S*v.X + q.C*v.Y}
}

// Inverse rotate a vector
func MulTRV(q Rot, v Vec2) Vec2 {
	return Vec2{q.C*v.X + q.S*v.Y, -q.S*v.X + q.C*v.Y}
}

func MulX(T Transform, v Vec2) Vec2 {
	x := (T.Q.C*v.X - T.Q.S*v.Y) + T.P.X
	y := (T.Q.S*v.X + T.Q.C*v.Y) + T.P.Y
	return Vec2{x, y}
}

func MulXT(T Transform, v Vec2) Vec2 {
	px := v.X - T.P.X
	py := v.Y - T.P.Y
	x := (T.Q.C*px + T.Q.S*py)
	y := (-T.Q.S*px + T.Q.C*py)
	return Vec2{x, y}
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
func MulTT(A, B Transform) Transform {
	var C Transform
	C.Q = MulRR(A.Q, B.Q)
	C.P = AddVV(MulRV(A.Q, B.P), A.P)
	return C
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
func MulTTT(A, B Transform) Transform {
	var C Transform
	C.Q = MulTRR(A.Q, B.Q)
	C.P = MulTRV(A.Q, SubVV(B.P, A.P))
	return C
}

func AbsI(x int) int {
	switch {
	case x < 0:
		return -x
	case x == 0:
		return 0 // return correctly abs(-0)
	}
	return x
}

func AbsF(x float64) float64 {
	switch {
	case x < 0:
		return -x
	case x == 0:
		return 0 // return correctly abs(-0)
	}
	return x
}

func AbsV(a Vec2) Vec2 {
	return Vec2{AbsF(a.X), AbsF(a.Y)}
}

func AbsM(A Mat22) Mat22 {
	return Mat22{AbsV(A.Ex), AbsV(A.Ey)}
}

func MinI(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func MinI32(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func MinF(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

func MinV(a, b Vec2) Vec2 {
	return Vec2{MinF(a.X, b.X), MinF(a.Y, b.Y)}
}

func MaxI(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func MaxF(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}

func MaxV(a, b Vec2) Vec2 {
	return Vec2{MaxF(a.X, b.X), MaxF(a.Y, b.Y)}
}

func ClampI(a, low, high int) int {
	return MaxI(low, MinI(a, high))
}

func ClampF(a, low, high float64) float64 {
	return MaxF(low, MinF(a, high))
}

func ClampV(a, low, high Vec2) Vec2 {
	return MaxV(low, MinV(a, high))
}

// "Next Largest Power of 2
// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
// largest power of 2. For a 32-bit value:"
func NextPowerOfTwo(x uint) uint {
	x |= (x >> 1)
	x |= (x >> 2)
	x |= (x >> 4)
	x |= (x >> 8)
	x |= (x >> 16)
	return x + 1
}

func IsPowerOfTwo(x uint) bool {
	return x > 0 && (x&(x-1)) == 0
}
