package box2d

import (
	"fmt"
)

const (
	MaxFloat = 3.402823466e+38
	Epsilon  = 1.192092896e-07
	Pi       = 3.14159265359
)

//
// Global tuning constants based on meters-kilograms-seconds (MKS) units.
//

// Collision

// The maximum number of contact points between two convex shapes. Do
// not change this value.
const MaxManifoldPoints = 2

// The maximum number of vertices on a convex polygon. You cannot increase
// this too much because b2BlockAllocator has a maximum object size.
const MaxPolygonVertices = 8

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment.
// This is in meters.
const AABBExtension = 0.1

// This is used to fatten AABBs in the dynamic tree. This is used to predict
// the future position based on the current displacement.
// This is a dimensionless multiplier.
const AABBMultiplier = 2.0

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant.
const LinearSlop = 0.005

// A small angle used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant.
const AngularSlop = (2.0 / 180.0 * Pi)

// The radius of the polygon/edge shape skin. This should not be modified. Making
// this smaller means polygons will have an insufficient buffer for continuous collision.
// Making it larger may create artifacts for vertex collision.
const PolygonRadius = (2.0 * LinearSlop)

// Maximum number of sub-steps per contact in continuous physics simulation.
const MaxSubSteps = 8

// Dynamics

// Maximum number of contacts to be handled to solve a TOI impact.
const MaxTOIContacts = 32

// A velocity threshold for elastic collisions. Any collision with a relative linear
// velocity below this threshold will be treated as inelastic.
const VelocityThreshold = 1.0

// The maximum linear position correction used when solving constraints. This helps to
// prevent overshoot.
const MaxLinearCorrection = 0.2

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
const MaxAngularCorrection = (8.0 / 180.0 * Pi)

// The maximum linear velocity of a body. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
const MaxTranslation = 2.0
const MaxTranslationSquared = (MaxTranslation * MaxTranslation)

// The maximum angular velocity of a body. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
const MaxRotation = (0.5 * Pi)
const MaxRotationSquared = (MaxRotation * MaxRotation)

// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
// that overlap is removed in one time step. However using values close to 1 often lead
// to overshoot.
const Baumgarte = 0.2
const ToiBaugarte = 0.75

// Sleep

// The time that a body must be still before it will go to sleep.
const TimeToSleep = 0.5

// A body cannot sleep if its linear velocity is above this tolerance.
const LinearSleepTolerance = 0.01

// A body cannot sleep if its angular velocity is above this tolerance.
const AngularSleepTolerance = (2.0 / 180.0 * Pi)

// Logging function.
// You can modify this to use your logging facility.
func Log(format string, a ...interface{}) {
	fmt.Printf(format, a...)
}

// Version numbering scheme.
// See http://en.wikipedia.org/wiki/Software_versioning
type Version struct {
	Major    int // significant changes
	Minor    int // incremental changes
	Revision int // bug fixes
}

// Current version.
var B2_version = Version{2, 2, 1}
