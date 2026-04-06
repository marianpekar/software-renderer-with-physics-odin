package main

// ============================================================
// constants.odin
// Global compile-time constants shared across the entire renderer
// and physics simulation. Centralising them here means changing a
// single number ripples through every system that uses it.
// ============================================================

// --- Rendering ---

// Pixel dimensions of the back-buffer image that is drawn to the screen.
SCREEN_WIDTH  :: 800
SCREEN_HEIGHT :: 600

// Conversion factor from degrees to radians.
// Derived from: π / 180 ≈ 0.01745329251
// Multiply any angle expressed in degrees by this constant to obtain radians,
// which is what the math library trigonometric functions expect.
DEG_TO_RAD :: 0.01745329251

// Vertical field-of-view for the perspective projection, in degrees.
// 70° gives a fairly natural, slightly wide viewing angle.
FOV :: 70

// Near and far clipping-plane distances along the camera's view axis.
// Geometry closer than NEAR_PLANE or farther than FAR_PLANE is clipped.
// The ratio FAR_PLANE / NEAR_PLANE (100) should be kept as small as is
// practical to preserve depth-buffer precision.
NEAR_PLANE :: 1.0
FAR_PLANE  :: 100.0

// --- Physics ---

// Fixed timestep used for the physics integration loop (seconds).
// 1/60 ≈ 0.01667 s keeps the simulation stable and frame-rate-independent.
// The main loop accumulates real elapsed time and steps the simulation in
// discrete chunks of this size (see the physics accumulator in main.odin).
PHYSICS_TIMESTEP :: 1.0 / 60.0

// Constant gravitational acceleration vector applied to all non-static
// rigid bodies when they are not considered grounded.
// Value matches Earth-surface gravity: 9.8 m/s² downward (negative Y).
GRAVITY :: Vector3{0.0, -9.8, 0.0}

// Minimum linear speed (magnitude of velocity) below which a body is
// considered at rest. Prevents micro-movement and floating-point drift from
// indefinitely keeping a body "in motion".
MIN_VELOCITY_THRESHOLD :: 0.1

// Minimum angular speed (magnitude of angular velocity) below which
// rotation is considered stopped. Much smaller than the linear threshold
// because angular velocities decay rapidly due to ANGULAR_DRAG.
MIN_ANGULAR_VELOCITY_THRESHOLD :: 1e-6

// Per-frame velocity damping multipliers applied after force integration.
// Values < 1.0 exponentially decay velocities, simulating air resistance
// or rolling/sliding friction.
// Boxes experience more damping than spheres, which roll more freely.
LINEAR_DRAG       :: f32(0.98)
SPHERE_LINEAR_DRAG :: f32(0.99)
ANGULAR_DRAG       :: f32(0.92)
SPHERE_ANGULAR_DRAG :: f32(0.99)

// Scalar multiplier for the gravitational tipping torque applied to a box
// whose centre of mass has moved past the edge of its support surface.
// Higher values make boxes tip more aggressively when overhanging.
TIPPING_STRENGTH :: 14.0

// Scalar multiplier for the angular correction torque that tries to align
// the "closest-up" axis of a resting box with the surface normal it sits on.
// Keeps boxes from slowly leaning / rotating while stationary.
STABILIZATION_STRENGTH :: 220

// Maximum surface-normal angle (in radians, measured from world-up) at which
// a box is still considered to be on a "flat" surface for the grounded check.
// 0.174532925 rad ≈ 10°, so slopes steeper than 10° cause the box to slide.
SLIDE_ANGLE_THRESHOLD :: 0.174532925

// How far below a box the ground-probe ray is cast each frame when testing
// whether the box is supported. A small positive offset avoids missing the
// surface when the box is exactly flush with it.
GROUND_PROBE_DIST :: 0.05

// The canonical world-space up direction. Used when building the camera's
// orthonormal basis and when computing tipping/stabilization torques.
WORLD_UP :: Vector3{0, 1, 0}

// Number of constraint-solver iterations executed per physics step.
// More iterations produce more accurate stacking / resting-contact behaviour
// but cost more CPU time. 8 is a good balance for this scene size.
SOLVER_ITERATIONS :: 8
