package main

// ============================================================
// matrix.odin
// 4×4 matrix type and the full set of transformation matrices
// used by the renderer and physics engine.
//
// Convention used throughout:
//   • Row-major storage: mat[row][col]
//   • Vectors are treated as row vectors multiplied on the LEFT,
//     so the translation lives in the rightmost column (mat[row][3]).
//   • Matrix concatenation order: child * parent (right-to-left).
// ============================================================

import "core:math"

// Matrix4x4 is a 4×4 matrix of 32-bit floats stored in row-major order.
// A 4-D (homogeneous) representation is used so that translation, rotation,
// scale, and projection can all be expressed as a single matrix multiply.
Matrix4x4 :: [4][4]f32

// ---------------------------------------------------------------
// Mat4MulVec3
// Multiplies a 4×4 matrix by a 3-component vector, implicitly
// treating the vector as a homogeneous point (w = 1).
//
// The w component is discarded after multiplication, which is
// correct for affine transforms (translation + rotation + scale)
// where w remains 1.  For projective transforms use Mat4MulVec4
// and perform the perspective divide yourself.
// ---------------------------------------------------------------
Mat4MulVec3 :: proc(mat: Matrix4x4, vec: Vector3) -> Vector3 {
    // Each output component is the dot product of the corresponding
    // matrix row with the augmented input vector [vec.x, vec.y, vec.z, 1].
    x := mat[0][0]*vec.x + mat[0][1]*vec.y + mat[0][2]*vec.z + mat[0][3]
    y := mat[1][0]*vec.x + mat[1][1]*vec.y + mat[1][2]*vec.z + mat[1][3]
    z := mat[2][0]*vec.x + mat[2][1]*vec.y + mat[2][2]*vec.z + mat[2][3]

    return Vector3{x, y, z}
}

// ---------------------------------------------------------------
// Mat4MulVec4
// Multiplies a 4×4 matrix by a full 4-component (homogeneous)
// vector.  This is the correct form to use when the w component
// is not 1, such as after a perspective projection where w encodes
// the original depth for the later perspective-divide step.
// ---------------------------------------------------------------
Mat4MulVec4 :: proc(mat: Matrix4x4, vec: Vector4) -> Vector4 {
    x := mat[0][0]*vec.x + mat[0][1]*vec.y + mat[0][2]*vec.z + mat[0][3]*vec.w
    y := mat[1][0]*vec.x + mat[1][1]*vec.y + mat[1][2]*vec.z + mat[1][3]*vec.w
    z := mat[2][0]*vec.x + mat[2][1]*vec.y + mat[2][2]*vec.z + mat[2][3]*vec.w
    w := mat[3][0]*vec.x + mat[3][1]*vec.y + mat[3][2]*vec.z + mat[3][3]*vec.w

    return Vector4{x, y, z, w}
}

// ---------------------------------------------------------------
// Mat4Mul
// Multiplies two 4×4 matrices together: result = a * b.
// Standard O(n³) matrix multiplication (n=4), which performs 64
// multiply-add operations.
//
// The resulting matrix represents the combined transform of first
// applying b, then applying a (right-to-left composition).
// ---------------------------------------------------------------
Mat4Mul :: proc(a, b: Matrix4x4) -> Matrix4x4 {
    result: Matrix4x4
    // Iterate over every output cell (i = row, j = column).
    for i in 0..<4 {
        for j in 0..<4 {
            // Each cell is the dot product of row i of a with column j of b.
            result[i][j] = a[i][0] * b[0][j] +
                           a[i][1] * b[1][j] +
                           a[i][2] * b[2][j] +
                           a[i][3] * b[3][j]
        }
    }
    return result
}

// ---------------------------------------------------------------
// MakeTranslationMatrix
// Builds a pure translation matrix that moves points by (x, y, z).
//
// The homogeneous translation matrix has the form:
//   [ 1  0  0  x ]
//   [ 0  1  0  y ]
//   [ 0  0  1  z ]
//   [ 0  0  0  1 ]
// When multiplied with a homogeneous point [px, py, pz, 1] the
// result is [px+x, py+y, pz+z, 1].
// ---------------------------------------------------------------
MakeTranslationMatrix :: proc(x: f32, y: f32, z: f32) -> Matrix4x4 {
    return Matrix4x4{
        {1.0,  0.0,  0.0,    x},
        {0.0,  1.0,  0.0,    y},
        {0.0,  0.0,  1.0,    z},
        {0.0,  0.0,  0.0,  1.0}
    }
}

// ---------------------------------------------------------------
// MakeScaleMatrix
// Builds a uniform or non-uniform scale matrix.
//
//   [ sx  0   0   0 ]
//   [ 0   sy  0   0 ]
//   [ 0   0   sz  0 ]
//   [ 0   0   0   1 ]
//
// Multiplying a point [px, py, pz, 1] gives [px*sx, py*sy, pz*sz, 1].
// When all three scale factors are equal (sx=sy=sz) the result is a
// uniform (isotropic) scale that preserves shape.
// ---------------------------------------------------------------
MakeScaleMatrix :: proc(sx: f32, sy: f32, sz: f32) -> Matrix4x4 {
    return Matrix4x4{
        {sx,   0.0,  0.0,  0.0},
        {0.0,   sy,  0.0,  0.0},
        {0.0,  0.0,   sz,  0.0},
        {0.0,  0.0,  0.0,  1.0}
    }
}

// ---------------------------------------------------------------
// MakeRotationMatrix
// Builds a 3-D rotation matrix from three Euler angles.
//
// Parameters (in degrees):
//   pitch — rotation around the local X axis (nose up/down)
//   yaw   — rotation around the local Y axis (nose left/right)
//   roll  — rotation around the local Z axis (bank)
//
// The convention used is ZYX intrinsic Euler angles (also called
// Tait–Bryan angles), so the combined rotation is:
//   R = Rz(roll) * Ry(yaw) * Rx(pitch)
// (i.e. pitch is applied first in body space, then yaw, then roll).
//
// Variable naming follows the standard aerospace convention:
//   alpha (α) = yaw    ca/sa = cos/sin of yaw
//   beta  (β) = pitch  cb/sb = cos/sin of pitch
//   gamma (γ) = roll   cg/sg = cos/sin of roll
//
// The resulting 3×3 rotation block (upper-left) is the product
// Rz * Ry * Rx expanded symbolically — see any aeronautics or
// robotics textbook for the full derivation.
// ---------------------------------------------------------------
MakeRotationMatrix :: proc(pitch, yaw, roll: f32) -> Matrix4x4 {
    // Convert input angles from degrees to radians.
    alpha := yaw   * DEG_TO_RAD
    beta  := pitch * DEG_TO_RAD
    gamma := roll  * DEG_TO_RAD

    // Pre-compute cosines and sines to avoid redundant trig calls.
    ca := math.cos(alpha)   // cos(yaw)
    sa := math.sin(alpha)   // sin(yaw)

    cb := math.cos(beta)    // cos(pitch)
    sb := math.sin(beta)    // sin(pitch)

    cg := math.cos(gamma)   // cos(roll)
    sg := math.sin(gamma)   // sin(roll)

    // Assembled ZYX Euler rotation matrix.
    // Each row is the transformed basis vector in world space.
    return Matrix4x4 {
        {ca*cb, ca*sb*sg-sa*cg,  ca*sb*cg+sa*sg, 0.0},
        {sa*cb, sa*sb*sg+ca*cg,  sa*sb*cg-ca*sg, 0.0},
        {  -sb,          cb*sg,  cb*cg,          0.0},
        {  0.0,            0.0,    0.0,          1.0}
    }
}

// ---------------------------------------------------------------
// MakeViewMatrix
// Builds a look-at view matrix that transforms world-space
// positions into camera (view) space.
//
// Algorithm: Gram-Schmidt orthogonalisation
//   1. Compute the forward vector (camera → target direction, then
//      inverted because in view space the camera looks toward -Z).
//   2. Compute the right vector by crossing the world-up (0,1,0)
//      with forward, giving a horizontal vector perpendicular to both.
//   3. Compute the camera's up vector by crossing forward with right,
//      ensuring all three axes are mutually perpendicular.
//   4. Pack the three axes as rows and add the translation via
//      dot products to encode the camera position offset.
//
// The resulting matrix has the form:
//   [ rx  ry  rz  -dot(r,eye) ]
//   [ ux  uy  uz  -dot(u,eye) ]
//   [ fx  fy  fz  -dot(f,eye) ]
//   [  0   0   0           1  ]
// where r/u/f are the right/up/forward unit axes.
// ---------------------------------------------------------------
MakeViewMatrix :: proc(eye: Vector3, target: Vector3) -> Matrix4x4 {
    // Forward: from target toward eye (so the camera looks down -forward in view space).
    forward := Vector3Normalize(eye - target)

    // Right: perpendicular to world-up and forward, giving the camera's local X axis.
    right   := Vector3CrossProduct(Vector3{0.0, 1.0, 0.0}, forward)

    // Up: perpendicular to both forward and right, giving the camera's local Y axis.
    up      := Vector3CrossProduct(forward, right)

    // Build the look-at matrix.  The translation components encode how far the
    // origin is offset along each camera axis when the camera sits at 'eye'.
    return Matrix4x4{
        {   right.x,   right.y,   right.z,  -Vector3DotProduct(right, eye)},
        {      up.x,      up.y,      up.z,  -Vector3DotProduct(up, eye)},
        { forward.x, forward.y, forward.z,  -Vector3DotProduct(forward, eye)},
        {       0.0,       0.0,       0.0,   1.0}
    }
}

// ---------------------------------------------------------------
// MakePerspectiveMatrix
// Builds a right-handed perspective projection matrix that maps
// the view frustum to Normalised Device Coordinates (NDC).
//
// After multiplying a view-space point by this matrix and performing
// the perspective divide (dividing xyz by w), the result is in NDC
// where x and y are in [-1, 1] and z is in (-1, 0] (for this
// convention — near maps to 0, far maps to -1 roughly).
//
// Parameters:
//   fov         — vertical field of view in degrees
//   screenWidth / screenHeight — used to derive the aspect ratio
//   near, far   — near and far clipping distances (positive values)
//
// Key derivation:
//   f = 1 / tan(fov/2)   (focal length for unit-plane)
//   aspect = width / height
//   The (2,2) and (2,3) entries encode the depth remapping.
//   The (3,2) = -1 entry performs the perspective divide by writing
//   the negated view-space Z into the output w.
// ---------------------------------------------------------------
MakePerspectiveMatrix :: proc(fov: f32, screenWidth: i32, screenHeight: i32, near: f32, far: f32) -> Matrix4x4 {
    // f is the cotangent of half the vertical FOV — larger FOV → smaller f → wider view.
    f := 1.0 / math.tan_f32(fov * 0.5 * DEG_TO_RAD)

    // Aspect ratio corrects horizontal scaling so pixels appear square.
    aspect := f32(screenWidth) / f32(screenHeight)

    return Matrix4x4{
        // Column 0: horizontal scale corrected for aspect ratio.
        { f / aspect, 0.0,                        0.0,  0.0},
        // Column 1: vertical scale determined by focal length.
        {        0.0,   f,                        0.0,  0.0},
        // Column 2: depth remapping; column 3: writes -view_z into w (perspective divide).
        {        0.0, 0.0,        -far / (far - near), -1.0},
        {        0.0, 0.0, -far * near / (far - near),  0.0},
    }
}

// ---------------------------------------------------------------
// MakeOrthographicMatrix
// Builds an orthographic (parallel) projection matrix.
// Unlike perspective, this preserves parallel lines — there is no
// perspective divide, so distant objects appear the same size as
// near ones.  Useful for technical / isometric views.
//
// The view volume is an axis-aligned box defined by:
//   left = -aspect,  right = +aspect
//   bottom = -1,     top = +1
//   near, far  (passed in)
//
// The matrix scales and translates that box into NDC [-1,1]³.
// Standard formula from any OpenGL / computer-graphics textbook:
//   scale_x  = 2 / (right - left)
//   scale_y  = 2 / (top - bottom)
//   scale_z  = -2 / (far - near)
//   trans_x  = -(right + left) / (right - left)
//   trans_y  = -(top + bottom) / (top - bottom)
//   trans_z  = -(far + near) / (far - near)
// ---------------------------------------------------------------
MakeOrthographicMatrix :: proc(screenWidth: i32, screenHeight: i32, near: f32, far: f32) -> Matrix4x4 {
    aspect := f32(screenWidth) / f32(screenHeight)
    // Define the symmetric view box in view space.
    left   := -aspect
    right  := +aspect
    bottom :: -1.0
    top    :: +1.0

    return Matrix4x4{
        {2.0 / (right - left),                  0.0,                  0.0,   -(right + left) / (right - left)},
        {                 0.0, 2.0 / (top - bottom),                  0.0,   -(top + bottom) / (top - bottom)},
        {                 0.0,                  0.0,  -2.0 / (far - near),       -(far + near) / (far - near)},
        {                 0.0,                  0.0,                  0.0,                                1.0},
    }
}

// ---------------------------------------------------------------
// MakeRotationMatrixAxisAngle
// Builds a rotation matrix from an arbitrary axis and an angle,
// using the Rodrigues' rotation formula (also known as the
// axis-angle rotation matrix).
//
// Given a unit axis vector a = (x, y, z) and angle θ:
//   R = I·cos(θ) + (1-cos(θ))·(a⊗a) + sin(θ)·[a]×
// where ⊗ is the outer product and [a]× is the skew-symmetric
// cross-product matrix of a.
//
// Expanded into component form:
//   t = 1 - cos(θ)
//   R[0][0] = t*x*x + c        R[0][1] = t*x*y - s*z    R[0][2] = t*x*z + s*y
//   R[1][0] = t*x*y + s*z      R[1][1] = t*y*y + c      R[1][2] = t*y*z - s*x
//   R[2][0] = t*x*z - s*y      R[2][1] = t*y*z + s*x    R[2][2] = t*z*z + c
//
// Used in physics to apply incremental angular-velocity rotations
// each frame and to set initial orientations on models.
// ---------------------------------------------------------------
MakeRotationMatrixAxisAngle :: proc(axis: Vector3, angle: f32) -> Matrix4x4 {
    // Normalise the axis so the formula assumes a unit vector.
    a := Vector3Normalize(axis)
    x := a.x
    y := a.y
    z := a.z

    c := math.cos(angle)   // cos(θ)
    s := math.sin(angle)   // sin(θ)

    // t = 1 - cos(θ), the "versine" factor that weights the outer-product term.
    t := 1.0 - c

    return Matrix4x4{
        {t*x*x + c,   t*x*y - s*z, t*x*z + s*y, 0},
        {t*x*y + s*z, t*y*y + c,   t*y*z - s*x, 0},
        {t*x*z - s*y, t*y*z + s*x, t*z*z + c,   0},
        {0,           0,           0,           1},
    }
}

// ---------------------------------------------------------------
// GetAxesFromRotationMatrix
// Extracts the three local coordinate axes (X, Y, Z) from a
// rotation matrix.  They are stored as the columns of the
// upper-left 3×3 block.
//
// Reading columns out of the row-major array:
//   axis[0] (local X / right)   = column 0 = (mat[0][0], mat[1][0], mat[2][0])
//   axis[1] (local Y / up)      = column 1 = (mat[0][1], mat[1][1], mat[2][1])
//   axis[2] (local Z / forward) = column 2 = (mat[0][2], mat[1][2], mat[2][2])
//
// These axes are used by the OBB collision detection (SAT) and by
// the stabilisation / overhang code in physics.odin.
// ---------------------------------------------------------------
GetAxesFromRotationMatrix :: proc(mat: Matrix4x4) -> [3]Vector3 {
    return {
        // Local X axis (right)
        {mat[0][0], mat[1][0], mat[2][0]},
        // Local Y axis (up)
        {mat[0][1], mat[1][1], mat[2][1]},
        // Local Z axis (forward)
        {mat[0][2], mat[1][2], mat[2][2]},
    }
}

// ---------------------------------------------------------------
// GetUpAxisFromRotationMatrix
// Convenience helper that extracts only the local Y (up) axis
// from a rotation matrix, equivalent to GetAxesFromRotationMatrix(...)[1].
// Used by physics code that only needs the "up" direction of a body
// without needing all three axes.
// ---------------------------------------------------------------
GetUpAxisFromRotationMatrix :: proc(mat: Matrix4x4) -> Vector3 {
    // Column 1 of the rotation matrix is the transformed world-Y axis.
    return {
        mat[0][1], mat[1][1], mat[2][1]
    }
}
