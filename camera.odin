package main

// ============================================================
// camera.odin
// Camera data structure and its construction procedure.
//
// The Camera holds a pre-computed orthonormal basis (forward,
// right, up) derived from a position and a look-at target point.
// These basis vectors are used by:
//   • Ray casting (raycast.odin) — to construct view rays
//   • The view matrix (matrix.odin) is built separately from
//     position + target, but the camera basis enables fast per-
//     frame use without rebuilding the full matrix every time.
// ============================================================

// Camera represents a view point in world space together with the
// three mutually perpendicular unit vectors that form its local
// coordinate frame.
//
// Fields:
//   position — world-space eye point (where the camera sits)
//   target   — world-space point the camera is aimed at
//   forward  — unit vector pointing from position toward target;
//              the camera looks along +forward
//   right    — unit vector pointing to the camera's right (local X)
//   up       — unit vector pointing upward relative to the camera
//              (local Y); may differ from world-up if the camera
//              is tilted
Camera :: struct {
    position: Vector3,
    target:   Vector3,
    forward:  Vector3,
    right:    Vector3,
    up:       Vector3
}

// MakeCamera constructs a Camera from a world-space eye position and
// a look-at target point using Gram-Schmidt orthogonalisation.
//
// Algorithm:
//   1. forward = normalise(target - position)
//      The direction the camera faces.
//   2. right = normalise(forward × WORLD_UP)
//      The camera's local X axis, perpendicular to both the forward
//      direction and the world up-vector.  The cross product order
//      (forward × WORLD_UP) gives a right-hand convention where the
//      result points to the right when looking along +forward.
//   3. up = right × forward
//      The camera's local Y axis, recomputed from the already-
//      orthogonalised right and forward vectors so that it is
//      guaranteed to be perpendicular to both (rather than relying
//      on WORLD_UP, which may not be perpendicular to forward when
//      the camera is pitched up/down).
//
// Note: this procedure will produce degenerate results if forward is
// parallel to WORLD_UP (i.e. looking straight up or down) because the
// cross product would yield the zero vector.  For this renderer that
// edge case does not arise given the fixed camera setup in main.odin.
MakeCamera :: proc(position, target: Vector3) -> Camera {
    // Step 1: camera look direction.
    forward := Vector3Normalize(target - position)

    // Step 2: camera right axis — perpendicular to forward and world up.
    right := Vector3Normalize(Vector3CrossProduct(forward, WORLD_UP))

    // Step 3: camera up axis — recomputed for strict perpendicularity.
    return Camera {
        position = position,
        target   = target,
        forward  = forward,
        right    = right,
        up       = Vector3CrossProduct(right, forward)
    }
}
