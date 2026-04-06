package main

// ============================================================
// light.odin
// Light source data structure and construction helper.
//
// Lights are stored in view space (already transformed by the
// camera's view matrix) so that per-pixel shading code can work
// directly in view space without an extra matrix multiply per pixel.
// ============================================================

// Light represents a single point/directional light source.
//
// Fields:
//   position  — light position expressed in view space.
//               Used by Phong shading to compute the per-fragment
//               light direction vector (light.position - fragPos).
//   direction — normalised direction vector of the light beam, in
//               world space before construction and stored as-is
//               for flat shading (where the direction is uniform
//               per triangle).
//   color     — RGBA colour of the light stored as a Vector4
//               (components in [0,1]).  The .w component is used
//               as an intensity / brightness scalar so a single
//               field controls both colour and strength.
Light :: struct {
    position:  Vector3,
    direction: Vector3,
    color:     Vector4,
}

// MakeLight creates a Light and immediately transforms its position
// into view space using the provided view matrix.
//
// Why transform to view space at construction time?
//   Shading is performed in view space (all vertex positions and
//   normals have already been multiplied through the view matrix by
//   ApplyTransformations in model.odin).  Pre-transforming the light
//   position once here means every per-pixel lighting calculation
//   can simply subtract positions without extra matrix work.
//
// Parameters:
//   position   — world-space position of the light source
//   direction  — world-space illumination direction (will be normalised)
//   color      — RGBA colour/intensity packed as Vector4
//   viewMatrix — the camera's view matrix (world → view transform)
MakeLight :: proc(position, direction: Vector3, color: Vector4, viewMatrix: Matrix4x4) -> Light {
    return {
        // Transform the world-space light position into view space.
        Mat4MulVec3(viewMatrix, position),
        // Normalise the direction vector so it is unit length.
        // A normalised direction is required for correct dot-product
        // (cosine) calculations in the diffuse shading step.
        Vector3Normalize(direction),
        color
    }
}
