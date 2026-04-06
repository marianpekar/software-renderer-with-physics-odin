package main

// ============================================================
// vectors.odin
// Basic fixed-size floating-point vector types and the core
// vector-algebra operations used throughout the renderer and
// physics simulation.
//
// All vectors are plain Odin array aliases so they can be
// indexed with [0]/[1]/[2] as well as via .x/.y/.z swizzles.
// ============================================================

import "core:math"

// Vector2 is a 2-component floating-point vector.
// Used primarily for texture UV coordinates and 2-D screen positions.
Vector2 :: [2]f32

// Vector3 is a 3-component floating-point vector.
// The workhorse type: world positions, directions, normals, forces,
// velocities, angular velocities, and colours (R,G,B) all use Vector3.
Vector3 :: [3]f32

// Vector4 is a 4-component floating-point vector.
// Used for homogeneous clip-space coordinates (x,y,z,w) and for light
// colours that carry an alpha/intensity channel in the w component.
Vector4 :: [4]f32

// Vector3Normalize returns the unit-length version of v (i.e. v divided
// by its own magnitude), which is the standard operation for converting
// a direction vector into a normalised direction.
//
// The formula is:  v̂ = v / |v|
//
// A zero-length vector would cause a division by zero, so the function
// guards against that case and returns the zero vector instead.
Vector3Normalize :: proc(v: Vector3) -> Vector3 {
    // Compute the Euclidean (L2) length of the vector.
    length := Vector3Length(v)

    // Guard: if the length is exactly zero there is no well-defined direction,
    // so return the zero vector rather than producing NaN / infinity.
    if length == 0.0 {
        return {0.0, 0.0, 0.0}
    }

    // Divide every component by the length to produce a unit vector.
    // The resulting vector has magnitude 1 and points in the same direction as v.
    return {
        v[0] / length,
        v[1] / length,
        v[2] / length,
    }
}

// Vector3CrossProduct computes the cross product of two 3-D vectors.
// The cross product v1 × v2 yields a third vector that is perpendicular
// to both v1 and v2.  Its direction follows the right-hand rule and its
// magnitude equals |v1| * |v2| * sin(θ), where θ is the angle between them.
//
// Cross products are used here to:
//   • Build orthonormal camera bases (right, up, forward)
//   • Compute face normals for back-face culling and lighting
//   • Derive torque vectors from force and moment-arm: τ = r × F
//
// Component formula:
//   result.x = v1.y * v2.z - v1.z * v2.y
//   result.y = v1.z * v2.x - v1.x * v2.z
//   result.z = v1.x * v2.y - v1.y * v2.x
Vector3CrossProduct :: proc(v1, v2: Vector3) -> Vector3 {
    return {
        v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0],
    }
}

// Vector3DotProduct computes the scalar (dot) product of two 3-D vectors.
// The dot product is defined as:  v1 · v2 = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
//
// Geometric interpretation:  v1 · v2 = |v1| * |v2| * cos(θ)
// so for unit vectors the result is simply the cosine of the angle between them.
//
// Dot products are used here for:
//   • Back-face culling:     comparing face normal with view direction
//   • Diffuse lighting:      N · L gives the Lambertian intensity factor
//   • Projection onto axis:  how far a point lies along a given axis
//   • Overlap tests:         projecting extents during SAT collision detection
Vector3DotProduct :: proc(v1, v2: Vector3) -> f32 {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
}

// FloorXY rounds the X and Y components of the vector down to the nearest
// integer (floor), leaving Z unchanged.
// Used just before rasterising a triangle vertex: pixel coordinates must be
// whole numbers so that the scan-line loop iterates over exact pixel centres.
FloorXY :: proc (v: ^Vector3) {
    v.x = math.floor(v.x)
    v.y = math.floor(v.y)
}

// Vector3Length returns the Euclidean (L2) norm (magnitude) of a 3-D vector.
// The formula is:  |v| = sqrt(v.x² + v.y² + v.z²)
// This is the standard Pythagorean extension into three dimensions.
Vector3Length :: proc(v: Vector3) -> f32 {
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
}
