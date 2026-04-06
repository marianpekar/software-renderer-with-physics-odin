package main

// ============================================================
// zbuffer.odin
// Depth buffer (Z-buffer) type and clear procedure.
//
// The Z-buffer (depth buffer) is a per-pixel array that stores
// the depth of the closest fragment written so far.  Before writing
// a new pixel, the renderer checks whether its depth value is closer
// than the stored value; if so, the pixel is written and the depth
// entry is updated.  This correctly resolves visibility for any draw
// order without requiring back-to-front sorting of triangles.
//
// Depth representation used here:
//   For PERSPECTIVE projection the stored value is 1/w (reciprocal
//   of clip-space w), which equals 1/viewZ for a standard perspective
//   matrix.  Storing 1/w instead of raw Z enables perspective-correct
//   attribute interpolation: interpolating 1/w linearly across a
//   triangle in screen space is equivalent to interpolating 1/z, which
//   is needed to avoid the "swimming" distortion on textured/shaded
//   quads (see DrawTexelFlatShaded / DrawTexelPhongShaded in draw.odin).
//
//   For ORTHOGRAPHIC projection the stored value is -clip.z (see
//   ProjectToScreen in draw.odin), which increases with distance from
//   the camera and can be compared directly.
//
//   In both cases "smaller stored value = closer to camera", so the
//   clear value is set to a very large number (999_999).
// ============================================================

// ZBuffer is a flat array of one f32 per pixel, laid out in row-major
// order: index = row * SCREEN_WIDTH + column.
ZBuffer :: [SCREEN_WIDTH * SCREEN_HEIGHT]f32

// ClearZBuffer resets all depth entries to the maximum sentinel value
// (999_999), indicating that no geometry has been rendered yet and any
// incoming fragment should be accepted.
// Called once at the beginning of every frame before rendering begins.
ClearZBuffer :: proc(zBuffer: ^ZBuffer) {
    for i in 0..<len(zBuffer) {
        // 999_999 is effectively "infinitely far away" — any real depth value
        // produced by ProjectToScreen will be smaller and will pass the test.
        zBuffer[i] = 999_999;
    }
}
