package main

// ============================================================
// draw.odin
// Software rasteriser: all triangle-drawing procedures.
//
// Rendering pipeline overview (per triangle):
//   1. Back-face culling — discard triangles facing away from the camera.
//   2. Projection       — transform view-space vertices to screen space
//                         via ProjectToScreen (perspective or orthographic).
//   3. Frustum culling  — discard triangles entirely outside the viewport.
//   4. Rasterisation    — scan-line fill the triangle pixel by pixel.
//   5. Depth test       — compare against the Z-buffer; write only the
//                         closest fragment.
//   6. Shading / texturing — compute the colour of the fragment.
//
// Supported shading modes (one procedure per mode):
//   DrawWireframe          — edge-only lines (no fill, no depth)
//   DrawUnlit              — flat colour fill, no lighting
//   DrawFlatShaded         — Lambertian flat shading (one colour per face)
//   DrawPhongShaded        — Phong shading (per-pixel diffuse, colour mesh)
//   DrawTexturedUnlit      — perspective-correct texture, no lighting
//   DrawTexturedFlatShaded — perspective-correct texture + flat shading
//   DrawTexturedPhongShaded— perspective-correct texture + Phong shading
//
// Depth representation:
//   Perspective:   stored value = 1/w (reciprocal of clip-space w)
//   Orthographic:  stored value = -clip.z
//   In both cases larger stored value = farther from camera.
//   Fragments pass the depth test when their depth < stored value.
// ============================================================

import rl "vendor:raylib"
import "core:math"

// ---------------------------------------------------------------
// DrawWireframe
// Renders all triangle edges as coloured lines, without filling.
// Optionally culls back-facing triangles before drawing.
//
// Each edge (p1→p2, p2→p3, p3→p1) is drawn as a DDA line.
// No depth-buffer involvement — wireframe is always drawn on top
// or underneath depending on draw order.
// ---------------------------------------------------------------
DrawWireframe :: proc(
    vertices:     []Vector3,
    triangles:    []Triangle,
    projMat:      Matrix4x4,
    projType:     ProjectionType,
    color:        rl.Color,
    cullBackFace: bool,
    image:        ^rl.Image
) {
    for &tri in triangles {
        v1 := vertices[tri[0]]
        v2 := vertices[tri[1]]
        v3 := vertices[tri[2]]

        // Optional back-face culling: skip triangles facing away from camera.
        if cullBackFace && IsBackFace(projType, v1, v2, v3) {
            continue
        }

        // Project the three view-space vertices to 2-D screen coordinates.
        p1 := ProjectToScreen(projType, projMat, v1)
        p2 := ProjectToScreen(projType, projMat, v2)
        p3 := ProjectToScreen(projType, projMat, v3)

        // Skip triangles that lie entirely outside the viewport/frustum.
        if (IsFaceOutsideFrustum(p1, p2, p3)) {
            continue
        }

        // Draw the three edges.
        DrawLine(p1.xy, p2.xy, color, image)
        DrawLine(p2.xy, p3.xy, color, image)
        DrawLine(p3.xy, p1.xy, color, image)
    }
}

// ---------------------------------------------------------------
// IsBackFace
// Returns true if the triangle (v1, v2, v3) faces away from the
// camera and should be culled.
//
// Back-face culling is based on the sign of the dot product between
// the face normal and the view direction:
//   • Compute the face normal via the cross product of two edges:
//       N = (v2 - v1) × (v3 - v1)
//   • For perspective: the view direction is the vector from the
//     triangle to the origin (camera sits at origin in view space),
//     approximated by normalise(v1) (the vector from origin to v1).
//     A back face has N pointing away from the camera, meaning the
//     dot product N·toCamera ≥ 0 (faces the same direction the
//     camera is looking, meaning it is the back side).
//   • For orthographic: the camera direction is always (0, 0, -1)
//     (looking down -Z in view space), so toCamera = (0, 0, -1).
// ---------------------------------------------------------------
IsBackFace :: proc(projType: ProjectionType, v1, v2, v3: Vector3) -> bool {
    // Two edge vectors of the triangle.
    edge1 := v2 - v1
    edge2 := v3 - v1

    // Face normal = edge1 × edge2 (right-hand rule gives outward normal
    // for counter-clockwise winding).
    cross     := Vector3CrossProduct(edge1, edge2)
    crossNorm := Vector3Normalize(cross)

    // View direction: from triangle surface toward the camera.
    toCamera: Vector3
    switch projType {
        case .Perspective:  toCamera = Vector3Normalize(v1)  // origin is at camera
        case .Orthographic: toCamera = Vector3{0, 0, -1}     // parallel rays along -Z
    }

    // If the face normal and the camera direction point in the same general
    // direction (dot ≥ 0), the face is turned away from us — cull it.
    return Vector3DotProduct(crossNorm, toCamera) >= 0.0
}

// ---------------------------------------------------------------
// ProjectToScreen
// Transforms a single view-space point to screen-space coordinates
// via the projection matrix, perspective divide, and NDC-to-pixel mapping.
//
// Steps:
//   1. Multiply the point by the projection matrix → clip space (vec4).
//   2. Perspective divide: divide xyz by w → NDC space (x,y ∈ [-1,+1]).
//   3. Map NDC to pixel coordinates:
//        screenX = (ndcX * 0.5 + 0.5) * SCREEN_WIDTH
//        screenY = (-ndcY * 0.5 + 0.5) * SCREEN_HEIGHT
//      Note the negation of ndcY: NDC Y is +up, but screen Y is +down.
//   4. The Z component of the returned Vector3 stores the depth value
//      used for Z-buffer comparisons:
//        Perspective:   invW = 1/clip.w  (perspective-correct depth)
//        Orthographic:  -clip.z         (linear depth)
// ---------------------------------------------------------------
ProjectToScreen :: proc(projType: ProjectionType, mat: Matrix4x4, p: Vector3) -> Vector3 {
    // Step 1: project into clip space (homogeneous coordinates).
    clip := Mat4MulVec4(mat, Vector4{p.x, p.y, p.z, 1.0})

    // Step 2: perspective divide (1/w for perspective; 1/1=1 for orthographic).
    invW : f32 = 1.0 / clip.w

    // NDC coordinates after divide.
    ndcX := clip.x * invW
    ndcY := clip.y * invW

    // Step 3: remap NDC to pixel coordinates.
    screenX := ( ndcX * 0.5 + 0.5 ) * SCREEN_WIDTH
    screenY := (-ndcY * 0.5 + 0.5 ) * SCREEN_HEIGHT  // flip Y

    // Step 4: pack depth into the Z component.
    switch projType {
        case .Perspective:  return Vector3{screenX, screenY, invW}
        case .Orthographic: return Vector3{screenX, screenY, -clip.z}
    }

    return Vector3{}
}

// ---------------------------------------------------------------
// IsFaceOutsideFrustum
// Returns true if the triangle formed by the three screen-space
// points is entirely outside the visible region (viewport or
// depth range), meaning it can be skipped without drawing.
//
// Two checks:
//   1. Depth range: if all three points lie beyond the near plane
//      (z > 1 or z < -1 depending on convention) the triangle is
//      clipped by the near/far planes.
//   2. Screen bounds: if the triangle's axis-aligned bounding box
//      in screen space doesn't overlap the viewport rectangle
//      [0, SCREEN_WIDTH] × [0, SCREEN_HEIGHT], skip it.
//
// Note: this is a conservative test — it may pass some partially
// off-screen triangles that the pixel-level viewport clamp in
// IsPointOutsideViewport will handle correctly.
// ---------------------------------------------------------------
IsFaceOutsideFrustum :: proc(p1, p2, p3: Vector3) -> bool {
    // Depth clip: reject if all vertices are outside the depth bounds.
    if (p1.z >  1.0 || p2.z >  1.0 || p3.z >  1.0) ||
       (p1.z < -1.0 || p2.z < -1.0 || p3.z < -1.0) {
        return true
    }

    // Screen AABB clip: find the tight bounding box of the triangle.
    minX := math.min(p1.x, math.min(p2.x, p3.x))
    maxX := math.max(p1.x, math.max(p2.x, p3.x))
    minY := math.min(p1.y, math.min(p2.y, p3.y))
    maxY := math.max(p1.y, math.max(p2.y, p3.y))

    // If the AABB lies fully to the left/right/above/below the viewport, skip.
    if maxX < 0 || minX > SCREEN_WIDTH ||
       maxY < 0 || minY > SCREEN_HEIGHT {
        return true
    }

    return false
}

// ---------------------------------------------------------------
// DrawLine
// Draws a straight line between two 2-D screen-space points using
// the DDA (Digital Differential Analyser) algorithm.
//
// DDA algorithm:
//   1. Compute the horizontal (dX) and vertical (dY) deltas.
//   2. Determine the dominant direction: whichever delta is larger
//      in absolute value gives the step count (longerDelta).
//   3. Divide both deltas by the step count to get per-step increments
//      incX = dX/steps,  incY = dY/steps.
//   4. Walk along the line in 'steps' equal increments, rounding
//      the current (x, y) to the nearest pixel at each step.
//
// DDA is simple and fast for rasterising lines without gaps.
// It is equivalent to Bresenham's algorithm but uses floating-point
// arithmetic rather than integer error terms.
// ---------------------------------------------------------------
DrawLine :: proc(a, b: Vector2, color: rl.Color, image: ^rl.Image) {
    dX := b.x - a.x
    dY := b.y - a.y

    // Number of steps = the longer axis extent (ensures no pixel gaps).
    longerDelta := math.abs(dX) >= math.abs(dY) ? math.abs(dX) : math.abs(dY)

    // Per-step increments along each axis.
    incX := dX / longerDelta
    incY := dY / longerDelta

    x := a.x
    y := a.y

    // Walk from a to b, writing one pixel per step.
    for i := 0; i <= int(longerDelta); i += 1 {
        rl.ImageDrawPixel(image, i32(x), i32(y), color)
        x += incX
        y += incY
    }
}

// ---------------------------------------------------------------
// DrawUnlit
// Renders all triangles filled with a flat colour, with no
// lighting calculation.  Uses the Z-buffer for correct depth ordering.
// ---------------------------------------------------------------
DrawUnlit :: proc(
    vertices:  []Vector3,
    triangles: []Triangle,
    projMat:   Matrix4x4,
    projType:  ProjectionType,
    color:     rl.Color,
    zBuffer:   ^ZBuffer,
    image:     ^rl.Image
) {
    for &tri in triangles {
        v1 := vertices[tri[0]]
        v2 := vertices[tri[1]]
        v3 := vertices[tri[2]]

        // Back-face cull.
        if IsBackFace(projType, v1, v2, v3) {
            continue
        }

        p1 := ProjectToScreen(projType, projMat, v1)
        p2 := ProjectToScreen(projType, projMat, v2)
        p3 := ProjectToScreen(projType, projMat, v3)

        if IsFaceOutsideFrustum(p1, p2, p3) {
            continue
        }

        // Fill the triangle with the flat colour.
        DrawFilledTriangle(&p1, &p2, &p3, color, zBuffer, image)
    }
}

// ---------------------------------------------------------------
// DrawFlatShaded
// Renders all triangles with Lambertian flat shading.
//
// Flat shading computes one colour per triangle (one lighting
// calculation per face using the face normal), then fills the
// entire triangle with that colour.
//
// Lambertian (diffuse) reflection model:
//   diffuse = max(0, N · L)
// where N = unit face normal and L = unit vector toward the light.
// This gives the cos(θ) factor of the rendering equation's diffuse term.
//
// Multiple lights are accumulated additively.  Each light's
// contribution is weighted by its colour and alpha (intensity).
// The accumulator is clamped to [0, 1] to avoid overflow.
//
// Ambient light is a constant minimum that prevents faces in
// total shadow from being pitch black.
// ---------------------------------------------------------------
DrawFlatShaded :: proc(
    vertices:  []Vector3,
    triangles: []Triangle,
    projMat:   Matrix4x4,
    projType:  ProjectionType,
    lights:    []Light,
    color:     rl.Color,
    zBuffer:   ^ZBuffer,
    image:     ^rl.Image,
    ambient:   Vector3
) {
    for &tri in triangles {
        v1 := vertices[tri[0]]
        v2 := vertices[tri[1]]
        v3 := vertices[tri[2]]

        // Compute the face normal from the two edge vectors.
        cross     := Vector3CrossProduct(v2 - v1, v3 - v1)
        crossNorm := Vector3Normalize(cross)

        // Determine the view direction for the back-face test.
        toCamera: Vector3
        switch projType {
            case .Perspective:  toCamera = Vector3Normalize(v1)
            case .Orthographic: toCamera = Vector3{0, 0, -1}
        }

        // Back-face cull: skip faces pointing away from the camera.
        if Vector3DotProduct(crossNorm, toCamera) >= 0.0 {
            continue
        }

        p1 := ProjectToScreen(projType, projMat, v1)
        p2 := ProjectToScreen(projType, projMat, v2)
        p3 := ProjectToScreen(projType, projMat, v3)

        if IsFaceOutsideFrustum(p1, p2, p3) {
            continue
        }

        // Accumulate diffuse contributions from all lights, starting
        // from the ambient baseline.
        lightAccum := ambient
        for &light in lights {
            // Lambertian factor: cosine of angle between face normal and light direction.
            // Clamped to 0 so back-lit faces contribute zero, not negative.
            diffuse := math.max(0.0, Vector3DotProduct(crossNorm, light.direction))

            // Add each channel's contribution weighted by light colour and intensity.
            lightAccum.r += diffuse * light.color.r * light.color.a
            lightAccum.g += diffuse * light.color.g * light.color.a
            lightAccum.b += diffuse * light.color.b * light.color.a
        }
        // Clamp each channel to [0, 1] to prevent saturation artefacts.
        lightAccum.r = math.min(lightAccum.r, 1.0)
        lightAccum.g = math.min(lightAccum.g, 1.0)
        lightAccum.b = math.min(lightAccum.b, 1.0)

        // Modulate the model colour by the accumulated light factor.
        shadedColor := rl.Color{
            u8(f32(color.r) * lightAccum.r),
            u8(f32(color.g) * lightAccum.g),
            u8(f32(color.b) * lightAccum.b),
            color.a
        }

        DrawFilledTriangle(&p1, &p2, &p3, shadedColor, zBuffer, image)
    }
}

// ---------------------------------------------------------------
// DrawFilledTriangle
// Rasterises a triangle using the classic scan-line decomposition
// into a flat-bottom triangle and a flat-top triangle.
//
// Algorithm:
//   1. Sort the three vertices by Y so that p1.y ≤ p2.y ≤ p3.y.
//   2. Floor the Y coordinates to align with pixel rows.
//   3. FLAT-BOTTOM HALF (from p1.y to p2.y):
//        Left  slope = (p2.x - p1.x) / (p2.y - p1.y)
//        Right slope = (p3.x - p1.x) / (p3.y - p1.y)
//        Scan from y = p1.y to y = p2.y, computing the left and
//        right X extent of the triangle at each row, then filling
//        every pixel between them.
//   4. FLAT-TOP HALF (from p2.y to p3.y):
//        Left  slope = (p3.x - p2.x) / (p3.y - p2.y)
//        Right slope = (p3.x - p1.x) / (p3.y - p1.y)
//        Scan from y = p2.y to y = p3.y, filling each span.
//
// The decomposition ensures the middle vertex p2 is handled exactly
// once (no pixel double-drawn, no gap at the shared scan line).
// ---------------------------------------------------------------
DrawFilledTriangle :: proc(
    p1, p2, p3: ^Vector3,
    color:      rl.Color,
    zBuffer:    ^ZBuffer,
    image:      ^rl.Image
) {
    // Sort vertices top-to-bottom (ascending Y).
    Sort(p1, p2, p3)

    FloorXY(p1)
    FloorXY(p2)
    FloorXY(p3)

    // --- Flat-bottom half (p1 at top, p2 at middle) ---
    if p2.y != p1.y {
        // Reciprocal slopes: dX / dY — how much X changes per pixel row.
        invSlope1 := (p2.x - p1.x) / (p2.y - p1.y)  // p1 → p2 edge
        invSlope2 := (p3.x - p1.x) / (p3.y - p1.y)  // p1 → p3 edge

        for y := p1.y; y <= p2.y; y += 1 {
            // X extent of the triangle at this scan line.
            xStart := p1.x + (y - p1.y) * invSlope1
            xEnd   := p1.x + (y - p1.y) * invSlope2

            // Ensure left-to-right fill order.
            if xStart > xEnd {
                xStart, xEnd = xEnd, xStart
            }

            for x := xStart; x <= xEnd; x += 1 {
                DrawPixel(x, y, p1, p2, p3, color, zBuffer, image)
            }
        }
    }

    // --- Flat-top half (p2 at middle, p3 at bottom) ---
    if p3.y != p1.y {
        invSlope1 := (p3.x - p2.x) / (p3.y - p2.y)  // p2 → p3 edge
        invSlope2 := (p3.x - p1.x) / (p3.y - p1.y)  // p1 → p3 edge

        for y := p2.y; y <= p3.y; y += 1 {
            xStart := p2.x + (y - p2.y) * invSlope1
            xEnd   := p1.x + (y - p1.y) * invSlope2

            if xStart > xEnd {
                xStart, xEnd = xEnd, xStart
            }

            for x := xStart; x <= xEnd; x += 1 {
                DrawPixel(x, y, p1, p2, p3, color, zBuffer, image)
            }
        }
    }
}

// ---------------------------------------------------------------
// DrawPixel
// Draws a single pixel at screen position (x, y) using barycentric
// interpolation for perspective-correct depth and Z-buffer testing.
//
// Depth interpolation:
//   For perspective-correct interpolation, the depth stored in p*.z
//   is 1/w (the reciprocal of the clip-space w component).  Barycentric
//   coordinates are defined in screen space, but raw Z or UV attributes
//   are NOT linear in screen space under perspective projection — only
//   the reciprocal 1/w (and thus attribute/w) is linear.
//
//   The interpolated 1/w is:
//     denom = α*(1/w1) + β*(1/w2) + γ*(1/w3)
//   The actual depth (proportional to view-space Z) is then:
//     depth = 1 / denom
//   A smaller depth means closer to the camera.
//
//   The pixel is written only if depth < zBuffer[index] (closer).
// ---------------------------------------------------------------
DrawPixel :: proc(
    x, y:       f32,
    p1, p2, p3: ^Vector3,
    color:      rl.Color,
    zBuffer:    ^ZBuffer,
    image:      ^rl.Image
) {
    ix := i32(x)
    iy := i32(y)

    // Discard pixels outside the screen bounds.
    if IsPointOutsideViewport(ix, iy) {
        return
    }

    p       := Vector2{x, y}
    // Compute barycentric weights (α, β, γ) for this pixel relative to the triangle.
    weights := BarycentricWeights(p1.xy, p2.xy, p3.xy, p)
    alpha   := weights.x
    beta    := weights.y
    gamma   := weights.z

    // Perspective-correct depth: interpolate 1/w values, then invert.
    denom := alpha*p1.z + beta*p2.z + gamma*p3.z
    depth := 1.0 / denom

    zIndex := SCREEN_WIDTH*iy + ix

    // Z-buffer test: write only if this fragment is closer.
    if (depth < zBuffer[zIndex]) {
        rl.ImageDrawPixel(image, ix, iy, color)
        zBuffer[zIndex] = depth
    }
}

// ---------------------------------------------------------------
// DrawTexturedUnlit
// Renders all triangles with perspective-correct texture mapping
// and no lighting (texels sampled at full brightness).
// ---------------------------------------------------------------
DrawTexturedUnlit :: proc(
    vertices:  []Vector3,
    triangles: []Triangle,
    uvs:       []Vector2,
    texture:   Texture,
    zBuffer:   ^ZBuffer,
    projMat:   Matrix4x4,
    projType:  ProjectionType,
    image:     ^rl.Image
) {
    for &tri in triangles {
        v1 := vertices[tri[0]]
        v2 := vertices[tri[1]]
        v3 := vertices[tri[2]]

        // UV coordinates for each vertex (indices 3-5 in the Triangle).
        uv1 := uvs[tri[3]]
        uv2 := uvs[tri[4]]
        uv3 := uvs[tri[5]]

        if IsBackFace(projType, v1, v2, v3) {
            continue
        }

        p1 := ProjectToScreen(projType, projMat, v1)
        p2 := ProjectToScreen(projType, projMat, v2)
        p3 := ProjectToScreen(projType, projMat, v3)

        if (IsFaceOutsideFrustum(p1, p2, p3)) {
            continue
        }

        // Pass light = 1.0 (white / full brightness) for the unlit variant.
        DrawTexturedTriangleFlatShaded(
            &p1, &p2, &p3,
            &uv1, &uv2, &uv3,
            texture,
            1.0, // Unlit: no light attenuation
            zBuffer,
            image
        )
    }
}

// ---------------------------------------------------------------
// DrawTexturedFlatShaded
// Renders all triangles with perspective-correct texture mapping
// combined with Lambertian flat shading (one light value per face).
// The shading is computed exactly as in DrawFlatShaded, but the
// resulting light multiplier is applied to each sampled texel rather
// than to a flat colour.
// ---------------------------------------------------------------
DrawTexturedFlatShaded :: proc(
    vertices:  []Vector3,
    triangles: []Triangle,
    uvs:       []Vector2,
    lights:    []Light,
    texture:   Texture,
    zBuffer:   ^ZBuffer,
    projMat:   Matrix4x4,
    projType:  ProjectionType,
    image:     ^rl.Image,
    ambient:   Vector3
) {
    for &tri in triangles {
        v1 := vertices[tri[0]]
        v2 := vertices[tri[1]]
        v3 := vertices[tri[2]]

        uv1 := uvs[tri[3]]
        uv2 := uvs[tri[4]]
        uv3 := uvs[tri[5]]

        // Compute face normal and back-face test.
        cross     := Vector3CrossProduct(v2 - v1, v3 - v1)
        crossNorm := Vector3Normalize(cross)
        toCamera  := Vector3Normalize(v1)

        if (Vector3DotProduct(crossNorm, toCamera) >= 0.0) {
            continue
        }

        p1 := ProjectToScreen(projType, projMat, v1)
        p2 := ProjectToScreen(projType, projMat, v2)
        p3 := ProjectToScreen(projType, projMat, v3)

        if (IsFaceOutsideFrustum(p1, p2, p3)) {
            continue
        }

        // Accumulate diffuse lighting for this face.
        lightAccum := ambient
        for &light in lights {
            diffuse := math.max(0.0, Vector3DotProduct(crossNorm, light.direction))
            lightAccum.r += diffuse * light.color.r * light.color.a
            lightAccum.g += diffuse * light.color.g * light.color.a
            lightAccum.b += diffuse * light.color.b * light.color.a
        }
        lightAccum.r = math.min(lightAccum.r, 1.0)
        lightAccum.g = math.min(lightAccum.g, 1.0)
        lightAccum.b = math.min(lightAccum.b, 1.0)

        DrawTexturedTriangleFlatShaded(
            &p1, &p2, &p3,
            &uv1, &uv2, &uv3,
            texture, lightAccum, zBuffer, image
        )
    }
}

// ---------------------------------------------------------------
// DrawTexturedTriangleFlatShaded
// Rasterises one textured triangle with a flat (face-uniform) light
// multiplier.  The scan-line fill follows the same flat-bottom /
// flat-top decomposition as DrawFilledTriangle.
//
// The 'light' parameter is a Vector3 (R,G,B) multiplier in [0,1]³
// that is applied uniformly to every texel in the triangle.
// ---------------------------------------------------------------
DrawTexturedTriangleFlatShaded :: proc(
    p1, p2, p3:   ^Vector3,
    uv1, uv2, uv3: ^Vector2,
    texture:       Texture,
    light:         Vector3,
    zBuffer:       ^ZBuffer,
    image:         ^rl.Image
) {
    // Sort vertices top-to-bottom; UVs travel with their vertices.
    Sort(p1, p2, p3, uv1, uv2, uv3)

    FloorXY(p1)
    FloorXY(p2)
    FloorXY(p3)

    // --- Flat-bottom half ---
    if p2.y != p1.y {
        invSlope1 := (p2.x - p1.x) / (p2.y - p1.y)
        invSlope2 := (p3.x - p1.x) / (p3.y - p1.y)

        for y := p1.y; y <= p2.y; y += 1 {
            xStart := p1.x + (y - p1.y) * invSlope1
            xEnd   := p1.x + (y - p1.y) * invSlope2

            if xStart > xEnd {
                xStart, xEnd = xEnd, xStart
            }

            for x := xStart; x <= xEnd; x += 1 {
                DrawTexelFlatShaded(
                    x, y,
                    p1, p2, p3,
                    uv1, uv2, uv3,
                    texture, light, zBuffer, image
                )
            }
        }
    }

    // --- Flat-top half ---
    if p3.y != p1.y {
        invSlope1 := (p3.x - p2.x) / (p3.y - p2.y)
        invSlope2 := (p3.x - p1.x) / (p3.y - p1.y)

        for y := p2.y; y <= p3.y; y += 1 {
            xStart := p2.x + (y - p2.y) * invSlope1
            xEnd   := p1.x + (y - p1.y) * invSlope2

            if xStart > xEnd {
                xStart, xEnd = xEnd, xStart
            }

            for x := xStart; x <= xEnd; x += 1 {
                DrawTexelFlatShaded(
                    x, y,
                    p1, p2, p3,
                    uv1, uv2, uv3,
                    texture, light, zBuffer, image
                )
            }
        }
    }
}

// ---------------------------------------------------------------
// DrawTexelFlatShaded
// Draws a single textured pixel with a flat (uniform) light multiplier.
//
// Perspective-correct UV interpolation:
//   The key insight is that 1/w, u/w, and v/w are all linear in
//   screen space (they are linear functions of the clip-space values).
//   Therefore, instead of interpolating u and v directly (which would
//   cause perspective distortion), we interpolate u/w and v/w and then
//   divide by the interpolated 1/w to recover the correct u and v:
//
//     interpolated_u = (α*(u1/w1) + β*(u2/w2) + γ*(u3/w3)) / (α/w1 + β/w2 + γ/w3)
//                    = (α*u1*p1.z + β*u2*p2.z + γ*u3*p3.z) * depth
//
//   where p*.z = 1/w (stored by ProjectToScreen) and depth = 1 / (α*p1.z + β*p2.z + γ*p3.z).
//
// Texture sampling:
//   The recovered (u, v) in [0,1]² is multiplied by the texture
//   dimensions and the result is taken modulo the dimensions to wrap
//   the texture (repeat mode).
// ---------------------------------------------------------------
DrawTexelFlatShaded :: proc(
    x, y:          f32,
    p1, p2, p3:    ^Vector3,
    uv1, uv2, uv3: ^Vector2,
    texture:       Texture,
    light:         Vector3,
    zBuffer:       ^ZBuffer,
    image:         ^rl.Image
) {
    ix := i32(x)
    iy := i32(y)
    if IsPointOutsideViewport(ix, iy) {
        return
    }

    p       := Vector2{x, y}
    weights := BarycentricWeights(p1.xy, p2.xy, p3.xy, p)
    alpha   := weights.x
    beta    := weights.y
    gamma   := weights.z

    // Perspective-correct depth denominator (sum of barycentric-weighted 1/w values).
    denom := alpha*p1.z + beta*p2.z + gamma*p3.z
    depth := 1.0 / denom

    zIndex := SCREEN_WIDTH*iy + ix

    // Z-buffer test: only draw if this fragment is closer than the stored depth.
    if depth <= zBuffer[zIndex] {
        // Perspective-correct UV interpolation.
        // Multiply each UV component by 1/w before interpolating, then multiply
        // the sum by the recovered depth (= 1/(sum of 1/w)) to undo the w division.
        interpU := ((uv1.x*p1.z)*alpha + (uv2.x*p2.z)*beta + (uv3.x*p3.z)*gamma) * depth
        interpV := ((uv1.y*p1.z)*alpha + (uv2.y*p2.z)*beta + (uv3.y*p3.z)*gamma) * depth

        // Convert UVs to texel coordinates with wrapping (modulo for repeat).
        texX := i32(interpU * f32(texture.width )) % texture.width
        texY := i32(interpV * f32(texture.height)) % texture.height

        // Sample the texel from the CPU pixel buffer.
        tex := texture.pixels[texY*texture.width + texX]

        // Apply the flat light multiplier to the sampled texel colour.
        shadedTex := rl.Color{
            u8(f32(tex.r) * light.r),
            u8(f32(tex.g) * light.g),
            u8(f32(tex.b) * light.b),
            tex.a,
        }

        rl.ImageDrawPixel(image, ix, iy, shadedTex)
        zBuffer[zIndex] = depth
    }
}

// ---------------------------------------------------------------
// DrawPhongShaded
// Renders all triangles with per-pixel Phong (diffuse) shading
// using interpolated vertex normals.
//
// Unlike flat shading (which uses one normal per face), Phong shading
// interpolates the per-vertex normals across the triangle using
// perspective-correct barycentric interpolation, then computes the
// lighting per pixel.  This produces smooth shading even on coarsely
// tessellated meshes.
//
// Each triangle's three vertex normals (tri[6], tri[7], tri[8]) are
// looked up from the view-space transformed normals array.
// ---------------------------------------------------------------
DrawPhongShaded :: proc(
    vertices:  []Vector3,
    triangles: []Triangle,
    normals:   []Vector3,
    lights:    []Light,
    color:     rl.Color,
    zBuffer:   ^ZBuffer,
    projMat:   Matrix4x4,
    projType:  ProjectionType,
    image:     ^rl.Image,
    ambient:   Vector3
) {
    for &tri in triangles {
        v1 := vertices[tri[0]]
        v2 := vertices[tri[1]]
        v3 := vertices[tri[2]]

        // Vertex normals (indices 6-8 in the Triangle).
        n1 := normals[tri[6]]
        n2 := normals[tri[7]]
        n3 := normals[tri[8]]

        if IsBackFace(projType, v1, v2, v3) {
            continue
        }

        p1 := ProjectToScreen(projType, projMat, v1)
        p2 := ProjectToScreen(projType, projMat, v2)
        p3 := ProjectToScreen(projType, projMat, v3)

        if IsFaceOutsideFrustum(p1, p2, p3) {
            continue
        }

        DrawTrianglePhongShaded(
            &v1, &v2, &v3,
            &p1, &p2, &p3,
            &n1, &n2, &n3,
            color, lights, zBuffer, image, ambient
        )
    }
}

// ---------------------------------------------------------------
// DrawTrianglePhongShaded
// Rasterises one triangle with Phong shading by decomposing it
// into flat-bottom and flat-top halves (same scan-line approach
// as DrawFilledTriangle), calling DrawPixelPhongShaded per pixel.
// The normals are sorted in sync with the screen-space points.
// ---------------------------------------------------------------
DrawTrianglePhongShaded :: proc(
    v1, v2, v3: ^Vector3,
    p1, p2, p3: ^Vector3,
    n1, n2, n3: ^Vector3,
    color:      rl.Color,
    lights:     []Light,
    zBuffer:    ^ZBuffer,
    image:      ^rl.Image,
    ambient:    Vector3
) {
    // Sort screen-space points by Y; view-space positions travel with them.
    Sort(p1, p2, p3, v1, v2, v3)

    FloorXY(p1)
    FloorXY(p2)
    FloorXY(p3)

    // --- Flat-bottom half ---
    if p2.y != p1.y {
        invSlope1 := (p2.x - p1.x) / (p2.y - p1.y)
        invSlope2 := (p3.x - p1.x) / (p3.y - p1.y)

        for y := p1.y; y <= p2.y; y += 1 {
            xStart := p1.x + (y - p1.y) * invSlope1
            xEnd   := p1.x + (y - p1.y) * invSlope2

            if xStart > xEnd {
                xStart, xEnd = xEnd, xStart
            }

            for x := xStart; x <= xEnd; x += 1 {
                DrawPixelPhongShaded(
                    x, y,
                    v1, v2, v3,
                    n1, n2, n3,
                    p1, p2, p3,
                    color, lights, zBuffer, image, ambient
                )
            }
        }
    }

    // --- Flat-top half ---
    if p3.y != p1.y {
        invSlope1 := (p3.x - p2.x) / (p3.y - p2.y)
        invSlope2 := (p3.x - p1.x) / (p3.y - p1.y)

        for y := p2.y; y <= p3.y; y += 1 {
            xStart := p2.x + (y - p2.y) * invSlope1
            xEnd   := p1.x + (y - p1.y) * invSlope2

            if xStart > xEnd {
                xStart, xEnd = xEnd, xStart
            }

            for x := xStart; x <= xEnd; x += 1 {
                DrawPixelPhongShaded(
                    x, y,
                    v1, v2, v3,
                    n1, n2, n3,
                    p1, p2, p3,
                    color, lights, zBuffer, image, ambient
                )
            }
        }
    }
}

// ---------------------------------------------------------------
// DrawPixelPhongShaded
// Draws a single pixel with full per-pixel Phong shading.
//
// Steps:
//   1. Compute barycentric weights (α, β, γ).
//   2. Compute perspective-correct depth (1/denom).
//   3. Z-buffer test.
//   4. Perspective-correct interpolation of the vertex normal:
//        N_interp = normalise(α*n1 + β*n2 + γ*n3)
//      (Exact perspective-correct normal interpolation would use
//       the 1/w trick; here the normals are interpolated in screen
//       space and renormalised, which is a common approximation.)
//   5. Perspective-correct interpolation of the view-space position:
//        P_interp = (α*(v1/w1) + β*(v2/w2) + γ*(v3/w3)) * depth
//      Used to compute the light direction per fragment.
//   6. For each light: compute the unit light vector (light.position - P_interp)
//      and evaluate the Lambertian diffuse term N_interp · L.
//   7. Accumulate light contributions and clamp to [0, 1].
//   8. Modulate model colour by accumulated light.
// ---------------------------------------------------------------
DrawPixelPhongShaded :: proc(
    x, y:       f32,
    v1, v2, v3: ^Vector3,
    n1, n2, n3: ^Vector3,
    p1, p2, p3: ^Vector3,
    color:      rl.Color,
    lights:     []Light,
    zBuffer:    ^ZBuffer,
    image:      ^rl.Image,
    ambient:    Vector3
) {
    ix := i32(x)
    iy := i32(y)
    if IsPointOutsideViewport(ix, iy) {
        return
    }

    p       := Vector2{x, y}
    weights := BarycentricWeights(p1.xy, p2.xy, p3.xy, p)
    alpha   := weights.x
    beta    := weights.y
    gamma   := weights.z

    // Perspective-correct depth.
    denom := alpha*p1.z + beta*p2.z + gamma*p3.z
    depth := 1.0 / denom

    zIndex := SCREEN_WIDTH*iy + ix
    if depth <= zBuffer[zIndex] {
        // Interpolate and normalise the vertex normal.
        interpNormal := Vector3Normalize(n1^ * alpha + n2^ * beta + n3^ * gamma)

        // Perspective-correct fragment position in view space.
        // Each position component is weighted by (attribute * 1/w), then the
        // sum is multiplied by depth = w to recover the actual interpolated position.
        interpPos := ((v1^*p1.z) * alpha + (v2^*p2.z) * beta + (v3^*p3.z) * gamma) * depth

        // Accumulate diffuse lighting per light source.
        lightAccum := ambient
        for &light in lights {
            // Per-fragment light direction: from fragment to light position.
            lightVec := Vector3Normalize(light.position - interpPos)
            // Lambertian diffuse: max(0, N · L)
            diffuse  := math.max(Vector3DotProduct(interpNormal, lightVec), 0.0)
            lightAccum.r += diffuse * light.color.r * light.color.a
            lightAccum.g += diffuse * light.color.g * light.color.a
            lightAccum.b += diffuse * light.color.b * light.color.a
        }
        lightAccum.r = math.min(lightAccum.r, 1.0)
        lightAccum.g = math.min(lightAccum.g, 1.0)
        lightAccum.b = math.min(lightAccum.b, 1.0)

        // Apply light to model colour.
        shadedColor := rl.Color{
            u8(f32(color.r) * lightAccum.r),
            u8(f32(color.g) * lightAccum.g),
            u8(f32(color.b) * lightAccum.b),
            color.a,
        }

        rl.ImageDrawPixel(image, ix, iy, shadedColor)
        zBuffer[zIndex] = depth
    }
}

// ---------------------------------------------------------------
// DrawTexturedPhongShaded
// Renders all triangles with perspective-correct texture sampling
// combined with per-pixel Phong shading.  All three vertex attribute
// sets (positions, UVs, normals) are combined in the per-pixel step.
// ---------------------------------------------------------------
DrawTexturedPhongShaded :: proc(
    vertices:  []Vector3,
    triangles: []Triangle,
    uvs:       []Vector2,
    normals:   []Vector3,
    lights:    []Light,
    texture:   Texture,
    zBuffer:   ^ZBuffer,
    projMat:   Matrix4x4,
    projType:  ProjectionType,
    image:     ^rl.Image,
    ambient:   Vector3
) {
    for &tri in triangles {
        v1 := vertices[tri[0]]
        v2 := vertices[tri[1]]
        v3 := vertices[tri[2]]

        uv1 := uvs[tri[3]]
        uv2 := uvs[tri[4]]
        uv3 := uvs[tri[5]]

        n1 := normals[tri[6]]
        n2 := normals[tri[7]]
        n3 := normals[tri[8]]

        if IsBackFace(projType, v1, v2, v3) {
            continue
        }

        p1 := ProjectToScreen(projType, projMat, v1)
        p2 := ProjectToScreen(projType, projMat, v2)
        p3 := ProjectToScreen(projType, projMat, v3)

        if IsFaceOutsideFrustum(p1, p2, p3) {
            continue
        }

        DrawTexturedTrianglePhongShaded(
            &v1, &v2, &v3,
            &p1, &p2, &p3,
            &uv1, &uv2, &uv3,
            &n1, &n2, &n3,
            texture, lights, zBuffer, image, ambient
        )
    }
}

// ---------------------------------------------------------------
// DrawTexturedTrianglePhongShaded
// Rasterises one fully-featured triangle: textured + Phong shading.
// All three attribute sets (screen points, UVs, view-space positions,
// and normals) are sorted in sync so each vertex's data stays together.
// ---------------------------------------------------------------
DrawTexturedTrianglePhongShaded :: proc(
    v1, v2, v3:    ^Vector3,
    p1, p2, p3:    ^Vector3,
    uv1, uv2, uv3: ^Vector2,
    n1, n2, n3:    ^Vector3,
    texture:       Texture,
    lights:        []Light,
    zBuffer:       ^ZBuffer,
    image:         ^rl.Image,
    ambient:       Vector3
) {
    // Sort all attribute groups by screen-space Y together.
    Sort(p1, p2, p3, uv1, uv2, uv3, v1, v2, v3)
    // Note: normals n1/n2/n3 are NOT explicitly sorted here because
    // they are looked up by the same vertex indices as positions, so
    // after the position sort each n* still corresponds to the right v*.

    FloorXY(p1)
    FloorXY(p2)
    FloorXY(p3)

    // --- Flat-bottom half ---
    if p2.y != p1.y {
        invSlope1 := (p2.x - p1.x) / (p2.y - p1.y)
        invSlope2 := (p3.x - p1.x) / (p3.y - p1.y)

        for y := p1.y; y <= p2.y; y += 1 {
            xStart := p1.x + (y - p1.y) * invSlope1
            xEnd   := p1.x + (y - p1.y) * invSlope2

            if xStart > xEnd {
                xStart, xEnd = xEnd, xStart
            }

            for x := xStart; x <= xEnd; x += 1 {
                DrawTexelPhongShaded(
                    x, y,
                    v1, v2, v3,
                    n1, n2, n3,
                    p1, p2, p3,
                    uv1, uv2, uv3,
                    texture, lights, zBuffer, image, ambient
                )
            }
        }
    }

    // --- Flat-top half ---
    if p3.y != p1.y {
        invSlope1 := (p3.x - p2.x) / (p3.y - p2.y)
        invSlope2 := (p3.x - p1.x) / (p3.y - p1.y)

        for y := p2.y; y <= p3.y; y += 1 {
            xStart := p2.x + (y - p2.y) * invSlope1
            xEnd   := p1.x + (y - p1.y) * invSlope2

            if xStart > xEnd {
                xStart, xEnd = xEnd, xStart
            }

            for x := xStart; x <= xEnd; x += 1 {
                DrawTexelPhongShaded(
                    x, y,
                    v1, v2, v3,
                    n1, n2, n3,
                    p1, p2, p3,
                    uv1, uv2, uv3,
                    texture, lights, zBuffer, image, ambient
                )
            }
        }
    }
}

// ---------------------------------------------------------------
// DrawTexelPhongShaded
// Draws a single pixel with perspective-correct texture sampling
// and per-pixel Phong shading.
//
// Combines the UV interpolation from DrawTexelFlatShaded with the
// normal and position interpolation from DrawPixelPhongShaded.
//
// Texture coordinate wrapping uses bitwise AND instead of modulo:
//   texX = i32(u * width)  & (width  - 1)
//   texY = i32(v * height) & (height - 1)
// This is only correct when width and height are powers of two
// (i.e. width-1 is an all-ones bit mask), and is faster than %.
// ---------------------------------------------------------------
DrawTexelPhongShaded :: proc(
    x, y:          f32,
    v1, v2, v3:    ^Vector3,
    n1, n2, n3:    ^Vector3,
    p1, p2, p3:    ^Vector3,
    uv1, uv2, uv3: ^Vector2,
    texture:       Texture,
    lights:        []Light,
    zBuffer:       ^ZBuffer,
    image:         ^rl.Image,
    ambient:       Vector3
) {
    ix := i32(x)
    iy := i32(y)
    if IsPointOutsideViewport(ix, iy) {
        return
    }

    p       := Vector2{x, y}
    weights := BarycentricWeights(p1.xy, p2.xy, p3.xy, p)
    alpha   := weights.x
    beta    := weights.y
    gamma   := weights.z

    // Perspective-correct depth.
    denom := alpha*p1.z + beta*p2.z + gamma*p3.z
    depth := 1.0 / denom

    zIndex := SCREEN_WIDTH*iy + ix
    if depth <= zBuffer[zIndex] {
        // Perspective-correct UV interpolation (same as DrawTexelFlatShaded).
        interpU := ((uv1.x*p1.z)*alpha + (uv2.x*p2.z)*beta + (uv3.x*p3.z)*gamma) * depth
        interpV := ((uv1.y*p1.z)*alpha + (uv2.y*p2.z)*beta + (uv3.y*p3.z)*gamma) * depth

        // Perspective-correct view-space fragment position.
        interpPos := ((v1^*p1.z)*alpha + (v2^*p2.z)*beta + (v3^*p3.z)*gamma) * depth

        // Interpolated and normalised fragment normal.
        interpNormal := Vector3Normalize(n1^*alpha + n2^*beta + n3^*gamma)

        // Texture lookup with power-of-two wrapping via bitwise AND.
        texX := i32(interpU * f32(texture.width )) & (texture.width  - 1)
        texY := i32(interpV * f32(texture.height)) & (texture.height - 1)
        tex  := texture.pixels[texY*texture.width + texX]

        // Per-pixel diffuse lighting accumulation.
        lightAccum := ambient
        for &light in lights {
            lightVec := Vector3Normalize(light.position - interpPos)
            diffuse  := math.max(Vector3DotProduct(interpNormal, lightVec), 0.0)
            lightAccum.r += diffuse * light.color.r * light.color.a
            lightAccum.g += diffuse * light.color.g * light.color.a
            lightAccum.b += diffuse * light.color.b * light.color.a
        }
        lightAccum.r = math.min(lightAccum.r, 1.0)
        lightAccum.g = math.min(lightAccum.g, 1.0)
        lightAccum.b = math.min(lightAccum.b, 1.0)

        // Apply lighting to the sampled texel colour.
        shadedTex := rl.Color{
            u8(f32(tex.r) * lightAccum.r),
            u8(f32(tex.g) * lightAccum.g),
            u8(f32(tex.b) * lightAccum.b),
            tex.a,
        }

        rl.ImageDrawPixel(image, ix, iy, shadedTex)
        zBuffer[zIndex] = depth
    }
}

// ---------------------------------------------------------------
// BarycentricWeights
// Computes the barycentric coordinates (α, β, γ) of a point p
// with respect to the triangle (a, b, c) in 2-D screen space.
//
// Barycentric coordinates satisfy:
//   p = α*a + β*b + γ*c
//   α + β + γ = 1
//
// They are used to interpolate any per-vertex attribute Q across
// the triangle: Q(p) = α*Q(a) + β*Q(b) + γ*Q(c).
//
// Derivation using the ratio-of-areas method:
//   The total signed area of triangle abc is:
//     area = (c-a).x*(b-a).y - (c-a).y*(b-a).x
//   The weight α (for vertex a, i.e. how much influence a has on p):
//     α corresponds to the sub-triangle pbc, computed as:
//     α = (pc.x*pb.y - pc.y*pb.x) / area
//   The weight β corresponds to sub-triangle apc:
//     β = (ac.x*ap.y - ac.y*ap.x) / area
//   γ is derived as 1 - α - β.
//
// The returned Vector3 packs (α, β, γ) into (.x, .y, .z).
// ---------------------------------------------------------------
BarycentricWeights :: proc(a, b, c, p: Vector2) -> Vector3 {
    // Edge vectors from a.
    ac := c - a
    ab := b - a
    ap := p - a

    // Vectors from p.
    pc := c - p
    pb := b - p

    // Signed area of the full triangle (twice, via cross product in 2-D).
    area := (ac.x * ab.y - ac.y * ab.x)

    // Barycentric weight α: sub-area opposite to vertex a (triangle pbc).
    alpha := (pc.x * pb.y - pc.y * pb.x) / area

    // Barycentric weight β: sub-area opposite to vertex b (triangle apc).
    beta := (ac.x * ap.y - ac.y * ap.x) / area

    // γ is the remainder; also equals the sub-area opposite to vertex c.
    gamma := 1.0 - alpha - beta

    return Vector3{alpha, beta, gamma}
}

// ---------------------------------------------------------------
// IsPointOutsideViewport
// Returns true if the integer pixel coordinate (x, y) lies outside
// the screen bounds [0, SCREEN_WIDTH) × [0, SCREEN_HEIGHT).
// Used to guard against out-of-bounds pixel writes.
// ---------------------------------------------------------------
IsPointOutsideViewport :: proc(x, y: i32) -> bool {
    return x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT
}
