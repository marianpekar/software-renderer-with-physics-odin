package main

// ============================================================
// sort.odin
// Sorting utilities for triangle rasterisation.
//
// Triangle scan-line fill (see draw.odin) requires the three
// screen-space vertices to be sorted top-to-bottom by Y coordinate
// (p1.y ≤ p2.y ≤ p3.y) before the rasteriser iterates scan lines.
//
// The sort is implemented as a 3-element sorting network — a fixed
// sequence of compare-and-swap operations that correctly sorts
// exactly three elements in at most 3 comparisons, with no branches
// beyond the swaps themselves.  This is optimal for 3 elements.
//
// Network (Knuth, "The Art of Computer Programming", vol. 3):
//   Compare swap (p1, p2)
//   Compare swap (p2, p3)
//   Compare swap (p1, p2)
// After these three passes the three elements are in non-decreasing order.
//
// Because multiple vertex attributes (screen position, UV, view-space
// vertex position) must stay associated with the same vertex, every
// attribute is swapped together whenever the position pointers are swapped.
// The Sort procedure is an overloaded proc-group that dispatches to
// the appropriate overload based on which attribute sets are present.
// ============================================================

// Sort is an overloaded procedure that selects the appropriate
// sorting variant based on the set of attribute pointers provided:
//   SortPoints             — sort only screen-space points (p1, p2, p3)
//   SortPointsAndUVs       — sort points + texture coordinates
//   SortPointsAndVertices  — sort points + view-space vertex positions
//   SortPointsUVsAndVertices — sort points + UVs + view-space positions
Sort :: proc {
    SortPoints,
    SortPointsAndUVs,
    SortPointsAndVertices,
    SortPointsUVsAndVertices
}

// ---------------------------------------------------------------
// SortPoints
// Sorts three screen-space points by Y coordinate ascending.
// All three components (x, y, z) are swapped together to keep
// the point intact (z holds the depth value used for z-buffering).
//
// Implements the 3-element sorting network:
//   pass 1: if p1.y > p2.y → swap(p1, p2)
//   pass 2: if p2.y > p3.y → swap(p2, p3)
//   pass 3: if p1.y > p2.y → swap(p1, p2)
// After three passes: p1.y ≤ p2.y ≤ p3.y
// ---------------------------------------------------------------
SortPoints :: proc(p1, p2, p3: ^Vector3) {
    // Pass 1: bring the smallest-Y element to p1.
    if p1.y > p2.y {
        p1.x, p2.x = p2.x, p1.x
        p1.y, p2.y = p2.y, p1.y
        p1.z, p2.z = p2.z, p1.z
    }
    // Pass 2: bring the largest-Y element to p3.
    if p2.y > p3.y {
        p2.x, p3.x = p3.x, p2.x
        p2.y, p3.y = p3.y, p2.y
        p2.z, p3.z = p3.z, p2.z
    }
    // Pass 3: final check between p1 and p2 to complete the sort.
    if p1.y > p2.y {
        p1.x, p2.x = p2.x, p1.x
        p1.y, p2.y = p2.y, p1.y
        p1.z, p2.z = p2.z, p1.z
    }
}

// ---------------------------------------------------------------
// SortPointsAndUVs
// Sorts three screen-space points by Y coordinate and simultaneously
// keeps texture UV coordinates paired with their respective vertices.
//
// UV coordinates must stay associated with the vertex they belong to
// because they are interpolated per-pixel using barycentric weights
// derived from the sorted vertex order.  If UVs are not swapped in
// sync with points, texture coordinates would map to the wrong vertex.
// ---------------------------------------------------------------
SortPointsAndUVs :: proc(
    p1, p2, p3:   ^Vector3,
    uv1, uv2, uv3: ^Vector2
) {
    // Pass 1
    if p1.y > p2.y {
        p1.x, p2.x   = p2.x, p1.x
        p1.y, p2.y   = p2.y, p1.y
        p1.z, p2.z   = p2.z, p1.z
        uv1.x, uv2.x = uv2.x, uv1.x
        uv1.y, uv2.y = uv2.y, uv1.y
    }
    // Pass 2
    if p2.y > p3.y {
        p2.x, p3.x   = p3.x, p2.x
        p2.y, p3.y   = p3.y, p2.y
        p2.z, p3.z   = p3.z, p2.z
        uv2.x, uv3.x = uv3.x, uv2.x
        uv2.y, uv3.y = uv3.y, uv2.y
    }
    // Pass 3
    if p1.y > p2.y {
        p1.x, p2.x   = p2.x, p1.x
        p1.y, p2.y   = p2.y, p1.y
        p1.z, p2.z   = p2.z, p1.z
        uv1.x, uv2.x = uv2.x, uv1.x
        uv1.y, uv2.y = uv2.y, uv1.y
    }
}

// ---------------------------------------------------------------
// SortPointsAndVertices
// Sorts three screen-space points by Y and keeps the corresponding
// view-space vertex positions (v1, v2, v3) in sync.
//
// View-space positions are needed for per-pixel Phong shading, where
// the fragment's 3-D position is reconstructed via perspective-correct
// interpolation using barycentric weights.  Keeping them paired with
// their screen-space points ensures correct interpolation after sorting.
// ---------------------------------------------------------------
SortPointsAndVertices :: proc(
    p1, p2, p3: ^Vector3,
    v1, v2, v3: ^Vector3,
) {
    // Pass 1
    if p1.y > p2.y {
        p1.x, p2.x = p2.x, p1.x
        p1.y, p2.y = p2.y, p1.y
        p1.z, p2.z = p2.z, p1.z
        v1.x, v2.x = v2.x, v1.x
        v1.y, v2.y = v2.y, v1.y
        v1.z, v2.z = v2.z, v1.z
    }
    // Pass 2
    if p2.y > p3.y {
        p2.x, p3.x = p3.x, p2.x
        p2.y, p3.y = p3.y, p2.y
        p2.z, p3.z = p3.z, p2.z
        v2.x, v3.x = v3.x, v2.x
        v2.y, v3.y = v3.y, v2.y
        v2.z, v3.z = v3.z, v2.z
    }
    // Pass 3
    if p1.y > p2.y {
        p1.x, p2.x = p2.x, p1.x
        p1.y, p2.y = p2.y, p1.y
        p1.z, p2.z = p2.z, p1.z
        v1.x, v2.x = v2.x, v1.x
        v1.y, v2.y = v2.y, v1.y
        v1.z, v2.z = v2.z, v1.z
    }
}

// ---------------------------------------------------------------
// SortPointsUVsAndVertices
// Sorts three screen-space points by Y and simultaneously keeps
// both UV coordinates and view-space vertex positions paired with
// their respective screen-space points.
//
// Used by DrawTexturedTrianglePhongShaded which needs all three
// attribute sets to perform perspective-correct texture sampling
// combined with per-pixel Phong illumination.
// ---------------------------------------------------------------
SortPointsUVsAndVertices :: proc(
    p1, p2, p3:    ^Vector3,
    uv1, uv2, uv3: ^Vector2,
    v1, v2, v3:    ^Vector3,
) {
    // Pass 1
    if p1.y > p2.y {
        p1.x, p2.x   = p2.x, p1.x
        p1.y, p2.y   = p2.y, p1.y
        p1.z, p2.z   = p2.z, p1.z
        uv1.x, uv2.x = uv2.x, uv1.x
        uv1.y, uv2.y = uv2.y, uv1.y
        v1.x, v2.x   = v2.x, v1.x
        v1.y, v2.y   = v2.y, v1.y
        v1.z, v2.z   = v2.z, v1.z
    }
    // Pass 2
    if p2.y > p3.y {
        p2.x, p3.x   = p3.x, p2.x
        p2.y, p3.y   = p3.y, p2.y
        p2.z, p3.z   = p3.z, p2.z
        uv2.x, uv3.x = uv3.x, uv2.x
        uv2.y, uv3.y = uv3.y, uv2.y
        v2.x, v3.x   = v3.x, v2.x
        v2.y, v3.y   = v3.y, v2.y
        v2.z, v3.z   = v3.z, v2.z
    }
    // Pass 3
    if p1.y > p2.y {
        p1.x, p2.x   = p2.x, p1.x
        p1.y, p2.y   = p2.y, p1.y
        p1.z, p2.z   = p2.z, p1.z
        uv1.x, uv2.x = uv2.x, uv1.x
        uv1.y, uv2.y = uv2.y, uv1.y
        v1.x, v2.x   = v2.x, v1.x
        v1.y, v2.y   = v2.y, v1.y
        v1.z, v2.z   = v2.z, v1.z
    }
}
