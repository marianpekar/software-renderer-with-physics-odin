package main

// ============================================================
// mesh.odin
// Mesh data structures and loaders.
//
// A Mesh holds raw geometry (vertices, normals, UV coordinates,
// and triangles) plus a set of pre-allocated "transformed" buffers
// that are written each frame by ApplyTransformations (model.odin)
// without reallocating memory.
//
// Two sources of geometry are supported:
//   1. LoadMeshFromObjFile — parses a Wavefront OBJ file.
//   2. MakeCube            — procedurally generates a unit cube.
// ============================================================

import "core:strings"
import "core:strconv"
import "core:log"
import "core:os"

// Triangle stores the nine indices needed to look up the three
// vertices, three UV coordinates, and three normals of one triangle.
//
// Layout:  [v1, v2, v3,  vt1, vt2, vt3,  vn1, vn2, vn3]
//   Indices 0-2 → positions in the vertices array
//   Indices 3-5 → positions in the uvs array
//   Indices 6-8 → positions in the normals array
//
// This matches the f v/vt/vn triplet format of Wavefront OBJ files.
Triangle :: [9]int

// Mesh holds all geometry data for one renderable object.
//
// Fields:
//   transformedVertices — pre-allocated buffer; overwritten each frame
//                         with vertices in view space (position * viewMatrix).
//   transformedNormals  — pre-allocated buffer; overwritten each frame
//                         with normals in view space (normal * viewMatrix).
//   vertices            — original object-space vertex positions.
//   normals             — original object-space surface normals.
//   uvs                 — texture coordinates (U, V) in [0,1]² range.
//   triangles           — index triples referencing vertices/uvs/normals.
Mesh :: struct {
    transformedVertices: []Vector3,
    transformedNormals:  []Vector3,
    vertices:            []Vector3,
    normals:             []Vector3,
    uvs:                 []Vector2,
    triangles:           []Triangle,
}

// ---------------------------------------------------------------
// LoadMeshFromObjFile
// Parses a Wavefront OBJ file and returns the corresponding Mesh.
//
// The Wavefront OBJ format is a line-based ASCII format.
// Relevant line types parsed here:
//   "v  x y z"        — vertex position
//   "vn nx ny nz"     — vertex normal
//   "vt u v"          — texture coordinate
//   "f v/vt/vn ..."   — face definition (triangle assumed, i.e. 3 vertices)
//
// OBJ files use 1-based indices; the parser subtracts 1 from every
// index so they become 0-based Odin array indices.
//
// After parsing, two additional slices are allocated to hold the
// per-frame transformed copies of vertices and normals.
// ---------------------------------------------------------------
LoadMeshFromObjFile :: proc(filepath: string) -> Mesh {
    // Read the entire file into memory at once.
    data, err := os.read_entire_file(filepath, context.allocator)
    if err != nil {
        log.panicf("Failed to read file %s", filepath)
    }

    // Dynamic arrays grow as lines are parsed.
    vertices:  [dynamic]Vector3
    normals:   [dynamic]Vector3
    triangles: [dynamic]Triangle
    uvs:       [dynamic]Vector2

    // Iterate over every line in the file.
    it := string(data)
    for line in strings.split_lines_iterator(&it) {
        // Skip blank lines.
        if len(line) <= 0 {
            continue
        }

        // Split on spaces; the first token identifies the record type.
        split := strings.split(line, " ")
        switch split[0] {
            case "v":
                // Vertex position: read three floats.
                x := ParseCoord(split, 1)
                y := ParseCoord(split, 2)
                z := ParseCoord(split, 3)
                append(&vertices, Vector3{x, y, z})

            case "vn":
                // Vertex normal: read three floats.
                nx := ParseCoord(split, 1)
                ny := ParseCoord(split, 2)
                nz := ParseCoord(split, 3)
                append(&normals, Vector3{nx, ny, nz})

            case "vt":
                // Texture coordinate: read two floats (U and V).
                u := ParseCoord(split, 1)
                v := ParseCoord(split, 2)
                append(&uvs, Vector2{u, v})

            case "f":
                // Face definition: "f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3"
                // Each token is a slash-separated triplet of indices.
                v1, vt1, vn1 := ParseIndices(split, 1)
                v2, vt2, vn2 := ParseIndices(split, 2)
                v3, vt3, vn3 := ParseIndices(split, 3)
                append(&triangles, Triangle{v1, v2, v3, vt1, vt2, vt3, vn1, vn2, vn3})
        }
    }

    // Construct the Mesh.  The "transformed" slices are pre-allocated
    // to the same size as their originals so the rasteriser can write
    // view-space data into them without allocating each frame.
    return Mesh {
        transformedVertices = make([]Vector3, len(vertices)),
        transformedNormals  = make([]Vector3, len(normals)),
        vertices  = vertices[:],
        normals   = normals[:],
        uvs       = uvs[:],
        triangles = triangles[:]
    }

    // --- Nested helper: ParseCoord ---
    // Parses the float at position idx in the split line tokens.
    // Panics with a descriptive message if parsing fails (malformed OBJ).
    ParseCoord :: proc(split: []string, idx: i32) -> f32 {
        coord, ok := strconv.parse_f32(split[idx])
        if !ok {
            log.panic("Failed to parse coordinate")
        }
        return coord
    }

    // --- Nested helper: ParseIndices ---
    // Parses the slash-separated "v/vt/vn" triplet at position idx
    // and returns zero-based integer indices for vertex, UV, and normal.
    // OBJ uses 1-based indices, so we subtract 1 from each.
    ParseIndices :: proc(split: []string, idx: int) -> (int, int, int) {
        // Split the token "v/vt/vn" on the '/' separator.
        indices := strings.split(split[idx], "/")

        // Parse the vertex position index.
        v, okv := strconv.parse_int(indices[0])
        if !okv {
            log.panic("Failed to parse index of a vertex")
        }

        // Parse the texture coordinate index.
        vt, okvt := strconv.parse_int(indices[1])
        if !okvt {
            log.panic("Failed to parse index of a UV")
        }

        // Parse the normal index.
        vn, okvn := strconv.parse_int(indices[2])
        if !okvn {
            log.panic("Failed to parse index of a normal")
        }

        // Convert from 1-based (OBJ) to 0-based (Odin) indices.
        return v - 1, vt - 1, vn - 1
    }
}

// ---------------------------------------------------------------
// MakeCube
// Procedurally generates a unit cube mesh centred at the origin,
// with vertices at ±1 on each axis.
//
// Geometry layout (8 unique vertices):
//   0 = (-1,-1,-1)   1 = (-1,+1,-1)   2 = (+1,+1,-1)   3 = (+1,-1,-1)
//   4 = (+1,+1,+1)   5 = (+1,-1,+1)   6 = (-1,+1,+1)   7 = (-1,-1,+1)
//
// 6 face normals (one per face, shared by both triangles of each face):
//   0 = front  (0,0,-1)    1 = right  (+1,0,0)
//   2 = back   (0,0,+1)    3 = left   (-1,0,0)
//   4 = top    (0,+1,0)    5 = bottom (0,-1,0)
//
// 4 UV coordinates cover the four corners of a face quad:
//   0 = (1,1)  1 = (1,0)  2 = (0,0)  3 = (0,1)
//
// Each face is divided into 2 triangles (12 triangles total).
// Triangle indices are listed as: [v1,v2,v3, vt1,vt2,vt3, vn1,vn2,vn3]
// ---------------------------------------------------------------
MakeCube :: proc() -> Mesh {
    // --- Vertex positions ---
    vertices := make([]Vector3, 8)
    vertices[0] = Vector3{-1.0, -1.0, -1.0} // front-bottom-left
    vertices[1] = Vector3{-1.0,  1.0, -1.0} // front-top-left
    vertices[2] = Vector3{ 1.0,  1.0, -1.0} // front-top-right
    vertices[3] = Vector3{ 1.0, -1.0, -1.0} // front-bottom-right
    vertices[4] = Vector3{ 1.0,  1.0,  1.0} // back-top-right
    vertices[5] = Vector3{ 1.0, -1.0,  1.0} // back-bottom-right
    vertices[6] = Vector3{-1.0,  1.0,  1.0} // back-top-left
    vertices[7] = Vector3{-1.0, -1.0,  1.0} // back-bottom-left

    // --- Face normals (one per face) ---
    normals := make([]Vector3, 6)
    normals[0] = { 0.0,  0.0, -1.0} // front face
    normals[1] = { 1.0,  0.0,  0.0} // right face
    normals[2] = { 0.0,  0.0,  1.0} // back face
    normals[3] = {-1.0,  0.0,  0.0} // left face
    normals[4] = { 0.0,  1.0,  0.0} // top face
    normals[5] = { 0.0, -1.0,  0.0} // bottom face

    // --- UV coordinates (four corners of a face tile) ---
    uvs := make([]Vector2, 4)
    uvs[0] = Vector2{1.0, 1.0} // bottom-right
    uvs[1] = Vector2{1.0, 0.0} // top-right
    uvs[2] = Vector2{0.0, 0.0} // top-left
    uvs[3] = Vector2{0.0, 1.0} // bottom-left

    // --- Triangles: 2 per face × 6 faces = 12 ---
    // Each Triangle is [v1,v2,v3, uv1,uv2,uv3, n1,n2,n3]
    triangles := make([]Triangle, 12)

    // Front face (-Z normal)
    triangles[0] =  Triangle{0, 1, 2,  0, 1, 2,  0, 0, 0}
    triangles[1] =  Triangle{0, 2, 3,  0, 2, 3,  0, 0, 0}
    // Right face (+X normal)
    triangles[2] =  Triangle{3, 2, 4,  0, 1, 2,  1, 1, 1}
    triangles[3] =  Triangle{3, 4, 5,  0, 2, 3,  1, 1, 1}
    // Back face (+Z normal)
    triangles[4] =  Triangle{5, 4, 6,  0, 1, 2,  2, 2, 2}
    triangles[5] =  Triangle{5, 6, 7,  0, 2, 3,  2, 2, 2}
    // Left face (-X normal)
    triangles[6] =  Triangle{7, 6, 1,  0, 1, 2,  3, 3, 3}
    triangles[7] =  Triangle{7, 1, 0,  0, 2, 3,  3, 3, 3}
    // Top face (+Y normal)
    triangles[8] =  Triangle{1, 6, 4,  0, 1, 2,  4, 4, 4}
    triangles[9] =  Triangle{1, 4, 2,  0, 2, 3,  4, 4, 4}
    // Bottom face (-Y normal)
    triangles[10] = Triangle{5, 7, 0,  0, 1, 2,  5, 5, 5}
    triangles[11] = Triangle{5, 0, 3,  0, 2, 3,  5, 5, 5}

    return Mesh{
        // Pre-allocate transformed buffers for vertices and normals.
        transformedVertices = make([]Vector3, 8),
        transformedNormals  = make([]Vector3, 6),
        vertices  = vertices,
        normals   = normals,
        triangles = triangles,
        uvs       = uvs
    }
}
