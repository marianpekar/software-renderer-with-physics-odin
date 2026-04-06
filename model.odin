package main

// ============================================================
// model.odin
// The Model aggregate type and the procedures that operate on it.
//
// A Model is the central game-object type: it bundles a Mesh,
// a Texture, rendering colours, spatial transforms (translation,
// rotation matrix, scale), a RigidBody for physics integration,
// and a collider descriptor.  Every object in the scene is a Model.
//
// Transform pipeline (applied each frame in ApplyTransformations):
//   Object space
//     → [Scale]          (uniform scale along all axes)
//     → [Rotation]       (orientation encoded as a 4×4 matrix)
//     → [Translation]    (world-space position)
//     → Model space (world space)
//     → [View / Camera]  (world → view transform)
//     → View space       (what the rasteriser actually works with)
// ============================================================

import rl "vendor:raylib"

// Model is the central game-object aggregate.
//
// Fields:
//   mesh           — geometry buffers (vertices, normals, uvs, triangles)
//   texture        — CPU-side texture pixel data for software rendering
//   color          — flat/shaded tint colour for non-textured modes
//   wireColor      — line colour used in wireframe render modes
//   translation    — world-space position of the model's pivot/origin
//   rotationMatrix — current orientation stored as a full 4×4 rotation
//                    matrix so it can be updated incrementally each frame
//                    by multiplying in small angular-velocity deltas
//   scale          — uniform scale factor applied before rotation/translation
//   rigidBody      — physics state (velocity, forces, mass, etc.)
//   sphereCollider — bounding sphere parameters (used when colliderType=Sphere)
//   colliderType   — which collider is active (None / Box / Sphere)
//   boxCollider    — OBB half-extents (used when colliderType=Box)
Model :: struct {
    mesh:           Mesh,
    texture:        Texture,
    color:          rl.Color,
    wireColor:      rl.Color,
    translation:    Vector3,
    rotationMatrix: Matrix4x4,
    scale:          f32,
    rigidBody:      RigidBody,
    sphereCollider: SphereCollider,
    colliderType:   ColliderType,
    boxCollider:    BoxCollider
}

// ---------------------------------------------------------------
// LoadModel
// Creates a fully initialised Model from an OBJ mesh file and a
// texture image file.
//
// Defaults:
//   rotationMatrix = identity (no initial rotation)
//   translation    = origin
//   scale          = 1.0
//   colliderType   = None (no physics collider until one is attached)
//
// The optional color parameter tints both the fill colour and the
// wireframe colour; defaults to rl.WHITE (no tint).
// ---------------------------------------------------------------
LoadModel :: proc(meshPath: string, texturePath: cstring, color: rl.Color = rl.WHITE) -> Model {
    model := Model{
        mesh           = LoadMeshFromObjFile(meshPath),
        texture        = LoadTextureFromFile(texturePath),
        // Identity rotation: MakeRotationMatrix(0,0,0) produces no rotation.
        rotationMatrix = MakeRotationMatrix(0,0,0),
        translation    = Vector3{0.0, 0.0, 0.0},
        scale          = 1.0,
        colliderType   = ColliderType.None
    }

    SetColor(&model, color)

    return model
}

// ---------------------------------------------------------------
// AddRigidbody
// Attaches physics properties to a model, enabling it to be
// integrated by ApplyPhysics each frame.
//
// Parameters:
//   isStatic   — if true, the body is immovable (infinite mass),
//                useful for floors and walls
//   bounciness — coefficient of restitution; >1 means super-elastic
//                (energy is gained on bounce, useful for gameplay feel)
//   friction   — damping applied to lateral forces when resting on a surface
//   mass       — mass in kg; heavier bodies accelerate less per applied force
//
// The inverse mass (invMass) is pre-computed and stored because the
// physics integration only ever needs 1/mass and avoids repeated divisions.
// ---------------------------------------------------------------
AddRigidbody :: proc(model: ^Model, isStatic: bool, bounciness: f32 = 1.0, friction: f32 = 0.5, mass: f32 = 1.0) {
    model.rigidBody.mass      = mass
    model.rigidBody.invMass   = 1.0 / mass
    model.rigidBody.bounciness = bounciness
    model.rigidBody.friction   = friction
    model.rigidBody.isStatic   = isStatic
}

// ---------------------------------------------------------------
// AddBoxCollider
// Attaches an Oriented Bounding Box (OBB) collider to the model.
// The size vector holds the half-extents along each local axis,
// i.e. the box extends ±size.x in X, ±size.y in Y, ±size.z in Z.
// ---------------------------------------------------------------
AddBoxCollider :: proc(model: ^Model, size: Vector3 = { 1.0, 1.0, 1.0 }) {
    model.colliderType    = ColliderType.Box
    model.boxCollider.size = size
}

// ---------------------------------------------------------------
// AddSphereCollider
// Attaches a sphere collider to the model.
// The radius is in object-space units; it is multiplied by model.scale
// at runtime to get the actual world-space radius.
// ---------------------------------------------------------------
AddSphereCollider :: proc(model: ^Model, radius: f32 = 1.0) {
    model.colliderType          = ColliderType.Sphere
    model.sphereCollider.radius = radius
}

// ---------------------------------------------------------------
// SetColor
// Sets both the filled-face colour and the wireframe edge colour
// to the same value.  Keeping them in sync avoids the two colours
// diverging accidentally when the user changes a model's tint.
// ---------------------------------------------------------------
SetColor :: proc(model: ^Model, color: rl.Color) {
    model.color     = color
    model.wireColor = color
}

// ---------------------------------------------------------------
// RotateAround
// Sets the model's rotation to a single axis-angle rotation,
// overwriting any previous orientation.
//
// Internally converts the angle from degrees to radians and
// constructs a new rotation matrix via Rodrigues' formula
// (see MakeRotationMatrixAxisAngle in matrix.odin).
//
// Note: this replaces the current rotationMatrix entirely rather
// than composing with it, so it is intended for setting an initial
// orientation, not for incremental frame-by-frame rotation.
// For incremental rotation see IntegrateTorque in physics.odin.
// ---------------------------------------------------------------
RotateAround :: proc(model: ^Model, axis: Vector3, angle: f32) {
    model.rotationMatrix = MakeRotationMatrixAxisAngle(axis, angle * DEG_TO_RAD)
}

// ---------------------------------------------------------------
// ApplyTransformations
// Computes the combined model-view matrix and transforms all
// vertices and normals into view space, storing the results in
// the mesh's transformedVertices and transformedNormals buffers.
//
// Transform pipeline:
//   1. Scale matrix S          — uniform scale from model.scale
//   2. Rotation matrix R       — model.rotationMatrix (orientation)
//   3. Translation matrix T    — model.translation (world position)
//   4. modelMatrix = T * R * S — full object-to-world transform
//   5. viewMatrix              — world-to-camera transform
//   6. combined = viewMatrix * modelMatrix
//
// Both vertices and normals are transformed by the combined matrix.
// Strictly speaking, normals should be transformed by the inverse
// transpose of the model matrix to remain perpendicular to faces
// after non-uniform scale.  Since only uniform scale is used here,
// transforming normals by the same matrix is correct.
// ---------------------------------------------------------------
ApplyTransformations :: proc(model: ^Model, camera: Camera) {
    // Build the individual transform matrices.
    translationMatrix := MakeTranslationMatrix(model.translation.x, model.translation.y, model.translation.z)
    scaleMatrix       := MakeScaleMatrix(model.scale, model.scale, model.scale)

    // Compose: T * (R * S)  — scale first, then rotate, then translate.
    modelMatrix := Mat4Mul(translationMatrix, Mat4Mul(model.rotationMatrix, scaleMatrix))

    // Build the view matrix and fold the model matrix into it so we get
    // a single world-to-view-space matrix from one multiplication.
    viewMatrix := MakeViewMatrix(camera.position, camera.target)
    viewMatrix  = Mat4Mul(viewMatrix, modelMatrix)

    // Transform all vertices and normals into view space.
    TransformVertices(&model.mesh.transformedVertices, model.mesh.vertices, viewMatrix)
    TransformVertices(&model.mesh.transformedNormals,  model.mesh.normals,  viewMatrix)
}

// ---------------------------------------------------------------
// TransformVertices
// Applies a 4×4 matrix to every vector in the 'original' slice and
// writes the results into 'transformed'.
//
// Both vertex positions and normals use the same procedure because
// both are represented as Vector3 and the combined model-view matrix
// handles both correctly under the uniform-scale assumption.
//
// This procedure is called each frame, so it overwrites the
// previous frame's values — there is no accumulation.
// ---------------------------------------------------------------
TransformVertices :: proc(transformed: ^[]Vector3, original: []Vector3, mat: Matrix4x4) {
    for i in 0..<len(original) {
        // Mat4MulVec3 treats the vector as a homogeneous point (w=1),
        // which is correct for both positions (want translation) and
        // normals (which include the rotation from the rotation matrix).
        transformed[i] = Mat4MulVec3(mat, original[i])
    }
}
