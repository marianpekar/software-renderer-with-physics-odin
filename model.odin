package main

import rl "vendor:raylib"

Model :: struct {
    mesh: Mesh,
    texture: Texture,
    color: rl.Color,
    wireColor: rl.Color,
    translation: Vector3,
    rotationMatrix: Matrix4x4,
    scale: f32,
    rigidBody: RigidBody,
    collider: Collider
}

LoadModel :: proc(meshPath: string, texturePath: cstring, color: rl.Color = rl.WHITE) -> Model {
    model := Model{
        mesh = LoadMeshFromObjFile(meshPath),
        texture = LoadTextureFromFile(texturePath),
        rotationMatrix = MakeRotationMatrix(0,0,0),
        translation = Vector3{0.0, 0.0, 0.0},
        scale = 1.0,
    }

    SetColor(&model, color)

    return model
}

AddRigidbody :: proc(model: ^Model, isStatic: bool, bounciness: f32 = 1.0, friction: f32 = 0.5, mass: f32 = 1.0) {
    model.rigidBody.mass = mass
    model.rigidBody.invMass = 1.0 / mass
    model.rigidBody.bounciness = bounciness
    model.rigidBody.friction = friction
    model.rigidBody.isStatic = isStatic
}

AddBoxCollider :: proc(model: ^Model, size: Vector3 = { 1.0, 1.0, 1.0 }) {
    model.collider = BoxCollider{size = size}
}

AddSphereCollider :: proc(model: ^Model, radius: f32 = 1.0) {
    model.collider = SphereCollider{radius = radius}
}

HasBoxCollider :: proc(model: ^Model) -> bool {
    _, has := model.collider.(BoxCollider)
    return has
}

HasSphereCollider :: proc(model: ^Model) -> bool {
    _, has := model.collider.(SphereCollider)
    return has
}

SetColor :: proc(model: ^Model, color: rl.Color) {
    model.color = color
    model.wireColor = color
}

RotateAround :: proc(model: ^Model, axis: Vector3, angle: f32) {
    model.rotationMatrix = MakeRotationMatrixAxisAngle(axis, angle * DEG_TO_RAD)
}

ApplyTransformations :: proc(model: ^Model, camera: Camera) {
    translationMatrix := MakeTranslationMatrix(model.translation.x, model.translation.y, model.translation.z)
    scaleMatrix := MakeScaleMatrix(model.scale, model.scale, model.scale)
    modelMatrix := Mat4Mul(translationMatrix, Mat4Mul(model.rotationMatrix, scaleMatrix))
    viewMatrix  := MakeViewMatrix(camera.position, camera.target)
    viewMatrix  = Mat4Mul(viewMatrix, modelMatrix)

    TransformVertices(&model.mesh.transformedVertices, model.mesh.vertices, viewMatrix)
    TransformVertices(&model.mesh.transformedNormals, model.mesh.normals, viewMatrix)
}

TransformVertices:: proc(transformed: ^[]Vector3, original: []Vector3, mat: Matrix4x4) {
    for i in 0..<len(original) {
        transformed[i] = Mat4MulVec3(mat, original[i])
    }
}