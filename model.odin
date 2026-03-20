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
    boxCollider: BoxCollider
}

LoadModel :: proc(meshPath: string, texturePath: cstring, isStatic: bool, color: rl.Color = rl.WHITE, wireColor: rl.Color = rl.GREEN) -> Model {
    model := Model{
        mesh = LoadMeshFromObjFile(meshPath),
        texture = LoadTextureFromFile(texturePath),
        color = color,
        wireColor = wireColor,
        rotationMatrix = MakeRotationMatrix(0,0,0),
        translation = Vector3{0.0, 0.0, 0.0},
        scale = 1.0
    }

    model.boxCollider.size = { 1.0, 1.0, 1.0 }

    if isStatic {
        FreezeModel(&model)
    } else {
        UnfreezeModel(&model)
    }

    return model
}

FreezeModel :: proc(model: ^Model) {
    model.rigidBody.isStatic = true
}

UnfreezeModel :: proc(model: ^Model) {
    model.rigidBody.isStatic = false
}

RotateAround :: proc(model: ^Model, axis: Vector3, angle: f32) {
    model.rotationMatrix = MakeRotationMatrixAxisAngle(axis, angle)
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