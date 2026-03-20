package main

RigidBody :: struct {
    velocity: Vector3,
    acceleration: Vector3,
    angularVelocity: Vector3,
    angularAcceleration: Vector3,
    isStatic: bool,
}

BoxCollider :: struct {
    size: Vector3
}

CollisionResult :: struct {
    hit: bool,
    normal: Vector3,
    depth: f32,
}

ApplyForceAtPoint :: proc(model: ^Model, force: Vector3, contactPoint: Vector3) {
    model.rigidBody.acceleration += force

    r := contactPoint - model.translation
    torque := Vector3CrossProduct(r, force)
    model.rigidBody.angularAcceleration += {0, torque.y, 0}
}

ApplyPhysics :: proc(models: []Model, deltaTime: f32) {
    for &model in models {
        if model.rigidBody.isStatic do continue

        model.rigidBody.acceleration += GRAVITY
        model.rigidBody.velocity += model.rigidBody.acceleration * deltaTime
        model.rigidBody.velocity *= LINEAR_DRAG
        model.translation += model.rigidBody.velocity * deltaTime
        model.rigidBody.acceleration  = {}

        model.rigidBody.angularVelocity += model.rigidBody.angularAcceleration * deltaTime
        model.rigidBody.angularVelocity *= ANGULAR_DRAG

        avlength  := Vector3Length(model.rigidBody.angularVelocity)
        if avlength > 1e-6 {
            axis := model.rigidBody.angularVelocity / avlength
            angle := avlength * deltaTime
            delta := MakeRotationMatrixAxisAngle(axis, angle)
            model.rotationMatrix = Mat4Mul(delta, model.rotationMatrix)
        }
        model.rigidBody.angularAcceleration = {}
    }
}

ResolveCollisions :: proc(models: []Model) {
    for i in 0..<len(models) {
        for j in i + 1..<len(models) {
            a := &models[i]
            b := &models[j]

            if a.rigidBody.isStatic && b.rigidBody.isStatic do continue

            result := GetCollisionResult(a, b)

            if result.hit {
                correction := result.normal * max(result.depth, 0.0)

                if !a.rigidBody.isStatic && !b.rigidBody.isStatic {
                    a.translation -= correction * 0.5
                    b.translation += correction * 0.5
                } else if !a.rigidBody.isStatic {
                    a.translation -= correction
                } else if !b.rigidBody.isStatic {
                    b.translation += correction
                }

                if !a.rigidBody.isStatic {
                    velAlongNormal := Vector3DotProduct(a.rigidBody.velocity, result.normal)
                    if velAlongNormal > 0 {
                        a.rigidBody.velocity -= result.normal * velAlongNormal
                    }
                }

                if !b.rigidBody.isStatic {
                    velAlongNormal := Vector3DotProduct(b.rigidBody.velocity, result.normal)
                    if velAlongNormal < 0 {
                        b.rigidBody.velocity -= result.normal * velAlongNormal
                    }
                }
            }
        }
    }
}

GetCollisionResult :: proc(a, b: ^Model) -> CollisionResult {
    axesA := GetAxesFromRotationMatrix(a.rotationMatrix)
    axesB := GetAxesFromRotationMatrix(b.rotationMatrix)

    colSizeA := a.boxCollider.size * a.scale
    colSizeB := b.boxCollider.size * b.scale

    centerDiff := b.translation - a.translation

    axes: [15]Vector3
    axes[0] = axesA[0]
    axes[1] = axesA[1]
    axes[2] = axesA[2]
    axes[3] = axesB[0]
    axes[4] = axesB[1]
    axes[5] = axesB[2]

    idx := 6
    for i in 0..<3 {
        for j in 0..<3 {
            axes[idx] = Vector3CrossProduct(axesA[i], axesB[j])
            idx += 1
        }
    }

    minDepth  : f32 = max(f32)
    minNormal : Vector3

    for axis in axes {
        if Vector3Length(axis) < 1e-6 do continue

        normalizedAxis := Vector3Normalize(axis)

        rA := ProjectRadius(colSizeA, axesA, normalizedAxis)
        rB := ProjectRadius(colSizeB, axesB, normalizedAxis)

        centerProj := abs(Vector3DotProduct(centerDiff, normalizedAxis))
        overlap    := rA + rB - centerProj

        if overlap <= 0 do return CollisionResult{hit = false}

        if overlap < minDepth {
            minDepth  = overlap
            minNormal = normalizedAxis
        }
    }

    if Vector3DotProduct(centerDiff, minNormal) < 0 {
        minNormal = -minNormal
    }

    return CollisionResult{
        hit = true, 
        normal = minNormal, 
        depth = minDepth
    }

    ProjectRadius :: proc(colSize: Vector3, axes: [3]Vector3, axis: Vector3) -> f32 {
    return colSize.x * abs(Vector3DotProduct(axes[0], axis)) +
           colSize.y * abs(Vector3DotProduct(axes[1], axis)) +
           colSize.z * abs(Vector3DotProduct(axes[2], axis))
    }
}

