package main

import "core:math"

ColliderType :: enum {
    None,
    Box,
    Sphere
}

BoxCollider :: struct {
    size: Vector3
}

SphereCollider :: struct {
    radius: f32
}

CollisionResult :: struct {
    hit: bool,
    normal: Vector3,
    depth: f32,
    contactPoint: Vector3
}

ResolveCollisions :: proc(models: []Model) {
    for i in 0..<len(models) {
        for j in i + 1..<len(models) {
            a := &models[i]
            b := &models[j]

            if a.rigidBody.isStatic && b.rigidBody.isStatic do continue
            
            result := GetCollisionResult(a, b)
            if result.hit {
                Correct(a, b, result)
                MoveStack(a, b, result)
                Push(a, result)
            }
        }
    }

    Correct :: proc(a, b: ^Model, r: CollisionResult) {
        correction := r.normal * max(r.depth, 0.0)

        if !a.rigidBody.isStatic && !b.rigidBody.isStatic {
            a.translation -= correction * 0.5
            b.translation += correction * 0.5
        } else if !a.rigidBody.isStatic {
            a.translation -= correction
        } else if !b.rigidBody.isStatic {
            b.translation += correction
        }
    }

    Push :: proc(m: ^Model, r: CollisionResult) {
        if m.rigidBody.isStatic do return

        m.rigidBody.velocity -= Vector3DotProduct(m.rigidBody.velocity, r.normal) * r.normal * m.rigidBody.bounciness
    }

    MoveStack :: proc(a, b: ^Model, r: CollisionResult) {
        a.rigidBody.isMovingBySupport = false
        b.rigidBody.isMovingBySupport = false

        if abs(Vector3DotProduct(r.normal, WORLD_UP)) < 0.7 do return

        isABottom := Vector3DotProduct(r.normal, WORLD_UP) > 0
        support := isABottom ? a : b
        resting := isABottom ? b : a

        if resting.rigidBody.isStatic do return

        if support.translation.y >= resting.translation.y do return

        supportNorm := Vector3DotProduct(support.rigidBody.velocity, r.normal)
        restingNorm := Vector3DotProduct(resting.rigidBody.velocity, r.normal)
        if supportNorm > restingNorm {
            resting.rigidBody.velocity += r.normal * (supportNorm - restingNorm)
        }

        supportSpeed := Vector3Length(support.rigidBody.velocity)
        if supportSpeed < MIN_VELOCITY_THRESHOLD do return

        supportTan := support.rigidBody.velocity - r.normal * supportNorm
        restingTan := resting.rigidBody.velocity - r.normal * restingNorm

        avgFriction := (support.rigidBody.friction + resting.rigidBody.friction) * 0.5
        resting.rigidBody.velocity += (supportTan - restingTan) * avgFriction
        resting.rigidBody.angularVelocity.y = support.rigidBody.angularVelocity.y

        resting.rigidBody.isMovingBySupport = true
    }
}

GetCollisionResult :: proc(a, b: ^Model) -> CollisionResult {
    switch {
        case a.colliderType == ColliderType.Box && b.colliderType == ColliderType.Box: 
            return BoxBox(a, b)
        case a.colliderType == ColliderType.Box && b.colliderType == ColliderType.Sphere:
            return BoxSphere(a, b)
        case a.colliderType == ColliderType.Sphere && b.colliderType == ColliderType.Box:
            result := BoxSphere(b, a)
            result.normal = -result.normal
            return result
        case a.colliderType == ColliderType.Sphere && b.colliderType == ColliderType.Sphere: 
            return SphereSphere(a, b)
    }
    return {}

    BoxBox :: proc(a, b: ^Model) -> CollisionResult {
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

        cA := FindClosestPoint(b.translation, a.translation, axesA, colSizeA)
        cB := FindClosestPoint(a.translation, b.translation, axesB, colSizeB)
        contactPoint := (cA + cB) * 0.5

        return CollisionResult{
            hit = true, 
            normal = minNormal, 
            depth = minDepth,
            contactPoint = contactPoint
        }

        ProjectRadius :: proc(colSize: Vector3, axes: [3]Vector3, axis: Vector3) -> f32 {
            return colSize.x * abs(Vector3DotProduct(axes[0], axis)) +
                   colSize.y * abs(Vector3DotProduct(axes[1], axis)) +
                   colSize.z * abs(Vector3DotProduct(axes[2], axis))
        }

        FindClosestPoint :: proc(point: Vector3, center: Vector3, axes: [3]Vector3, size: Vector3) -> Vector3 {
            d := point - center
            result := center

            for i in 0..<3 {
                dist := Vector3DotProduct(d, axes[i])
                dist = math.clamp(dist, -size[i], size[i])
                result += axes[i] * dist
            }

            return result
        }
    }

    BoxSphere :: proc(a, b: ^Model) -> CollisionResult {
        axes := GetAxesFromRotationMatrix(a.rotationMatrix)
        size := a.boxCollider.size * a.scale
        radius := b.sphereCollider.radius * b.scale
        dirAB := b.translation - a.translation
        
        closestPoint := a.translation
        for i in 0..<3 {
            dist := Vector3DotProduct(dirAB, axes[i])
            dist = math.clamp(dist, -size[i], size[i])
            closestPoint += axes[i] * dist
        }

        dirCpB := b.translation - closestPoint
        dist := Vector3Length(dirCpB)

        if dist >= radius do return CollisionResult{hit = false}

        normal := WORLD_UP
        if dist > 1e-6 {
            normal = dirCpB / dist
        }

        return CollisionResult{
            hit = true,
            normal = normal,
            depth = radius - dist,
            contactPoint = closestPoint,
        }
    }

    SphereSphere :: proc(a, b: ^Model) -> CollisionResult {
        rA := a.sphereCollider.radius * a.scale
        rB := b.sphereCollider.radius * b.scale
        diff := b.translation - a.translation
        dist := Vector3Length(diff)
        sum := rA + rB

        if dist >= sum do return CollisionResult{hit = false}

        normal := WORLD_UP
        if dist > 1e-6 {
            normal = diff / dist
        }

        return CollisionResult{
            hit = true,
            normal = normal,
            depth = sum - dist,
            contactPoint = a.translation + normal * rA,
        }
    }
}
