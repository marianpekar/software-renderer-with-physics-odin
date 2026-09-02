package main

import "core:math"

BoxCollider :: Vector3
SphereCollider :: f32

Collider :: union {
    BoxCollider,
    SphereCollider
}

CollisionResult :: struct {
    hit: bool,
    normal: Vector3,
    depth: f32,
    contactPoint: Vector3
}

ResolveCollisions :: proc(models: []Model) {
    context.allocator = context.temp_allocator

    bounds := GetWorldAABB(models)
    octree := MakeOctree(bounds)
    for i in 0..<len(models) {
        OctreeInsert(&octree, i, GetModelAABB(&models[i]), models)
    }

    for pair in OctreeGetCandidatePairs(&octree) {
        a := &models[pair[0]]
        b := &models[pair[1]]

        rba, has_rba := a.rigidBody.?
        rbb, has_rbb := b.rigidBody.?
        if (!has_rba || rba.isStatic) && (!has_rbb || rbb.isStatic) do continue

        result := GetCollisionResult(a, b)
        if result.hit {
            Correct(a, b, result.normal, result.depth)
            MoveStack(a, b, result.normal)
            Push(a, result.normal)
        }
    }

    free_all(context.temp_allocator)

    Correct :: proc(a, b: ^Model, normal: Vector3, depth: f32) {
        correction := normal * max(depth, 0.0)

        rba, has_rba := a.rigidBody.?
        rbb, has_rbb := b.rigidBody.?

        if (has_rba && !a.rigidBody.?.isStatic) && (has_rbb && !b.rigidBody.?.isStatic) {
            a.translation -= correction * 0.5
            b.translation += correction * 0.5
        } else if !has_rba || !a.rigidBody.?.isStatic {
            a.translation -= correction
        } else if !has_rbb || !b.rigidBody.?.isStatic {
            b.translation += correction
        }
    }

    Push :: proc(m: ^Model, normal: Vector3) {
        rb, has_rb := &m.rigidBody.(RigidBody)
        if !has_rb || rb.isStatic do return
        
        rb.velocity -= Vector3DotProduct(rb.velocity, normal) * normal * rb.bounciness
    }

    MoveStack :: proc(a, b: ^Model, normal: Vector3) {
        rba, has_rba := &a.rigidBody.(RigidBody)
        rbb, has_rbb := &b.rigidBody.(RigidBody)

        if has_rba do rba.isMovingBySupport = false
        if has_rbb do rbb.isMovingBySupport = false

        if abs(Vector3DotProduct(normal, WORLD_UP)) < 0.7 do return

        isABottom := Vector3DotProduct(normal, WORLD_UP) > 0
        support := isABottom ? a : b
        resting := isABottom ? b : a

        rbr, has_rbr := &resting.rigidBody.(RigidBody)
        if !has_rbr || rbr.isStatic do return

        if support.translation.y >= resting.translation.y do return

        rbs, has_rbs := &support.rigidBody.(RigidBody)

        if !has_rbs || !has_rbr do return

        supportNorm := Vector3DotProduct(rbs.velocity, normal)
        restingNorm := Vector3DotProduct(rbr.velocity, normal)
        if supportNorm > restingNorm {
            rbr.velocity += normal * (supportNorm - restingNorm)
        }

        supportSpeed := Vector3Length(rbs.velocity)
        if supportSpeed < MIN_VELOCITY_THRESHOLD do return

        supportTan := rbs.velocity - normal * supportNorm
        restingTan := rbr.velocity - normal * restingNorm

        avgFriction := (rbs.friction + rbr.friction) * 0.5
        rbr.velocity += (supportTan - restingTan) * avgFriction
        rbr.angularVelocity.y = rbs.angularVelocity.y

        rbr.isMovingBySupport = true
    }
}

GetCollisionResult :: proc(a, b: ^Model) -> CollisionResult {
    if HasBoxCollider(a) {
        if HasBoxCollider(b) {
            return BoxBox(a, b)
        }
        return BoxSphere(a, b)
    }

    if HasSphereCollider(a) {
        if HasBoxCollider(b) {
            result := BoxSphere(b, a)
            result.normal = -result.normal
            return result
        }
        return SphereSphere(a, b)
    }
    return {}

    BoxBox :: proc(a, b: ^Model) -> CollisionResult {
        axesA := GetAxesFromRotationMatrix(a.rotationMatrix)
        axesB := GetAxesFromRotationMatrix(b.rotationMatrix)

        colliderA := a.collider.(BoxCollider) * a.scale
        colliderB := b.collider.(BoxCollider) * b.scale

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

        direction := b.translation - a.translation

        minDepth  : f32 = max(f32)
        minNormal : Vector3

        for axis in axes {
            if Vector3Length(axis) < 1e-6 do continue

            normalizedAxis := Vector3Normalize(axis)

            rA := ProjectRadius(colliderA, axesA, normalizedAxis)
            rB := ProjectRadius(colliderB, axesB, normalizedAxis)

            projection := abs(Vector3DotProduct(direction, normalizedAxis))
            overlap    := rA + rB - projection

            if overlap <= 0 do return CollisionResult{hit = false}

            if overlap < minDepth {
                minDepth  = overlap
                minNormal = normalizedAxis
            }
        }

        cA := FindClosestPoint(b.translation, a.translation, axesA, colliderA)
        cB := FindClosestPoint(a.translation, b.translation, axesB, colliderB)
        contactPoint := (cA + cB) * 0.5

        return CollisionResult{
            hit = true, 
            normal = -minNormal if Vector3DotProduct(direction, minNormal) < 0 else minNormal,
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
        size := a.collider.(BoxCollider) * a.scale
        radius := b.collider.(SphereCollider) * b.scale
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
        rA := a.collider.(SphereCollider) * a.scale
        rB := b.collider.(SphereCollider) * b.scale
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
