package main

import "core:math"

RigidBody :: struct {
    force: Vector3,
    velocity: Vector3,
    torque: Vector3,
    angularVelocity: Vector3,
    bounciness: f32,
    friction: f32,
    isStatic: bool,
    mass: f32,
    invMass: f32,
    isMovingBySupport: bool
}

AddForceAtPoint :: proc(model: ^Model, force: Vector3, contactPoint: Vector3) {
    rb, has_rb := &model.rigidBody.(RigidBody)
    if !has_rb do return

    rb.force += force

    r := contactPoint - model.translation
    torque := Vector3CrossProduct(r, force)
    rb.torque += torque
}

ApplyPhysics :: proc(models: []Model, deltaTime: f32) {
    for &model in models {
        rb, has_rb := model.rigidBody.?
        if !has_rb || rb.isStatic do continue

        ApplyGravity(&model, models, deltaTime)
        IntegrateLinearForce(&model, deltaTime)
        IntegrateTorque(&model, deltaTime)
    }
}

ApplyGravity :: proc(model: ^Model, models: []Model, deltaTime: f32) {
    isGrounded := false

    for &other in models {
        if &other == model do continue

        model.translation.y -= GROUND_PROBE_DIST
        probeResult := GetCollisionResult(model, &other)
        model.translation.y += GROUND_PROBE_DIST

        if probeResult.hit {
            if HasBoxCollider(model) {
                ApplyFriction(model, other)
                ApplyStabilization(model, probeResult.normal)

                if HasBoxCollider(&other)  {
                    overhang := GetOverhang(model, other)
                    surfaceAngle := math.acos(abs(Vector3DotProduct(probeResult.normal, WORLD_UP)))

                    if overhang.x <= 1e-6 && overhang.y <= 1e-6 && surfaceAngle < SLIDE_ANGLE_THRESHOLD {
                        isGrounded = true
                    }

                    if (overhang.x > 1e-6 || overhang.y > 1e-6) && probeResult.normal.y <= -0.5 {
                        ApplyTipping(model, probeResult.contactPoint, overhang)
                    }
                }
            }

            break
        }
    }

    if HasSphereCollider(model) {
        ApplyRolling(model)
    }

    rb := &model.rigidBody.(RigidBody)

    if !isGrounded {
        rb.velocity += GRAVITY * deltaTime
    }

    GetOverhang :: proc(model: ^Model, other: Model) -> Vector2 {
        otherAxes := GetAxesFromRotationMatrix(other.rotationMatrix)
        center := model.translation - other.translation
        projX := abs(Vector3DotProduct(center, otherAxes[0]))
        projZ := abs(Vector3DotProduct(center, otherAxes[2]))
        halfX := other.collider.(BoxCollider).x * other.scale
        halfZ := other.collider.(BoxCollider).z * other.scale
        return { projX - halfX, projZ - halfZ }
    }

    ApplyFriction :: proc(model: ^Model, other: Model) {
        rbo, has_rbo := other.rigidBody.?
        avgFriction := (model.rigidBody.?.friction + rbo.friction) * 0.5 if has_rbo else model.rigidBody.?.friction 

        rb := &model.rigidBody.(RigidBody)
        rb.force.x *= avgFriction
        rb.force.z *= avgFriction
        rb.torque.y *= avgFriction
    }

    ApplyRolling :: proc(model: ^Model) {
        rb := &model.rigidBody.(RigidBody)

        speed := Vector3Length(rb.velocity)
        if speed > MIN_VELOCITY_THRESHOLD && !rb.isMovingBySupport {
            axis := Vector3CrossProduct(WORLD_UP, rb.velocity / speed)
            radius := model.collider.(SphereCollider) * model.scale
            rb.angularVelocity = axis * (speed / radius)
        } else {
            rb.angularVelocity *= 0.1
        }   
    }

    ApplyStabilization :: proc(model: ^Model, normal: Vector3) {
        closestUp  := FindClosestUpAxis(model.rotationMatrix, normal)
        correction := Vector3CrossProduct(closestUp, normal)
        rb := &model.rigidBody.(RigidBody)
        rb.torque += correction * STABILIZATION_STRENGTH
    }

    ApplyTipping :: proc(model: ^Model, contactPoint: Vector3, overhang: Vector2) {
        rContact := model.translation - contactPoint
        gravTorque := Vector3CrossProduct(rContact, GRAVITY)
        if overhang.x > 1e-6 do gravTorque.x -= overhang.x
        if overhang.y > 1e-6 do gravTorque.z -= overhang.y
        rb := &model.rigidBody.(RigidBody)
        rb.torque += Vector3Normalize(gravTorque) * rb.mass * TIPPING_STRENGTH
    }
}

IntegrateLinearForce :: proc(model: ^Model, deltaTime: f32) {
    rb := &model.rigidBody.(RigidBody)
    rb.velocity += rb.force * rb.invMass * deltaTime
    rb.force = {}

    drag: f32
    if HasBoxCollider(model) { drag = LINEAR_DRAG } else { drag = SPHERE_LINEAR_DRAG }
    rb.velocity *= drag
    
    if Vector3Length(rb.velocity) > MIN_VELOCITY_THRESHOLD {
        model.translation += rb.velocity * deltaTime
    }
}

IntegrateTorque :: proc(model: ^Model, deltaTime: f32) {
    rb := &model.rigidBody.(RigidBody)
    rb.angularVelocity += rb.torque * rb.invMass * deltaTime
    rb.torque = {}

    drag: f32
    if HasBoxCollider(model) { drag = ANGULAR_DRAG } else { drag = SPHERE_ANGULAR_DRAG }
    rb.angularVelocity *= drag

    avlength  := Vector3Length(rb.angularVelocity)
    if avlength > MIN_ANGULAR_VELOCITY_THRESHOLD {
        axis := rb.angularVelocity / avlength
        angle := avlength * deltaTime
        delta := MakeRotationMatrixAxisAngle(axis, angle)
        model.rotationMatrix = Mat4Mul(delta, model.rotationMatrix)
    }
}

FindClosestUpAxis :: proc(rotationMatrix: Matrix4x4, referenceUp: Vector3) -> Vector3 {
    axes := GetAxesFromRotationMatrix(rotationMatrix)

    closestAxis := axes[0]
    closestDot := abs(Vector3DotProduct(axes[0], referenceUp))

    for i in 1..<3 {
        d := abs(Vector3DotProduct(axes[i], referenceUp))
        if d > closestDot {
            closestDot  = d
            closestAxis = axes[i]
        }
    }

    if Vector3DotProduct(closestAxis, referenceUp) < 0 {
        closestAxis = -closestAxis
    }

    return closestAxis
}