package main

// ============================================================
// physics.odin
// Rigid-body physics simulation: force/torque accumulation,
// semi-implicit Euler integration, and the heuristic behaviours
// that make box stacking, tipping, stabilisation, and sphere
// rolling look plausible.
//
// Integration scheme: semi-implicit (symplectic) Euler
//   v(t+dt) = v(t) + a(t) * dt    ← velocity updated first
//   x(t+dt) = x(t) + v(t+dt) * dt ← position uses new velocity
// This is more energy-conserving than explicit Euler and is the
// standard choice for game physics.
//
// The simulation runs at a fixed PHYSICS_TIMESTEP (see constants.odin)
// accumulated from real frame time in main.odin's game loop.
// ============================================================

import "core:math"

// RigidBody stores all physics state for one simulated object.
//
// Fields:
//   force            — net force accumulator (reset after integration)
//   velocity         — linear velocity (m/s)
//   torque           — net torque accumulator (reset after integration)
//   angularVelocity  — angular velocity (radians/s) stored as a vector;
//                      its direction is the rotation axis and its
//                      magnitude is the angular speed
//   bounciness       — coefficient of restitution (≥0); 1 = perfectly
//                      elastic, >1 = super-elastic (arcade feel)
//   friction         — lateral damping factor applied when resting on a surface
//   isStatic         — if true, the body ignores all forces (infinite mass)
//   mass             — mass in kg
//   invMass          — 1/mass, pre-computed to avoid per-step divisions
//   isMovingBySupport — flag set when the body is being carried by the
//                       surface beneath it (see MoveStack in collisions.odin)
RigidBody :: struct {
    force:             Vector3,
    velocity:          Vector3,
    torque:            Vector3,
    angularVelocity:   Vector3,
    bounciness:        f32,
    friction:          f32,
    isStatic:          bool,
    mass:              f32,
    invMass:           f32,
    isMovingBySupport: bool
}

// ---------------------------------------------------------------
// AddForceAtPoint
// Applies a force to a rigid body at a world-space contact point,
// computing both the linear force contribution and the resulting
// torque using the moment-arm relationship:
//
//   τ = r × F
//
// where r is the vector from the body's centre of mass (translation)
// to the contact point.  This is the standard Newtonian torque formula:
// a force applied away from the centre of mass creates a rotational effect
// proportional to the perpendicular distance (lever arm).
//
// Forces and torques are accumulated into the rigid body's accumulators
// and applied during the next integration step.
// ---------------------------------------------------------------
AddForceAtPoint :: proc(model: ^Model, force: Vector3, contactPoint: Vector3) {
    // Accumulate the linear force component.
    model.rigidBody.force += force

    // Compute the moment arm: vector from centre of mass to contact point.
    r := contactPoint - model.translation

    // Torque = r × F  (cross product gives axis and magnitude of rotation tendency).
    torque := Vector3CrossProduct(r, force)
    model.rigidBody.torque += torque
}

// ---------------------------------------------------------------
// ApplyPhysics
// Main per-step physics update: iterates over all models and
// runs gravity, linear integration, and angular integration for
// every non-static body.
//
// Called once per PHYSICS_TIMESTEP from the fixed-step accumulator
// in main.odin.
// ---------------------------------------------------------------
ApplyPhysics :: proc(models: []Model, deltaTime: f32) {
    for &model in models {
        // Static bodies are immovable — skip all integration.
        if model.rigidBody.isStatic do continue

        // 1. Determine if the model is grounded; apply gravity if not.
        //    Also handles friction, stabilisation, tipping, and rolling.
        ApplyGravity(&model, models, deltaTime)

        // 2. Integrate the accumulated linear force into velocity and position.
        IntegrateLinearForce(&model, deltaTime)

        // 3. Integrate the accumulated torque into angular velocity and orientation.
        IntegrateTorque(&model, deltaTime)
    }
}

// ---------------------------------------------------------------
// ApplyGravity
// Determines whether a body is resting on a surface and either
// applies gravity (if in the air) or applies the various resting-
// contact heuristics (friction, stabilisation, tipping, rolling).
//
// Ground detection: the model's translation is temporarily moved
// down by GROUND_PROBE_DIST and a collision test is run against
// every other model.  If a hit is found the model is considered
// supported and gravity is suppressed.
//
// Nested procedures implement the resting-contact behaviours:
//   GetOverhang        — how far the box's centre has moved past
//                        the edge of the supporting surface
//   ApplyFriction      — damps lateral force/torque while resting
//   ApplyRolling       — drives angular velocity from linear velocity
//                        for spheres rolling without slipping
//   ApplyStabilization — torque correction to align the box's
//                        nearest-up axis with the surface normal
//   ApplyTipping       — gravitational tipping torque when the box
//                        is overhanging its support edge
// ---------------------------------------------------------------
ApplyGravity :: proc(model: ^Model, models: []Model, deltaTime: f32) {
    isGrounded := false

    for &other in models {
        // Skip self-collision.
        if &other == model do continue

        // Probe downward: temporarily shift the model slightly below
        // its current position and test for a collision.
        model.translation.y -= GROUND_PROBE_DIST
        probeResult := GetCollisionResult(model, &other)
        model.translation.y += GROUND_PROBE_DIST // Restore position immediately.

        if probeResult.hit {
            if model.colliderType == ColliderType.Box {
                // Damp lateral forces while the box is resting on the surface.
                ApplyFriction(model, other)

                // Apply a corrective torque to keep the box aligned with the surface.
                ApplyStabilization(model, probeResult.normal)

                // Measure how far the box's centre has moved past the support edge.
                overhang     := GetOverhang(model, other)

                // Compute the angle between the surface normal and world-up.
                // acos(|N · up|) gives the tilt of the supporting surface in radians.
                surfaceAngle := math.acos(abs(Vector3DotProduct(probeResult.normal, WORLD_UP)))

                // Consider the box grounded only if it is not overhanging and
                // the surface is nearly flat (within SLIDE_ANGLE_THRESHOLD ≈ 10°).
                if overhang.x <= 1e-6 && overhang.y <= 1e-6 && surfaceAngle < SLIDE_ANGLE_THRESHOLD {
                    isGrounded = true
                }

                // If the box is overhanging and the support surface pushes upward
                // (normal.y <= -0.5 means the contact normal points mostly downward
                // from the contact geometry's perspective — i.e. the box is sitting
                // on top), apply a tipping torque to topple it.
                if (overhang.x > 1e-6 || overhang.y > 1e-6) && probeResult.normal.y <= -0.5 {
                    ApplyTipping(model, probeResult.contactPoint, overhang)
                }
            }

            // Stop checking other models once a support is found.
            break
        }
    }

    // Sphere rolling: update angular velocity from linear velocity
    // regardless of whether grounded (rolling is driven by the velocity
    // even in mid-air to avoid visual pops when landing).
    if model.colliderType == ColliderType.Sphere {
        ApplyRolling(model)
    }

    // If no support was found, apply gravitational acceleration.
    // Using semi-implicit Euler: v += g * dt (velocity first).
    if !isGrounded {
        model.rigidBody.velocity += GRAVITY * deltaTime
    }

    // -----------------------------------------------------------------
    // GetOverhang
    // Returns how far (in the support object's local X and Z axes) the
    // resting model's centre-of-mass projects beyond the support's half-extents.
    //
    // Method:
    //   1. Express the vector from support centre to resting model centre
    //      in the support's local frame by projecting onto its axes.
    //   2. Subtract the support's half-extents.
    //   3. Positive values indicate the centre has gone past the edge.
    //
    // Returns a Vector2 where x = X-axis overhang, y = Z-axis overhang.
    // Values ≤ 0 mean the model is fully supported on that axis.
    // -----------------------------------------------------------------
    GetOverhang :: proc(model: ^Model, other: Model) -> Vector2 {
        // Get the support's local coordinate axes from its rotation matrix.
        otherAxes := GetAxesFromRotationMatrix(other.rotationMatrix)

        // Vector from support centre to resting model centre.
        center := model.translation - other.translation

        // Project onto the support's X and Z axes (horizontal plane).
        projX := abs(Vector3DotProduct(center, otherAxes[0]))
        projZ := abs(Vector3DotProduct(center, otherAxes[2]))

        // Half-extents of the support surface (scaled to world size).
        halfX := other.boxCollider.size.x * other.scale
        halfZ := other.boxCollider.size.z * other.scale

        // Overhang = projection beyond half-extent; negative = fully supported.
        return { projX - halfX, projZ - halfZ }
    }

    // -----------------------------------------------------------------
    // ApplyFriction
    // Damps the lateral force and yaw torque of a resting box.
    // The average friction of the two surfaces in contact is used,
    // matching a simple Coulomb friction model where the combined
    // friction coefficient is the mean of the two materials.
    // -----------------------------------------------------------------
    ApplyFriction :: proc(model: ^Model, other: Model) {
        avgFriction := (model.rigidBody.friction + other.rigidBody.friction) * 0.5
        // Scale down lateral forces (X and Z) — vertical force (Y) is left intact.
        model.rigidBody.force.x *= avgFriction
        model.rigidBody.force.z *= avgFriction
        // Damp yaw rotation torque (spinning on the surface).
        model.rigidBody.torque.y *= avgFriction
    }

    // -----------------------------------------------------------------
    // ApplyRolling
    // Drives a sphere's angular velocity from its linear velocity so
    // it appears to roll without slipping.
    //
    // Rolling without slipping constraint:
    //   ω = (v × up) / r        (axis perpendicular to velocity, in the
    //                             horizontal plane; magnitude = speed / radius)
    //
    // When speed is below the threshold the angular velocity decays
    // quickly (multiplied by 0.1) to avoid jitter from near-zero velocities.
    //
    // isMovingBySupport prevents double-driving angular velocity when
    // the sphere is being carried by another moving surface (MoveStack
    // already sets the angular velocity in that case).
    // -----------------------------------------------------------------
    ApplyRolling :: proc(model: ^Model) {
        speed := Vector3Length(model.rigidBody.velocity)
        if speed > MIN_VELOCITY_THRESHOLD && !model.rigidBody.isMovingBySupport {
            // Rotation axis = world-up × velocity direction (right-hand rule).
            axis   := Vector3CrossProduct(WORLD_UP, model.rigidBody.velocity / speed)
            radius := model.sphereCollider.radius * model.scale
            // Angular speed = linear speed / radius (pure rolling).
            model.rigidBody.angularVelocity = axis * (speed / radius)
        } else {
            // Decay angular velocity rapidly when barely moving.
            model.rigidBody.angularVelocity *= 0.1
        }
    }

    // -----------------------------------------------------------------
    // ApplyStabilization
    // Applies a restoring torque to keep a resting box oriented with
    // one of its faces flat against the surface.
    //
    // Algorithm:
    //   1. Find the local axis of the box that most closely aligns
    //      with the surface normal (FindClosestUpAxis).
    //   2. The cross product of that axis with the surface normal gives
    //      an angular correction vector whose magnitude is proportional
    //      to the angular error (sin of the misalignment angle).
    //   3. Scale by STABILIZATION_STRENGTH and accumulate as torque.
    //
    // This is a proportional (P) controller on orientation error.
    // -----------------------------------------------------------------
    ApplyStabilization :: proc(model: ^Model, normal: Vector3) {
        closestUp  := FindClosestUpAxis(model.rotationMatrix, normal)
        // correction direction and magnitude come from the cross product.
        correction := Vector3CrossProduct(closestUp, normal)
        model.rigidBody.torque += correction * STABILIZATION_STRENGTH
    }

    // -----------------------------------------------------------------
    // ApplyTipping
    // Applies a torque that topples a box when its centre of mass has
    // moved past the edge of its support, simulating the real-world
    // effect of gravity pulling an overhanging object off balance.
    //
    // Algorithm:
    //   1. r = vector from contact point to body centre.
    //   2. gravTorque = r × g  (torque from gravity acting at the centre).
    //   3. Subtract overhang amounts from X/Z components to bias the
    //      torque direction toward the overhanging side.
    //   4. Normalise and scale by mass and TIPPING_STRENGTH.
    // -----------------------------------------------------------------
    ApplyTipping :: proc(model: ^Model, contactPoint: Vector3, overhang: Vector2) {
        // Vector from contact (edge/corner) to the body's centre of mass.
        rContact := model.translation - contactPoint

        // Torque due to gravity: r × g
        gravTorque := Vector3CrossProduct(rContact, GRAVITY)

        // Bias the torque toward the overhanging direction.
        if overhang.x > 1e-6 do gravTorque.x -= overhang.x
        if overhang.y > 1e-6 do gravTorque.z -= overhang.y

        // Normalise and scale to get a smooth, mass-proportional tipping force.
        model.rigidBody.torque += Vector3Normalize(gravTorque) * model.rigidBody.mass * TIPPING_STRENGTH
    }
}

// ---------------------------------------------------------------
// IntegrateLinearForce
// Semi-implicit Euler integration for linear motion.
//
// Steps:
//   1. Convert accumulated force to velocity change:
//      Δv = F * invMass * dt    (Newton's second law: a = F/m)
//   2. Clear the force accumulator for the next step.
//   3. Apply drag:  v *= LINEAR_DRAG  (exponential damping).
//   4. If the resulting speed exceeds the rest threshold,
//      advance the position:  x += v * dt.
//
// The speed threshold check prevents micro-jitter when a body
// should logically be at rest but has tiny residual velocity.
// ---------------------------------------------------------------
IntegrateLinearForce :: proc(model: ^Model, deltaTime: f32) {
    // Apply accumulated force: v += F/m * dt
    model.rigidBody.velocity += model.rigidBody.force * model.rigidBody.invMass * deltaTime

    // Reset force accumulator — forces must be re-applied every frame.
    model.rigidBody.force = {}

    // Apply drag coefficient (different for boxes vs. spheres).
    drag := model.colliderType == ColliderType.Box ? LINEAR_DRAG : SPHERE_LINEAR_DRAG
    model.rigidBody.velocity *= drag

    // Only move the body if it is actually moving (avoids floating-point drift).
    if Vector3Length(model.rigidBody.velocity) > MIN_VELOCITY_THRESHOLD {
        model.translation += model.rigidBody.velocity * deltaTime
    }
}

// ---------------------------------------------------------------
// IntegrateTorque
// Semi-implicit Euler integration for rotational motion.
//
// Steps:
//   1. Convert torque to angular-velocity change:
//      Δω = τ * invMass * dt  (simplified — ignores inertia tensor,
//      using scalar mass as a proxy for rotational inertia)
//   2. Clear the torque accumulator.
//   3. Apply angular drag: ω *= ANGULAR_DRAG.
//   4. If angular speed is above the threshold, construct an
//      incremental axis-angle rotation matrix for this frame's
//      angular displacement (θ = |ω| * dt) and left-multiply it
//      onto the existing rotation matrix to update orientation:
//      R(t+dt) = delta * R(t)
//
// The rotation is applied as matrix multiplication rather than
// Euler-angle addition to avoid gimbal lock.
// ---------------------------------------------------------------
IntegrateTorque :: proc(model: ^Model, deltaTime: f32) {
    // Apply accumulated torque: ω += τ/m * dt
    model.rigidBody.angularVelocity += model.rigidBody.torque * model.rigidBody.invMass * deltaTime

    // Reset torque accumulator.
    model.rigidBody.torque = {}

    // Apply angular drag.
    drag := model.colliderType == ColliderType.Box ? ANGULAR_DRAG : SPHERE_ANGULAR_DRAG
    model.rigidBody.angularVelocity *= drag

    avlength := Vector3Length(model.rigidBody.angularVelocity)
    if avlength > MIN_ANGULAR_VELOCITY_THRESHOLD {
        // Decompose angular velocity into axis (unit vector) and speed (magnitude).
        axis  := model.rigidBody.angularVelocity / avlength

        // Angular displacement for this timestep: θ = |ω| * dt
        angle := avlength * deltaTime

        // Build an incremental rotation matrix for this frame's angular change.
        delta := MakeRotationMatrixAxisAngle(axis, angle)

        // Compose the incremental rotation with the existing rotation:
        //   R_new = delta * R_old
        // Left-multiplication rotates in world space.
        model.rotationMatrix = Mat4Mul(delta, model.rotationMatrix)
    }
}

// ---------------------------------------------------------------
// FindClosestUpAxis
// Given a rigid body's rotation matrix and a reference up vector
// (typically the surface normal), returns the body's local axis that
// most closely points in the same direction as the reference.
//
// Algorithm:
//   1. Extract the three local axes from the rotation matrix.
//   2. For each axis, compute the absolute dot product with the
//      reference vector — maximum |dot| means closest alignment
//      regardless of sign.
//   3. If the closest axis points in the opposite direction
//      (negative dot product), flip it so it always points "up".
//
// This is used by ApplyStabilization to find which face of the box
// is intended to be the "floor-facing" face and then compute the
// angular error to correct.
// ---------------------------------------------------------------
FindClosestUpAxis :: proc(rotationMatrix: Matrix4x4, referenceUp: Vector3) -> Vector3 {
    axes := GetAxesFromRotationMatrix(rotationMatrix)

    // Initialise with the first axis as the best candidate.
    closestAxis := axes[0]
    closestDot  := abs(Vector3DotProduct(axes[0], referenceUp))

    // Compare remaining axes (Y and Z) and keep whichever aligns most
    // closely with the reference direction.
    for i in 1..<3 {
        d := abs(Vector3DotProduct(axes[i], referenceUp))
        if d > closestDot {
            closestDot  = d
            closestAxis = axes[i]
        }
    }

    // Ensure the chosen axis points in the same general direction as
    // the reference rather than opposite (flip if dot product is negative).
    if Vector3DotProduct(closestAxis, referenceUp) < 0 {
        closestAxis = -closestAxis
    }

    return closestAxis
}
