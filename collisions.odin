package main

// ============================================================
// collisions.odin
// Collision detection and resolution for three shape pairs:
//   Box  vs Box    — OBB-OBB using the Separating Axis Theorem (SAT)
//   Box  vs Sphere — Closest-point test on the OBB surface
//   Sphere vs Sphere — Simple distance vs. sum-of-radii test
//
// Resolution is handled by a multi-pass iterative solver
// (SOLVER_ITERATIONS passes per physics step) to improve accuracy
// when multiple bodies are in simultaneous contact (stacking).
// ============================================================

import "core:math"

// ColliderType enumerates the three possible collider states.
// None means the model participates in rendering but not physics collision.
ColliderType :: enum {
    None,   // No collider; model is transparent to collision queries.
    Box,    // Oriented Bounding Box (OBB).
    Sphere  // Sphere.
}

// BoxCollider stores the half-extents of an OBB in local space.
// The box extends ±size.x in local X, ±size.y in local Y, ±size.z in local Z.
// At runtime the extents are multiplied by model.scale to get world-size extents.
BoxCollider :: struct {
    size: Vector3
}

// SphereCollider stores the radius of a sphere in object space.
// World-space radius = radius * model.scale.
SphereCollider :: struct {
    radius: f32
}

// CollisionResult carries everything the solver needs after a collision query.
//
// Fields:
//   hit          — true if the shapes overlap
//   normal       — unit vector pointing from shape B toward shape A
//                  (the direction A must move to separate)
//   depth        — penetration depth (how far the shapes overlap along normal)
//   contactPoint — world-space point of contact, computed as the midpoint
//                  of the two closest surface points
CollisionResult :: struct {
    hit:          bool,
    normal:       Vector3,
    depth:        f32,
    contactPoint: Vector3
}

// ---------------------------------------------------------------
// ResolveCollisions
// Iterative constraint solver that resolves all pairwise overlaps
// in the scene over SOLVER_ITERATIONS passes.
//
// Running multiple iterations allows corrections from one contact
// to propagate to neighbouring contacts — important for stacking.
// A single pass would leave residual penetration when several
// bodies rest on top of each other.
//
// Nested procedures implement the three resolution steps:
//   Correct   — positional correction (pushes overlapping bodies apart)
//   Push      — velocity correction  (reflects velocity along normal)
//   MoveStack — transfers velocity between a support and a resting body
// ---------------------------------------------------------------
ResolveCollisions :: proc(models: []Model) {
    for iter in 0..<SOLVER_ITERATIONS {
        // Test every unique pair (i < j avoids duplicate tests).
        for i in 0..<len(models) {
            for j in i + 1..<len(models) {
                a := &models[i]
                b := &models[j]

                // Two static bodies never need to be resolved against each other.
                if a.rigidBody.isStatic && b.rigidBody.isStatic do continue

                result := GetCollisionResult(a, b)
                if result.hit {
                    // Step 1: separate the overlapping shapes positionally.
                    Correct(a, b, result)
                    // Step 2: propagate support velocity (conveyor-belt effect).
                    MoveStack(a, b, result)
                    // Step 3: reflect velocity components along the collision normal.
                    Push(a, result)
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // Correct
    // Positional correction: pushes overlapping bodies apart along the
    // collision normal by exactly the penetration depth.
    //
    // If both bodies are dynamic they each move half the correction so
    // that the total separation equals the full depth.
    // If only one is dynamic it takes the full correction.
    // Static bodies are never moved.
    // -----------------------------------------------------------------
    Correct :: proc(a, b: ^Model, r: CollisionResult) {
        // The correction vector: normal * max(depth, 0) to avoid negative corrections.
        correction := r.normal * max(r.depth, 0.0)

        if !a.rigidBody.isStatic && !b.rigidBody.isStatic {
            // Both dynamic: share the correction equally.
            a.translation -= correction * 0.5
            b.translation += correction * 0.5
        } else if !a.rigidBody.isStatic {
            // Only A can move: give it the full correction.
            a.translation -= correction
        } else if !b.rigidBody.isStatic {
            // Only B can move.
            b.translation += correction
        }
    }

    // -----------------------------------------------------------------
    // Push
    // Velocity correction along the collision normal using the
    // coefficient of restitution (bounciness).
    //
    // The component of velocity along the contact normal is reflected:
    //   v_new = v - (v · n) * n * bounciness
    //
    // This removes (or reverses) the velocity that was driving the
    // penetration while leaving tangential velocity unchanged.
    // bounciness > 1 adds energy (super-elastic / arcade bounce).
    //
    // Only applied to body A here; B's response comes from a separate
    // Correct call with flipped normal when it is processed as A later,
    // or is handled implicitly through static objects.
    // -----------------------------------------------------------------
    Push :: proc(m: ^Model, r: CollisionResult) {
        if m.rigidBody.isStatic do return

        // Project velocity onto the normal, then remove/reflect that component.
        m.rigidBody.velocity -= Vector3DotProduct(m.rigidBody.velocity, r.normal) * r.normal * m.rigidBody.bounciness
    }

    // -----------------------------------------------------------------
    // MoveStack
    // Transfers velocity from a moving support object to the object
    // resting on top of it — the "conveyor belt" or "carried object" effect.
    //
    // This only activates when the collision normal is mostly vertical
    // (|N · worldUp| ≥ 0.7), meaning the contact is a horizontal support
    // rather than a side collision.
    //
    // Algorithm:
    //   1. Determine which body is the lower support and which is resting.
    //   2. Match the resting body's normal velocity to the support's,
    //      preventing the resting body from sinking into the support.
    //   3. Transfer the support's tangential (horizontal) velocity to the
    //      resting body, scaled by the average friction coefficient.
    //   4. Mirror the support's yaw angular velocity onto the resting body.
    //   5. Set isMovingBySupport = true on the resting body so ApplyRolling
    //      in physics.odin knows not to override the angular velocity.
    // -----------------------------------------------------------------
    MoveStack :: proc(a, b: ^Model, r: CollisionResult) {
        // Reset the support-movement flag for both bodies each iteration.
        a.rigidBody.isMovingBySupport = false
        b.rigidBody.isMovingBySupport = false

        // Only activate for near-horizontal contacts (mostly vertical normal).
        if abs(Vector3DotProduct(r.normal, WORLD_UP)) < 0.7 do return

        // Determine which is on top: if the normal points up (+Y), A is the floor.
        isABottom := Vector3DotProduct(r.normal, WORLD_UP) > 0
        support  := isABottom ? a : b
        resting  := isABottom ? b : a

        // Static resting bodies don't need velocity transfer.
        if resting.rigidBody.isStatic do return

        // Sanity check: the support should be below the resting body.
        if support.translation.y >= resting.translation.y do return

        // Ensure the resting body moves at least as fast upward as the support.
        supportNorm := Vector3DotProduct(support.rigidBody.velocity, r.normal)
        restingNorm := Vector3DotProduct(resting.rigidBody.velocity, r.normal)
        if supportNorm > restingNorm {
            resting.rigidBody.velocity += r.normal * (supportNorm - restingNorm)
        }

        // Only transfer tangential velocity if the support is actually moving.
        supportSpeed := Vector3Length(support.rigidBody.velocity)
        if supportSpeed < MIN_VELOCITY_THRESHOLD do return

        // Compute the tangential (horizontal) components of each body's velocity.
        supportTan := support.rigidBody.velocity - r.normal * supportNorm
        restingTan := resting.rigidBody.velocity - r.normal * restingNorm

        // Average friction determines how much of the support's horizontal
        // motion is transferred to the resting body.
        avgFriction := (support.rigidBody.friction + resting.rigidBody.friction) * 0.5
        resting.rigidBody.velocity += (supportTan - restingTan) * avgFriction

        // Match yaw rotation of the resting body to the support's yaw spin.
        resting.rigidBody.angularVelocity.y = support.rigidBody.angularVelocity.y

        resting.rigidBody.isMovingBySupport = true
    }
}

// ---------------------------------------------------------------
// GetCollisionResult
// Dispatches to the appropriate shape-pair collision test based on
// the collider types of the two models.
//
// For Sphere vs Box, the result is computed as Box vs Sphere and
// then the normal is flipped to maintain the convention that the
// normal points from B toward A.
// ---------------------------------------------------------------
GetCollisionResult :: proc(a, b: ^Model) -> CollisionResult {
    switch {
        case a.colliderType == ColliderType.Box    && b.colliderType == ColliderType.Box:
            return BoxBox(a, b)
        case a.colliderType == ColliderType.Box    && b.colliderType == ColliderType.Sphere:
            return BoxSphere(a, b)
        case a.colliderType == ColliderType.Sphere && b.colliderType == ColliderType.Box:
            // Reuse BoxSphere with swapped arguments; flip normal to restore convention.
            result := BoxSphere(b, a)
            result.normal = -result.normal
            return result
        case a.colliderType == ColliderType.Sphere && b.colliderType == ColliderType.Sphere:
            return SphereSphere(a, b)
    }
    return {}

    // -----------------------------------------------------------------
    // BoxBox (OBB vs OBB) — Separating Axis Theorem (SAT)
    //
    // The Separating Axis Theorem states that two convex shapes do NOT
    // overlap if and only if there exists at least one axis along which
    // their projections do not overlap (a "separating axis").
    // Conversely, if all candidate axes show overlap, the shapes must
    // be intersecting.
    //
    // For two OBBs the candidate axes are:
    //   • 3 face normals of OBB A  (axesA[0..2])
    //   • 3 face normals of OBB B  (axesB[0..2])
    //   • 9 cross products of each axis pair  (axesA[i] × axesB[j])
    // → 15 axes total.  The cross products detect edge-edge collisions
    // that the face-normal axes would miss.
    //
    // For each axis:
    //   rA = projection radius of OBB A onto the axis
    //   rB = projection radius of OBB B onto the axis
    //   centerProj = projection of the centre-to-centre vector onto axis
    //   overlap = rA + rB - centerProj
    // If overlap ≤ 0 on ANY axis → no collision.
    // The axis with the smallest positive overlap is the Minimum
    // Translation Vector (MTV), used as the collision normal and depth.
    // -----------------------------------------------------------------
    BoxBox :: proc(a, b: ^Model) -> CollisionResult {
        // Extract the three local axes (columns) from each OBB's rotation matrix.
        axesA := GetAxesFromRotationMatrix(a.rotationMatrix)
        axesB := GetAxesFromRotationMatrix(b.rotationMatrix)

        // World-space half-extents: object-space size * uniform scale.
        colSizeA := a.boxCollider.size * a.scale
        colSizeB := b.boxCollider.size * b.scale

        // Vector from centre of A to centre of B.
        centerDiff := b.translation - a.translation

        // Build the 15 candidate separating axes.
        axes: [15]Vector3
        axes[0] = axesA[0]  // A local X
        axes[1] = axesA[1]  // A local Y
        axes[2] = axesA[2]  // A local Z
        axes[3] = axesB[0]  // B local X
        axes[4] = axesB[1]  // B local Y
        axes[5] = axesB[2]  // B local Z

        // Cross-product edge-edge axes: 3×3 = 9 axes.
        idx := 6
        for i in 0..<3 {
            for j in 0..<3 {
                axes[idx] = Vector3CrossProduct(axesA[i], axesB[j])
                idx += 1
            }
        }

        // Track the axis with the minimum overlap (smallest penetration depth).
        minDepth  : f32 = max(f32)
        minNormal : Vector3

        for axis in axes {
            // Skip degenerate axes (near-zero length from parallel edge pairs).
            if Vector3Length(axis) < 1e-6 do continue

            normalizedAxis := Vector3Normalize(axis)

            // Project each OBB's half-extents onto the test axis.
            rA := ProjectRadius(colSizeA, axesA, normalizedAxis)
            rB := ProjectRadius(colSizeB, axesB, normalizedAxis)

            // Project the centre-to-centre offset onto the axis.
            centerProj := abs(Vector3DotProduct(centerDiff, normalizedAxis))

            // Overlap = sum of projected radii minus projected centre distance.
            overlap := rA + rB - centerProj

            // Found a separating axis — no collision.
            if overlap <= 0 do return CollisionResult{hit = false}

            // Track the axis with minimum penetration (MTV).
            if overlap < minDepth {
                minDepth  = overlap
                minNormal = normalizedAxis
            }
        }

        // Ensure the normal points from A toward B (away from A).
        if Vector3DotProduct(centerDiff, minNormal) < 0 {
            minNormal = -minNormal
        }

        // Compute the contact point as the midpoint of the two closest
        // surface points (one on each OBB).
        cA := FindClosestPoint(b.translation, a.translation, axesA, colSizeA)
        cB := FindClosestPoint(a.translation, b.translation, axesB, colSizeB)
        contactPoint := (cA + cB) * 0.5

        return CollisionResult{
            hit          = true,
            normal       = minNormal,
            depth        = minDepth,
            contactPoint = contactPoint
        }

        // ProjectRadius
        // Computes the "support radius" of an OBB projected onto a given axis.
        // This is the half-length of the OBB's shadow on that axis:
        //   r = |e1·axis|*h1 + |e2·axis|*h2 + |e3·axis|*h3
        // where e1/e2/e3 are the OBB's local axes and h1/h2/h3 are its half-extents.
        ProjectRadius :: proc(colSize: Vector3, axes: [3]Vector3, axis: Vector3) -> f32 {
            return colSize.x * abs(Vector3DotProduct(axes[0], axis)) +
                   colSize.y * abs(Vector3DotProduct(axes[1], axis)) +
                   colSize.z * abs(Vector3DotProduct(axes[2], axis))
        }

        // FindClosestPoint
        // Finds the closest point on an OBB surface to an external point.
        // Algorithm:
        //   1. Compute the vector from the OBB centre to the query point.
        //   2. Project it onto each local axis and clamp to ±half-extent.
        //   3. Sum up the clamped contributions to get the closest surface point.
        FindClosestPoint :: proc(point: Vector3, center: Vector3, axes: [3]Vector3, size: Vector3) -> Vector3 {
            d := point - center
            result := center

            for i in 0..<3 {
                // Project the offset onto this axis.
                dist := Vector3DotProduct(d, axes[i])
                // Clamp to within the OBB's extent along this axis.
                dist = math.clamp(dist, -size[i], size[i])
                // Add the clamped contribution to the closest point.
                result += axes[i] * dist
            }

            return result
        }
    }

    // -----------------------------------------------------------------
    // BoxSphere
    // Detects a collision between an OBB (a) and a sphere (b).
    //
    // Algorithm:
    //   1. Express the vector from the box centre to the sphere centre
    //      in the box's local frame.
    //   2. Clamp each component to the box's half-extents to find the
    //      closest point on the box surface to the sphere centre.
    //   3. If the distance from that closest point to the sphere centre
    //      is less than the sphere's radius, the shapes overlap.
    //   4. The collision normal points from the closest-point toward
    //      the sphere centre; depth is radius − distance.
    // -----------------------------------------------------------------
    BoxSphere :: proc(a, b: ^Model) -> CollisionResult {
        axes   := GetAxesFromRotationMatrix(a.rotationMatrix)
        size   := a.boxCollider.size * a.scale
        radius := b.sphereCollider.radius * b.scale

        // Vector from box centre to sphere centre.
        dirAB := b.translation - a.translation

        // Find the closest point on the box surface to the sphere centre
        // by clamping the projection of dirAB onto each local axis.
        closestPoint := a.translation
        for i in 0..<3 {
            dist := Vector3DotProduct(dirAB, axes[i])
            dist = math.clamp(dist, -size[i], size[i])
            closestPoint += axes[i] * dist
        }

        // Vector from closest point to sphere centre.
        dirCpB := b.translation - closestPoint
        dist   := Vector3Length(dirCpB)

        // No collision if the closest point is outside the sphere.
        if dist >= radius do return CollisionResult{hit = false}

        // Collision normal: from closest point toward sphere centre.
        // If dist ≈ 0 the sphere centre is inside the box; default to world up.
        normal := WORLD_UP
        if dist > 1e-6 {
            normal = dirCpB / dist
        }

        return CollisionResult{
            hit          = true,
            normal       = normal,
            depth        = radius - dist,
            contactPoint = closestPoint,
        }
    }

    // -----------------------------------------------------------------
    // SphereSphere
    // Detects a collision between two spheres.
    //
    // Two spheres overlap when the distance between their centres is
    // less than the sum of their radii:
    //   dist < rA + rB
    //
    // Collision normal = unit vector from A centre toward B centre.
    // Penetration depth = (rA + rB) − dist.
    // Contact point = surface of A along the normal direction.
    // -----------------------------------------------------------------
    SphereSphere :: proc(a, b: ^Model) -> CollisionResult {
        rA := a.sphereCollider.radius * a.scale
        rB := b.sphereCollider.radius * b.scale

        // Vector from A centre to B centre.
        diff := b.translation - a.translation
        dist := Vector3Length(diff)
        sum  := rA + rB

        // No collision if centres are farther apart than the sum of radii.
        if dist >= sum do return CollisionResult{hit = false}

        // Collision normal: from A toward B (or world-up for coincident centres).
        normal := WORLD_UP
        if dist > 1e-6 {
            normal = diff / dist
        }

        return CollisionResult{
            hit          = true,
            normal       = normal,
            depth        = sum - dist,
            // Contact point is the point on A's surface closest to B.
            contactPoint = a.translation + normal * rA,
        }
    }
}
