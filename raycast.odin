package main

// ============================================================
// raycast.odin
// Mouse-picking via ray casting against scene models.
//
// A ray is cast from the camera through a screen-space pixel and
// tested against each model's collider.  The closest hit is returned.
//
// Supported intersection tests:
//   OBB  — Slab method (also known as the Kay-Kajiya algorithm):
//           intersect the ray with three pairs of parallel planes.
//   Sphere — Analytic quadratic formula: solve |rayOrigin + t*rayDir - centre|² = r².
//
// Both perspective and orthographic projections are supported.
// Their only difference is how the ray origin and direction are
// computed from the screen-space pixel position.
// ============================================================

import "core:math"

// RayHit stores the result of a ray cast against the scene.
//
// Fields:
//   hit       — true if the ray intersected any collider
//   model     — pointer to the closest intersected model (nil if no hit)
//   position  — world-space hit point on the collider surface
//   direction — unit direction of the ray (same for all models in one cast)
RayHit :: struct {
    hit:       bool,
    model:     ^Model,
    position:  Vector3,
    direction: Vector3
}

// ---------------------------------------------------------------
// CastRay
// Casts a ray from the camera through a screen-space pixel coordinate
// and returns the closest intersection with any model's collider.
//
// High-level steps:
//   1. Convert the pixel (screenX, screenY) to NDC coordinates:
//        ndcX = (screenX / SCREEN_WIDTH) * 2 - 1   → [-1, +1]
//        ndcY = (screenY / SCREEN_HEIGHT) * 2 - 1  → [-1, +1]
//   2. Compute the ray origin and direction for the active projection type.
//   3. Test the ray against every model's collider; keep the closest hit.
//
// Parameters:
//   screenX/screenY — pixel coordinates of the mouse cursor
//   camera          — view parameters (position, forward, right, up)
//   projType        — Perspective or Orthographic
//   models          — the full list of scene objects
// ---------------------------------------------------------------
CastRay :: proc(screenX, screenY: f32, camera: Camera, projType: ProjectionType, models: []Model) -> RayHit {
    // Step 1: convert pixel position to NDC.
    // NDC (Normalised Device Coordinates): X and Y in [-1, +1].
    ndcX := (screenX / f32(SCREEN_WIDTH))  * 2.0 - 1.0
    ndcY := (screenY / f32(SCREEN_HEIGHT)) * 2.0 - 1.0

    // Step 2: build the ray for this projection type.
    rayOrigin := GetRayOrigin(ndcX, ndcY, camera, projType)
    rayDir    := GetRayDirection(ndcX, ndcY, camera, projType)

    hit:         RayHit
    closestDist: f32 = max(f32)

    // Step 3: test the ray against every model.
    for &model in models {
        center := model.translation
        // Vector from ray origin to the collider's centre.
        delta := center - rayOrigin

        if model.colliderType == ColliderType.Box {
            // ---------------------------------------------------------
            // OBB–Ray intersection: the Slab (Kay-Kajiya) method.
            //
            // An OBB can be described as the intersection of three "slabs"
            // (pairs of parallel planes).  The ray interval [tMin, tMax]
            // that lies inside ALL three slabs is the range of t values
            // for which the ray is inside the OBB.
            //
            // For each local axis i:
            //   e = projection of delta onto axis  (centre offset)
            //   f = projection of rayDir onto axis (directional component)
            //
            //   If |f| > ε (ray not parallel to slab):
            //     t1 = (e - halfExtent) / f   ← entry plane
            //     t2 = (e + halfExtent) / f   ← exit plane
            //     tMin = max(tMin, min(t1, t2))
            //     tMax = min(tMax, max(t1, t2))
            //
            //   If |f| ≤ ε (ray parallel to slab):
            //     if the origin is outside the slab → no intersection.
            //
            // If tMin > tMax at any point → no intersection.
            // If tMax < 0 → intersection is behind the ray origin.
            // ---------------------------------------------------------
            axes  := GetAxesFromRotationMatrix(model.rotationMatrix)
            size  := model.boxCollider.size * model.scale

            tMin     : f32 = -max(f32)
            tMax     : f32 =  max(f32)
            modelHit := true

            for i in 0..<3 {
                axis := axes[i]

                // Project the centre-offset and ray direction onto this axis.
                e := Vector3DotProduct(axis, delta)
                f := Vector3DotProduct(axis, rayDir)

                if abs(f) > 1e-6 {
                    // Ray is not parallel to this slab — compute entry/exit t values.
                    t1 := (e - size[i]) / f
                    t2 := (e + size[i]) / f

                    // Ensure t1 < t2.
                    if t1 > t2 {
                        t1, t2 = t2, t1
                    }

                    // Narrow the [tMin, tMax] interval to the intersection of all slabs.
                    tMin = max(tMin, t1)
                    tMax = min(tMax, t2)

                    // If the intervals no longer overlap, or the OBB is fully behind
                    // the ray origin, this axis is a separator — no intersection.
                    if tMin > tMax || tMax < 0 {
                        modelHit = false
                        continue
                    }
                } else {
                    // Ray is parallel to the slab planes.
                    // Check whether the origin is between the planes:
                    //   -e - size → plane 1 distance;  -e + size → plane 2 distance.
                    // If the origin is outside both planes, there is no intersection.
                    if -e - size[i] > 0 || -e + size[i] < 0 {
                        modelHit = false
                        continue
                    }
                }
            }

            // Record the hit if this OBB is the closest so far.
            if modelHit && tMin < closestDist {
                closestDist   = tMin
                hit.hit       = true
                hit.model     = &model
                // Hit position: travel along the ray to the entry point.
                hit.position  = rayOrigin + rayDir * tMin
                hit.direction = rayDir
            }

        } else if model.colliderType == ColliderType.Sphere {
            // ---------------------------------------------------------
            // Sphere–Ray intersection: analytic quadratic method.
            //
            // A ray is parameterised as P(t) = rayOrigin + t * rayDir.
            // The sphere centred at C with radius r satisfies |P - C|² = r².
            // Substituting and expanding:
            //   |rayOrigin + t*rayDir - C|² = r²
            //   (delta = C - rayOrigin)
            // Let  tca = delta · rayDir   (distance along ray to closest approach)
            //      d²  = |delta|² - tca²  (squared distance of closest approach)
            //
            // If d² > r²  → ray misses the sphere.
            // Otherwise:
            //   thc = sqrt(r² - d²)     (half-chord length through sphere)
            //   t1 = tca - thc           (entry intersection)
            //   t2 = tca + thc           (exit intersection)
            //   t  = t1 < 0 ? t2 : t1   (use exit if we are inside sphere)
            // ---------------------------------------------------------
            r  := model.sphereCollider.radius * model.scale
            r2 := r * r

            // Projection of the centre-offset onto the ray: closest approach distance.
            tca := Vector3DotProduct(delta, rayDir)

            // Squared perpendicular distance from the sphere centre to the ray.
            d2 := Vector3DotProduct(delta, delta) - tca * tca

            // Ray misses the sphere if closest approach is farther than radius.
            if d2 > r2 {
                continue
            }

            // Half-chord through the sphere.
            thc := math.sqrt(r2 - d2)
            t1  := tca - thc  // Entry
            t2  := tca + thc  // Exit

            // If t1 is behind the origin, use t2 (we are inside the sphere).
            t := t1 < 0 ? t2 : t1

            // Record the closest hit in front of the ray origin.
            if t >= 0 && t < closestDist {
                closestDist   = t
                hit.hit       = true
                hit.model     = &model
                hit.position  = rayOrigin + rayDir * t
                hit.direction = rayDir
            }
        }
    }

    return hit

    // -----------------------------------------------------------------
    // GetRayOrigin
    // Returns the world-space starting point of the ray for the given
    // projection type.
    //
    // Perspective: all rays originate at the camera eye position.
    //
    // Orthographic: rays are parallel and their origin is offset from
    //   the camera position within the near plane, based on the NDC
    //   coordinates of the pixel.  The offset is scaled by the aspect
    //   ratio to match the orthographic view volume.
    // -----------------------------------------------------------------
    GetRayOrigin :: proc(ndcX, ndcY: f32, camera: Camera, projType: ProjectionType) -> Vector3 {
        if projType == .Orthographic {
            // For orthographic projection, the ray origin lies in the
            // near plane at the corresponding world-space position.
            aspect := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
            return camera.position +
                   camera.right * (ndcX * aspect) +
                   camera.up   * (-ndcY)
        }

        // For perspective, all rays converge at the eye (camera position).
        return camera.position
    }

    // -----------------------------------------------------------------
    // GetRayDirection
    // Returns the world-space unit direction of the ray for the given
    // projection type.
    //
    // Perspective:
    //   The ray direction is computed by adding horizontal and vertical
    //   offsets (scaled by NDC and the tangent of half-FOV) to the
    //   camera forward vector.  This reconstructs the view-space ray
    //   direction and transforms it into world space via the camera basis.
    //
    //   The tangent of half-FOV maps NDC values to the physical size
    //   of the near plane in view space:
    //     dirX = ndcX * aspect * tan(FOV/2)   → horizontal offset
    //     dirY = -ndcY * tan(FOV/2)            → vertical offset (Y flipped)
    //
    // Orthographic:
    //   All rays share the same direction (the camera's forward vector)
    //   since there is no perspective convergence.
    // -----------------------------------------------------------------
    GetRayDirection :: proc(ndcX, ndcY: f32, camera: Camera, projType: ProjectionType) -> Vector3 {
        if projType == .Orthographic {
            // Parallel projection: all rays go in the same direction.
            return camera.forward
        }

        // Perspective: reconstruct the view-space ray from FOV and NDC.
        aspect      := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
        tanHalfFov  := math.tan_f32(FOV * 0.5 * DEG_TO_RAD)

        // Combine forward direction with the lateral and vertical offsets.
        return Vector3Normalize (
            camera.forward +
            camera.right * (ndcX * aspect * tanHalfFov) +
            camera.up    * (-ndcY * tanHalfFov)  // NDC Y is flipped vs. screen Y
        )
    }
}
