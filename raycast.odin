package main

import "core:math"

Ray :: struct {
    hit: bool,
    model: ^Model,
    position: Vector3,
    direction: Vector3
}

CastRay :: proc(screenX, screenY: f32, camera: Camera, projType: ProjectionType, models: []Model) -> Ray {
    ndcX := (screenX / f32(SCREEN_WIDTH)) * 2.0 - 1.0
    ndcY := (screenY / f32(SCREEN_HEIGHT)) * 2.0 - 1.0

    rayOrigin := GetRayOrigin(ndcX, ndcY, camera, projType)

    ray: Ray
    ray.direction = GetRayDirection(ndcX, ndcY, camera, projType)

    closestDist := max(f32)

    for &model in models {
        center := model.translation
        delta := center - rayOrigin

        if HasBoxCollider(&model) {
            axes := GetAxesFromRotationMatrix(model.rotationMatrix)
            size := model.collider.(BoxCollider) * model.scale

            tMin := min(f32)
            tMax := max(f32)
            hit := true

            for i in 0..<3 {
                axis := axes[i]
                e := Vector3DotProduct(axis, delta)
                f := Vector3DotProduct(axis, ray.direction)
    
                t1 := (e - size[i]) / f
                t2 := (e + size[i]) / f

                if t1 > t2 {
                    t1, t2 = t2, t1 
                }

                tMin = max(tMin, t1)
                tMax = min(tMax, t2)

                if tMin > tMax {
                    hit = false
                    break
                }
            }

            if hit && tMin < closestDist {
                closestDist = tMin
                ray.hit = true
                ray.model = &model
                ray.position = rayOrigin + ray.direction * tMin
            }

        } 
        else if HasSphereCollider(&model) {
            r := model.collider.(SphereCollider) * model.scale
            r2 := r * r
            tca := Vector3DotProduct(delta, ray.direction)
            d2 := Vector3DotProduct(delta, delta) - tca * tca
            
            if d2 > r2 {
                continue
            }

            thc := math.sqrt(r2 - d2)
            t1 := tca - thc
            t2 := tca + thc
            t := t1 < 0 ? t2 : t1

            if t >= 0 && t < closestDist {
                closestDist = t
                ray.hit = true
                ray.model = &model
                ray.position = rayOrigin + ray.direction * t
            }
        }
    }

    return ray

    GetRayOrigin :: proc(ndcX, ndcY: f32, camera: Camera, projType: ProjectionType) -> Vector3 {
        if projType == .Orthographic {
            aspect := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
            return camera.position + camera.right * (ndcX * aspect) + camera.up * (-ndcY)
        }
        
        return camera.position
    }

    GetRayDirection :: proc(ndcX, ndcY: f32, camera: Camera, projType: ProjectionType) -> Vector3 {
        if projType == .Orthographic {
            return camera.forward
        }

        aspect := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
        tanHalfFov := math.tan_f32(FOV * 0.5 * DEG_TO_RAD)

        return Vector3Normalize (
            camera.forward +
            camera.right * (ndcX * aspect * tanHalfFov) +
            camera.up * (-ndcY * tanHalfFov)
        )
    }
}