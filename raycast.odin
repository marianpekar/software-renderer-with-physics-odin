package main

import "core:math"

RayHit :: struct {
    hit: bool,
    model: ^Model,
    position: Vector3,
    direction: Vector3
}

CastRay :: proc(screenX, screenY: f32, camera: Camera, models: []Model) -> RayHit {
    rayDir := ScreenToWorldDirection(screenX, screenY, camera)
    hit: RayHit
    closestDist := max(f32)

    for &model in models {
        center := model.translation
        delta := center - camera.position

        if model.colliderType == ColliderType.Box {
            axes := GetAxesFromRotationMatrix(model.rotationMatrix)
            size := model.boxCollider.size * model.scale

            tMin := -max(f32)
            tMax :=  max(f32)
            modelHit := true

            for i in 0..<3 {
                axis := axes[i]
                e := Vector3DotProduct(axis, delta)
                f := Vector3DotProduct(axis, rayDir)

                if abs(f) > 1e-6 {
                    t1 := (e - size[i]) / f
                    t2 := (e + size[i]) / f

                    if t1 > t2 {
                        t1, t2 = t2, t1 
                    }

                    tMin = max(tMin, t1)
                    tMax = min(tMax, t2)

                    if tMin > tMax || tMax < 0 {
                        modelHit = false
                        continue
                    }
                } else {
                    if -e - size[i] > 0 || -e + size[i] < 0 {
                        modelHit = false
                        continue
                    }
                }
            }

            if modelHit && tMin < closestDist {
                closestDist = tMin
                hit.hit = true
                hit.model = &model
                hit.position = camera.position + rayDir * tMin
                hit.direction = rayDir
            }

        } 
        else if model.colliderType == ColliderType.Sphere {
            r := model.sphereCollider.radius * model.scale
            r2 := r * r
            tca := Vector3DotProduct(delta, rayDir)
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
                hit.hit = true
                hit.model = &model
                hit.position = camera.position + rayDir * t
                hit.direction = rayDir
            }
        }
    }

    return hit
}

ScreenToWorldDirection :: proc(screenX, screenY: f32, camera: Camera) -> Vector3 {
    ndcX := (screenX / f32(SCREEN_WIDTH)) * 2.0 - 1.0
    ndcY := (screenY / f32(SCREEN_HEIGHT)) * 2.0 - 1.0

    aspect := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
    tanHalfFov := math.tan_f32(FOV * 0.5 * DEG_TO_RAD)

    return Vector3Normalize (
        camera.forward + 
        camera.right * (ndcX * aspect * tanHalfFov) + 
        camera.up * (-ndcY * tanHalfFov)
    )
}