package main

Camera :: struct {
    position: Vector3,
    target: Vector3,
    forward: Vector3,
    right: Vector3,
    up: Vector3
}

MakeCamera :: proc(position, target: Vector3) -> Camera {
    forward := Vector3Normalize(target - position)
    right := Vector3Normalize(Vector3CrossProduct(forward, WORLD_UP))

    return Camera {
        position = position,
        target = target,
        forward = forward,
        right = right,
        up = Vector3CrossProduct(right, forward)
    }
}