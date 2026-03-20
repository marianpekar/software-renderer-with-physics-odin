package main

import rl "vendor:raylib"

HandleInputs :: proc(
    model: ^^Model,
    models: []Model, camera: Camera,
    renderMode: ^i8, renderModesCount: i8,
    projType: ^ProjectionType,
    deltaTime: f32
) {
    linearForce: f32 = (rl.IsKeyDown(rl.KeyboardKey.LEFT_SHIFT) ? 150 : 75) * deltaTime
    if rl.IsKeyDown(rl.KeyboardKey.W) do model^.rigidBody.acceleration.z += linearForce
    if rl.IsKeyDown(rl.KeyboardKey.S) do model^.rigidBody.acceleration.z -= linearForce
    if rl.IsKeyDown(rl.KeyboardKey.A) do model^.rigidBody.acceleration.x += linearForce
    if rl.IsKeyDown(rl.KeyboardKey.D) do model^.rigidBody.acceleration.x -= linearForce

    linearForceUp: f32 = (rl.IsKeyDown(rl.KeyboardKey.LEFT_SHIFT) ? 2000 : 1000) * deltaTime
    if rl.IsKeyDown(rl.KeyboardKey.SPACE) do model^.rigidBody.acceleration.y += linearForceUp

    linearStep: f32 = (rl.IsKeyDown(rl.KeyboardKey.LEFT_SHIFT) ? 0.25 : 1) * deltaTime
    if rl.IsKeyDown(rl.KeyboardKey.KP_ADD) do model^.scale += linearStep
    if rl.IsKeyDown(rl.KeyboardKey.KP_SUBTRACT) do model^.scale -= linearStep

    if rl.IsKeyPressed(rl.KeyboardKey.LEFT) {
        renderMode^ = (renderMode^ + renderModesCount - 1) % renderModesCount
    } else if rl.IsKeyPressed(rl.KeyboardKey.RIGHT) {
        renderMode^ = (renderMode^ + 1) % renderModesCount
    }

    if rl.IsKeyPressed(rl.KeyboardKey.KP_0) {
        projType^ = .Perspective
    }
    if rl.IsKeyPressed(rl.KeyboardKey.KP_1) {
        projType^ = .Orthographic
    }

    pushForce: f32 = (rl.IsKeyDown(rl.KeyboardKey.LEFT_SHIFT) ? 360 : 180) * deltaTime
    poking := false
    freezing := false
    if rl.IsMouseButtonDown(rl.MouseButton.LEFT) {
        poking = true
    } else if rl.IsMouseButtonDown(rl.MouseButton.RIGHT) {
        poking = true
        pushForce = -pushForce
    } else if rl.IsMouseButtonPressed(rl.MouseButton.MIDDLE) {
        freezing = true
    }

    if poking || freezing {
        rayHit := CastRay(f32(rl.GetMouseX()), f32(rl.GetMouseY()), camera, models)
        if rayHit.hit {
            model^ = rayHit.model

            if poking {
                ApplyForceAtPoint(model^, rayHit.direction * pushForce, rayHit.position)
            }
            else if freezing {
                model^.rigidBody.isStatic = !model^.rigidBody.isStatic
            }
        }
    }
}