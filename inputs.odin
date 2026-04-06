package main

// ============================================================
// inputs.odin
// Player input handling: movement forces, render-mode cycling,
// projection-type switching, and mouse-based model interaction.
//
// All force magnitudes are scaled by deltaTime so that behaviour
// is frame-rate independent.  An optional LEFT_SHIFT modifier
// doubles most force values for faster/stronger interaction.
// ============================================================

import rl "vendor:raylib"

// HandleInputs reads keyboard and mouse state for one frame and
// applies the appropriate effects to the selected model and
// renderer settings.
//
// Parameters:
//   model            — double pointer to the currently selected model;
//                      reassigned when the user clicks a different object
//   models           — the full scene slice, needed for ray casting
//   camera           — current camera (used to build pick rays)
//   renderMode       — index of the active render mode (0–7); cycled by arrow keys
//   renderModesCount — total number of render modes available
//   projType         — current projection type (Perspective / Orthographic);
//                      switched by numpad keys
//   deltaTime        — elapsed time for this frame (seconds)
//
// Keybindings summary:
//   W/A/S/D           — apply linear force along Z/X axes (movement)
//   SPACE             — apply upward force (jump/throw)
//   KP_ADD/KP_SUB     — increase / decrease model scale
//   LEFT/RIGHT arrow  — cycle render mode backward / forward
//   KP_0              — switch to perspective projection
//   KP_1              — switch to orthographic projection
//   LEFT_SHIFT        — hold to double most force magnitudes
//   Mouse LEFT        — push the picked object away from camera
//   Mouse RIGHT       — pull the picked object toward camera
//   Mouse MIDDLE      — toggle the picked object between static and dynamic
HandleInputs :: proc(
    model:            ^^Model,
    models:           []Model,
    camera:           Camera,
    renderMode:       ^i8,
    renderModesCount: i8,
    projType:         ^ProjectionType,
    deltaTime:        f32
) {
    // Base linear force magnitude, doubled when SHIFT is held.
    // Multiplied by deltaTime to give frame-rate-independent impulses.
    linearForce: f32 = (rl.IsKeyDown(rl.KeyboardKey.LEFT_SHIFT) ? 150 : 75) * deltaTime

    // WASD movement: apply force along the world Z and X axes.
    // Note: W increases +Z, S decreases Z, A increases +X, D decreases X.
    // This is a simplification that ignores the camera's facing direction.
    if rl.IsKeyDown(rl.KeyboardKey.W) do model^.rigidBody.force.z += linearForce
    if rl.IsKeyDown(rl.KeyboardKey.S) do model^.rigidBody.force.z -= linearForce
    if rl.IsKeyDown(rl.KeyboardKey.A) do model^.rigidBody.force.x += linearForce
    if rl.IsKeyDown(rl.KeyboardKey.D) do model^.rigidBody.force.x -= linearForce

    // SPACE: apply a strong upward impulse, with SHIFT for an even stronger jump.
    linearForceUp: f32 = (rl.IsKeyDown(rl.KeyboardKey.LEFT_SHIFT) ? 2000 : 1000) * deltaTime
    if rl.IsKeyDown(rl.KeyboardKey.SPACE) do model^.rigidBody.force.y += linearForceUp

    // Numpad +/-: adjust the uniform scale of the selected model.
    // SHIFT gives finer / coarser increments.
    linearStep: f32 = (rl.IsKeyDown(rl.KeyboardKey.LEFT_SHIFT) ? 0.25 : 1) * deltaTime
    if rl.IsKeyDown(rl.KeyboardKey.KP_ADD)      do model^.scale += linearStep
    if rl.IsKeyDown(rl.KeyboardKey.KP_SUBTRACT) do model^.scale -= linearStep

    // LEFT/RIGHT arrow: cycle through render modes (0 = wireframe no cull,
    // 1 = wireframe cull, 2 = unlit, 3 = flat-shaded, 4 = Phong,
    // 5 = textured unlit, 6 = textured flat, 7 = textured Phong).
    // Modular arithmetic wraps around at both ends.
    if rl.IsKeyPressed(rl.KeyboardKey.LEFT) {
        // Wrap backward: (current + count - 1) % count avoids negative modulo.
        renderMode^ = (renderMode^ + renderModesCount - 1) % renderModesCount
    } else if rl.IsKeyPressed(rl.KeyboardKey.RIGHT) {
        renderMode^ = (renderMode^ + 1) % renderModesCount
    }

    // Numpad 0/1: switch between perspective and orthographic projection.
    if rl.IsKeyPressed(rl.KeyboardKey.KP_0) {
        projType^ = .Perspective
    }
    if rl.IsKeyPressed(rl.KeyboardKey.KP_1) {
        projType^ = .Orthographic
    }

    // Mouse interaction: cast a ray from the camera through the cursor
    // and interact with whatever model is hit.
    //
    // Push force magnitude, with SHIFT for a stronger push/pull.
    pushForce: f32 = (rl.IsKeyDown(rl.KeyboardKey.LEFT_SHIFT) ? 360 : 180) * deltaTime

    poking   := false  // true when the user wants to push or pull
    freezing := false  // true when the user wants to toggle isStatic

    if rl.IsMouseButtonDown(rl.MouseButton.LEFT) {
        // Left button: push the object away (positive force along ray direction).
        poking = true
    } else if rl.IsMouseButtonDown(rl.MouseButton.RIGHT) {
        // Right button: pull the object toward the camera (negative force).
        poking    = true
        pushForce = -pushForce
    } else if rl.IsMouseButtonPressed(rl.MouseButton.MIDDLE) {
        // Middle button press (single-frame): toggle static/dynamic state.
        freezing = true
    }

    if poking || freezing {
        // Cast a ray from the camera through the mouse cursor position.
        rayHit := CastRay(f32(rl.GetMouseX()), f32(rl.GetMouseY()), camera, projType^, models)

        if rayHit.hit {
            // The clicked model becomes the new selected model.
            model^ = rayHit.model

            if poking {
                // Apply an impulse at the hit point, offset from the centre
                // of mass, so that hitting off-centre creates torque as well
                // as linear velocity (see AddForceAtPoint in physics.odin).
                AddForceAtPoint(model^, rayHit.direction * pushForce, rayHit.position)
            } else if freezing {
                // Toggle: static ↔ dynamic.  Frozen objects are immovable until
                // the user middle-clicks them again.
                model^.rigidBody.isStatic = !model^.rigidBody.isStatic
            }
        }
    }
}
