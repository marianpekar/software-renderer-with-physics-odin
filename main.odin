package main

// ============================================================
// main.odin
// Application entry point and main loop.
//
// Responsibilities:
//   • Window initialisation (via Raylib)
//   • Scene setup: models, lights, camera, projection matrices
//   • Fixed-timestep physics loop (accumulator pattern)
//   • Render-mode dispatch: 8 progressively richer shading modes
//   • Input handling delegation
//   • Software back-buffer management (draw to CPU image, upload
//     to GPU texture, then blit)
//
// Render modes (cycled with LEFT/RIGHT arrow keys):
//   0 — Wireframe (no back-face culling)
//   1 — Wireframe (with back-face culling)
//   2 — Flat colour fill (unlit)
//   3 — Flat shading (Lambertian, per-face)
//   4 — Phong shading (per-pixel diffuse, colour mesh)
//   5 — Textured, unlit
//   6 — Textured + flat shading
//   7 — Textured + Phong shading
//
// Projection types (KP_0 / KP_1):
//   Perspective  — standard pinhole camera projection
//   Orthographic — parallel projection (no depth foreshortening)
// ============================================================

import rl "vendor:raylib"

// ProjectionType enumerates the two projection modes supported
// by the renderer and the ray caster.
ProjectionType :: enum {
    Perspective,   // Pinhole/perspective divide; distant objects appear smaller.
    Orthographic,  // Parallel projection; object size is independent of distance.
}

// main is the application entry point.
main :: proc() {
    // --- Window ---
    // Create an OS window of the configured pixel dimensions.
    rl.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Renderer")

    // -------------------------------------------------------
    // Scene setup
    // Create all models, attach rigid bodies and colliders,
    // then position and scale them into the initial layout.
    // -------------------------------------------------------

    // Small yellow sphere — light, very bouncy (restitution 1.9).
    sphereS := LoadModel("assets/sphere.obj", "assets/asphalt.png", rl.YELLOW)
    AddRigidbody(&sphereS, isStatic = false, bounciness = 1.9, mass = 1.0)
    AddSphereCollider(&sphereS)

    // Medium red sphere — heavier, moderately bouncy.
    sphereM := LoadModel("assets/sphere.obj", "assets/asphalt.png", rl.RED)
    AddRigidbody(&sphereM, isStatic = false, bounciness = 1.2, mass = 2.0)
    AddSphereCollider(&sphereM)

    // Green box — medium mass, higher restitution than the blue box.
    cubeM := LoadModel("assets/cube.obj", "assets/box.png", rl.GREEN)
    AddRigidbody(&cubeM, isStatic = false, bounciness = 1.4, mass = 3.0)
    AddBoxCollider(&cubeM)

    // Blue box — heaviest dynamic object, lowest bounce.
    cubeL := LoadModel("assets/cube.obj", "assets/box.png", rl.BLUE)
    AddRigidbody(&cubeL, isStatic = false, bounciness = 1.2, mass = 5.0)
    AddBoxCollider(&cubeL)

    // Floor — static (infinite mass), large white cube.
    cubeFloor := LoadModel("assets/cube.obj", "assets/box.png")
    AddRigidbody(&cubeFloor, isStatic = true)
    AddBoxCollider(&cubeFloor)

    // Initial world-space positions.
    sphereS.translation   = {1.0, 3.0, 1.0}
    sphereM.translation   = {0.0, 4.0, 1.0}
    cubeM.translation     = {0.0, 2.0, 1.0}
    cubeL.translation     = {0.0, 1.0, 1.0}
    cubeFloor.translation = {0.0, -5.0, 1.0}

    // Scale the objects so they fit the scene nicely.
    sphereS.scale   = 0.3   // Small sphere
    cubeM.scale     = 0.5   // Medium cube
    sphereM.scale   = 0.5   // Medium sphere
    cubeFloor.scale = 2.5   // Large floor slab

    // Initial orientations: rotate boxes by different yaw angles so they
    // don't land perfectly aligned (more interesting stacking behaviour).
    RotateAround(&cubeL,     {0, 1, 0}, 30)
    RotateAround(&cubeM,     {0, 1, 0}, 330)
    RotateAround(&cubeFloor, {0, 1, 1}, 0)

    // Collect all models into a slice for per-frame iteration.
    models := []Model{sphereS, sphereM, cubeM, cubeL, cubeFloor}

    // -------------------------------------------------------
    // Camera
    // Fixed camera looking from (0,0,-3) toward (0,-1,0) — slightly
    // below the origin so the stacked objects are centred in the view.
    // -------------------------------------------------------
    camera := MakeCamera({0.0, 0.0, -3.0}, {0.0, -1.0, 0.0})

    // -------------------------------------------------------
    // Lights
    // Two coloured point/directional lights are created in view space
    // so the shading code can work directly in view space coordinates.
    // light:  blue-ish, upper-left
    // light2: green-ish, lower-right
    // Both have alpha = 1.0 (full intensity).
    // -------------------------------------------------------
    viewMatrix := MakeViewMatrix(camera.position, camera.target)
    light  := MakeLight({-2.0, 2.0, 1.0}, { 1.0,  1.0, 0.0}, {0.0, 0.1, 1.0, 1.0}, viewMatrix)
    light2 := MakeLight({2.0, -2.0, 1.0}, {-1.0, -1.0, 0.0}, {0.0, 1.0, 0.0, 1.0}, viewMatrix)
    lights := []Light{light, light2}

    // Ambient light: a small constant contribution so shadowed faces
    // are not completely black.  Two different ambients are used for
    // different render modes to vary the look.
    ambient  := Vector3{0.2, 0.2, 0.2}  // For flat and textured-flat modes.
    ambient2 := Vector3{0.1, 0.1, 0.2}  // Slightly blue ambient for Phong modes.

    // -------------------------------------------------------
    // Z-buffer: allocated on the heap (too large for the stack).
    // -------------------------------------------------------
    zBuffer := new(ZBuffer)

    // -------------------------------------------------------
    // Render mode state.
    // Start in mode 7 (textured Phong) — the richest mode.
    // -------------------------------------------------------
    renderModesCount :: 8
    renderMode: i8 = renderModesCount - 1  // Start at mode 7.

    // -------------------------------------------------------
    // Back-buffer image and display texture.
    // All drawing happens into renderImage (CPU memory).
    // After every frame, renderImage is uploaded to renderTexture
    // (GPU) and drawn to the screen as a full-screen quad.
    // -------------------------------------------------------
    renderImage   := rl.GenImageColor(SCREEN_WIDTH, SCREEN_HEIGHT, rl.LIGHTGRAY)
    renderTexture := rl.LoadTextureFromImage(renderImage)

    // -------------------------------------------------------
    // Projection matrices.
    // Both are pre-computed once at startup; only one is active
    // per frame depending on projectionType.
    // -------------------------------------------------------
    projectionMatrix  : Matrix4x4
    projectionType    : ProjectionType = .Perspective
    perspectiveMatrix  := MakePerspectiveMatrix(FOV, SCREEN_WIDTH, SCREEN_HEIGHT, NEAR_PLANE, FAR_PLANE)
    orthographicMatrix := MakeOrthographicMatrix(SCREEN_WIDTH, SCREEN_HEIGHT, NEAR_PLANE, FAR_PLANE)

    // The currently selected (keyboard/mouse-active) model.
    // Starts as the first model (sphereS).
    selectedModel := &models[0]

    // -------------------------------------------------------
    // Fixed-timestep physics accumulator.
    // Real elapsed time is accumulated here and drained in
    // increments of PHYSICS_TIMESTEP so the simulation runs at
    // a stable rate regardless of frame rate.
    // -------------------------------------------------------
    physicsAccumulator: f32

    // -------------------------------------------------------
    // Main loop
    // -------------------------------------------------------
    for !rl.WindowShouldClose() {
        // --- Time ---
        // rl.GetFrameTime() returns the wall-clock time elapsed since
        // the previous frame in seconds.  Capped implicitly by the
        // physics accumulator drain loop to avoid a "spiral of death"
        // (accumulating too much time after a long stall).
        deltaTime := rl.GetFrameTime()

        // --- Input ---
        // Handle keyboard and mouse input; may modify selectedModel,
        // renderMode, projectionType, and physics forces.
        HandleInputs(&selectedModel, models[:], camera, &renderMode, renderModesCount, &projectionType, deltaTime)

        // --- Physics (fixed-timestep) ---
        // Accumulate elapsed time.
        physicsAccumulator += deltaTime
        // Drain the accumulator in fixed-size steps.
        for physicsAccumulator >= PHYSICS_TIMESTEP {
            // Step 1: apply forces, integrate velocities and positions.
            ApplyPhysics(models, PHYSICS_TIMESTEP)
            // Step 2: detect and resolve all pairwise collisions
            //         (SOLVER_ITERATIONS passes for stability).
            ResolveCollisions(models)
            physicsAccumulator -= PHYSICS_TIMESTEP
        }

        // --- Transform all models to view space ---
        // Computes the combined model-view matrix for each model and
        // writes the transformed vertices and normals into the mesh's
        // transformedVertices / transformedNormals buffers.
        for &model in models {
            ApplyTransformations(&model, camera)
        }

        // --- Select projection matrix for this frame ---
        switch projectionType {
            case .Perspective:  projectionMatrix = perspectiveMatrix
            case .Orthographic: projectionMatrix = orthographicMatrix
        }

        // --- Render ---
        rl.BeginDrawing()

        // Clear the Z-buffer to "infinitely far" before drawing anything.
        ClearZBuffer(zBuffer)

        // Draw every model using the currently active render mode.
        for &model in models {
            switch renderMode {
                // Mode 0: wireframe edges, no back-face culling
                case 0: DrawWireframe(
                    model.mesh.transformedVertices, model.mesh.triangles,
                    projectionMatrix, projectionType,
                    model.wireColor, false, &renderImage)

                // Mode 1: wireframe edges, with back-face culling
                case 1: DrawWireframe(
                    model.mesh.transformedVertices, model.mesh.triangles,
                    projectionMatrix, projectionType,
                    model.wireColor, true, &renderImage)

                // Mode 2: solid fill, no lighting (flat colour)
                case 2: DrawUnlit(
                    model.mesh.transformedVertices, model.mesh.triangles,
                    projectionMatrix, projectionType,
                    model.color, zBuffer, &renderImage)

                // Mode 3: flat shading — one Lambertian colour per face
                case 3: DrawFlatShaded(
                    model.mesh.transformedVertices, model.mesh.triangles,
                    projectionMatrix, projectionType,
                    lights, model.color, zBuffer, &renderImage, ambient)

                // Mode 4: Phong shading — per-pixel diffuse, colour mesh
                case 4: DrawPhongShaded(
                    model.mesh.transformedVertices, model.mesh.triangles,
                    model.mesh.transformedNormals,
                    lights, model.color, zBuffer,
                    projectionMatrix, projectionType, &renderImage, ambient2)

                // Mode 5: perspective-correct texture, no lighting
                case 5: DrawTexturedUnlit(
                    model.mesh.transformedVertices, model.mesh.triangles,
                    model.mesh.uvs, model.texture, zBuffer,
                    projectionMatrix, projectionType, &renderImage)

                // Mode 6: perspective-correct texture + flat shading
                case 6: DrawTexturedFlatShaded(
                    model.mesh.transformedVertices, model.mesh.triangles,
                    model.mesh.uvs, lights, model.texture, zBuffer,
                    projectionMatrix, projectionType, &renderImage, ambient)

                // Mode 7: perspective-correct texture + Phong shading (highest quality)
                case 7: DrawTexturedPhongShaded(
                    model.mesh.transformedVertices, model.mesh.triangles,
                    model.mesh.uvs, model.mesh.transformedNormals,
                    lights, model.texture, zBuffer,
                    projectionMatrix, projectionType, &renderImage, ambient2)
            }
        }

        // --- Upload back-buffer to GPU and display ---
        // rl.UpdateTexture transfers the CPU pixel data to the GPU texture object.
        rl.UpdateTexture(renderTexture, renderImage.data)
        // Draw the texture as a full-screen quad at (0,0) with no tint.
        rl.DrawTexture(renderTexture, 0, 0, rl.WHITE)
        // Overlay the FPS counter in the top-left corner.
        rl.DrawFPS(10, 10)

        // Clear the back-buffer to black so the next frame starts clean.
        rl.ImageClearBackground(&renderImage, rl.BLACK)

        rl.EndDrawing()
    }

    // Clean up the window when the main loop exits.
    rl.CloseWindow()
}
