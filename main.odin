package main

import rl "vendor:raylib"

ProjectionType :: enum {
    Perspective,
    Orthographic,
}

main :: proc() {
    rl.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Renderer")

    sphereS := LoadModel("assets/sphere.obj", "assets/asphalt.png", rl.YELLOW)
    AddRigidbody(&sphereS, isStatic = false, bounciness = 1.9, mass = 1.0)
    AddSphereCollider(&sphereS)

    sphereM := LoadModel("assets/sphere.obj", "assets/asphalt.png", rl.RED)
    AddRigidbody(&sphereM, isStatic = false, bounciness = 1.7, mass = 2.0)
    AddSphereCollider(&sphereM)

    cubeM := LoadModel("assets/cube.obj", "assets/box.png", rl.GREEN)
    AddRigidbody(&cubeM, isStatic = false, bounciness = 1.4, mass = 3.0)
    AddBoxCollider(&cubeM)
    
    cubeL := LoadModel("assets/cube.obj", "assets/box.png", rl.BLUE)
    AddRigidbody(&cubeL, isStatic = false, bounciness = 1.2, mass = 5.0)
    AddBoxCollider(&cubeL)

    cubeFloor := LoadModel("assets/cube.obj", "assets/box.png")
    AddRigidbody(&cubeFloor, isStatic = true)
    AddBoxCollider(&cubeFloor)

    sphereS.translation = {1.0, 3.0, 1.0}
    sphereM.translation = {1.0, 2.0, 1.0}
    cubeM.translation = {0.0, 2.0, 1.0}
    cubeL.translation = {0.0, 1.0, 1.0}
    cubeFloor.translation = {0.0, -5.0, 1.0}
    sphereS.scale = 0.3
    cubeM.scale = 0.5
    sphereM.scale = 0.5
    cubeFloor.scale = 2.5
    RotateAround(&cubeL, {0, 1, 0}, 30)
    RotateAround(&cubeM, {0, 1, 0}, 330)
    RotateAround(&cubeFloor, {0, 1, 1}, 10)

    models := []Model{sphereS, sphereM, cubeM, cubeL, cubeFloor}

    camera := MakeCamera({0.0, 0.0, -3.0}, {0.0, -1.0, 0.0})

    light  := MakeLight({-1.0, 2.0, -5.0}, { 1.0,  1.0, 0.0}, {0.0, 0.1, 1.0, 1.0})
    light2 := MakeLight({ 1.0, -2.0, -5.0}, {-1.0, -1.0, 0.0}, {0.0, 1.0, 0.0, 1.0})
    lights := []Light{light, light2}

    ambient := Vector3{0.2, 0.2, 0.2}
    ambient2 := Vector3{0.1, 0.1, 0.2}

    zBuffer := new(ZBuffer)

    renderModesCount :: 8
    renderMode: i8 = renderModesCount - 1

    renderImage := rl.GenImageColor(SCREEN_WIDTH, SCREEN_HEIGHT, rl.LIGHTGRAY)
    renderTexture := rl.LoadTextureFromImage(renderImage)

    projectionMatrix : Matrix4x4
    projectionType : ProjectionType = .Perspective
    perspectiveMatrix := MakePerspectiveMatrix(FOV, SCREEN_WIDTH, SCREEN_HEIGHT, NEAR_PLANE, FAR_PLANE)
    orthographicMatrix := MakeOrthographicMatrix(SCREEN_WIDTH, SCREEN_HEIGHT, NEAR_PLANE, FAR_PLANE)

    selectedModel := &models[0]

    physicsAccumulator: f32

    for !rl.WindowShouldClose() {
        deltaTime := rl.GetFrameTime()

        HandleInputs(&selectedModel, models[:], camera, &renderMode, renderModesCount, &projectionType, deltaTime)

        physicsAccumulator += deltaTime
        for physicsAccumulator >= PHYSICS_TIMESTEP {
            ApplyPhysics(models, PHYSICS_TIMESTEP)
            ResolveCollisions(models)
            physicsAccumulator -= PHYSICS_TIMESTEP
        }

        for &model in models {
            ApplyTransformations(&model, camera)
        }

        switch projectionType {
            case .Perspective: projectionMatrix = perspectiveMatrix
            case .Orthographic: projectionMatrix = orthographicMatrix
        }

        rl.BeginDrawing()

        ClearZBuffer(zBuffer)
        
        for &model in models {
            switch renderMode {
                case 0: DrawWireframe(model.mesh.transformedVertices, model.mesh.triangles, projectionMatrix, projectionType, model.wireColor, false, &renderImage)
                case 1: DrawWireframe(model.mesh.transformedVertices, model.mesh.triangles, projectionMatrix, projectionType, model.wireColor, true, &renderImage)
                case 2: DrawUnlit(model.mesh.transformedVertices, model.mesh.triangles, projectionMatrix, projectionType, model.color, zBuffer, &renderImage)
                case 3: DrawFlatShaded(model.mesh.transformedVertices, model.mesh.triangles, projectionMatrix, projectionType, lights, model.color, zBuffer, &renderImage, ambient)
                case 4: DrawPhongShaded(model.mesh.transformedVertices, model.mesh.triangles, model.mesh.transformedNormals, lights, model.color, zBuffer, projectionMatrix, projectionType, &renderImage, ambient2)
                case 5: DrawTexturedUnlit(model.mesh.transformedVertices, model.mesh.triangles, model.mesh.uvs, model.texture, zBuffer, projectionMatrix, projectionType, &renderImage)
                case 6: DrawTexturedFlatShaded(model.mesh.transformedVertices, model.mesh.triangles, model.mesh.uvs, lights, model.texture, zBuffer, projectionMatrix, projectionType, &renderImage, ambient)
                case 7: DrawTexturedPhongShaded(model.mesh.transformedVertices, model.mesh.triangles, model.mesh.uvs, model.mesh.transformedNormals, lights, model.texture, zBuffer, projectionMatrix, projectionType, &renderImage, ambient2)
            }
        }

        rl.UpdateTexture(renderTexture, renderImage.data)
        rl.DrawTexture(renderTexture, 0, 0, rl.WHITE)
        rl.DrawFPS(10, 10)
        rl.ImageClearBackground(&renderImage, rl.BLACK)

        rl.EndDrawing()
    }

    rl.CloseWindow()
}