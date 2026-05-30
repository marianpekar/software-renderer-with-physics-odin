package main

import rm "raycasted_maze"
import rl "vendor:raylib"

ProjectionType :: enum {
    Perspective,
    Orthographic,
}

main :: proc() {
    rl.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Renderer")

    cubeM := LoadModel("assets/cube.obj", "assets/box.png", rl.GREEN)
    AddRigidbody(&cubeM, isStatic = false, bounciness = 1.4, mass = 3.0)
    AddBoxCollider(&cubeM)
    
    cubeL := LoadModel("assets/cube.obj", "assets/box.png", rl.BLUE)
    AddRigidbody(&cubeL, isStatic = false, bounciness = 1.2, mass = 5.0)
    AddBoxCollider(&cubeL)

    cubeFloor := LoadModel("assets/cube.obj", "assets/box.png")
    AddRigidbody(&cubeFloor, isStatic = true)
    AddBoxCollider(&cubeFloor)

    cubeM.translation = {0.0, 2.0, 1.0}
    cubeL.translation = {0.0, 1.0, 1.0}
    cubeFloor.translation = {0.0, -5.0, 1.0}
    cubeM.scale = 0.5
    cubeFloor.scale = 2.5
    RotateAround(&cubeL, {0, 1, 0}, 30)
    RotateAround(&cubeM, {0, 1, 0}, 330)
    RotateAround(&cubeFloor, {0, 1, 1}, 0)

    models := []Model{cubeM, cubeL, cubeFloor}

    camera := MakeCamera({0.0, 0.0, -3.0}, {0.0, -1.0, 0.0})

    viewMatrix := MakeViewMatrix(camera.position, camera.target)
    light  := MakeLight({-2.0, 2.0, 1.0}, { 1.0,  1.0, 0.0}, {1.0, 1.0, 1.0, 1.0}, viewMatrix)
    light2 := MakeLight({2.0, -2.0, 1.0}, {-1.0, -1.0, 0.0}, {1.0, 1.0, 1.0, 1.0}, viewMatrix)
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

    raycastedMazeImage := rl.GenImageColor(rm.SCREEN_WIDTH, rm.SCREEN_HEIGHT, rl.BLACK)
    raycastedMazeTexture := rl.LoadTextureFromImage(renderImage)
    rl.ImageFormat(&raycastedMazeImage, .UNCOMPRESSED_R8G8B8A8)
    models[0].texture = Texture{
        width = raycastedMazeImage.width, 
        height = raycastedMazeImage.height,
        pixels = ([^]rl.Color)(raycastedMazeImage.data)
    }
    models[1].texture = Texture{
        width = raycastedMazeImage.width, 
        height = raycastedMazeImage.height,
        pixels = ([^]rl.Color)(raycastedMazeImage.data)
    }

    player: rm.Player
    player.mazeType = .Recursive

    cursor: rm.Cursor
    cursor.tile = 1

    map_: rm.Map
    map_.size = 16
    map_.isTransparent = true
    map_.show = false
    
    maze: rm.Maze
    rays: rm.Rays

    tiles := rm.LoadTiles("raycasted_maze/tiles")
    mapColors := rm.MakeMapColors(tiles)

    RestartMaze(&maze, &player)

    for !rl.WindowShouldClose() {
        deltaTime := rl.GetFrameTime()

        rm.HandleInputs(&player, &maze, &cursor, &map_, rl.GetFrameTime())
        if player.restart do RestartMaze(&maze, &player)
        rm.CastRays(player, &maze, &rays)
        rm.Render(player, rays, tiles, &raycastedMazeImage)
        if map_.show {
            rm.RenderMap(maze, player, rays, mapColors, map_, cursor, &raycastedMazeImage)
        }
        rl.UpdateTexture(raycastedMazeTexture, raycastedMazeImage.data)

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
        rl.ImageClearBackground(&renderImage, rl.BLACK)

        rl.EndDrawing()
    }

    rl.CloseWindow()
}

RestartMaze :: proc(maze: ^rm.Maze, player: ^rm.Player) {
    start := rm.Vec2i{rm.MAZE_WIDTH / 2 + 1, rm.MAZE_HEIGHT / 2 + 1}
    maze^ = rm.GenerateMaze(start, player.mazeType)

    player.x = f32(start.x) * rm.TILE_SIZE + rm.TILE_SIZE / 2
    player.y = f32(start.y) * rm.TILE_SIZE + rm.TILE_SIZE / 2
    player.angle = 0
    player.restart = false
}