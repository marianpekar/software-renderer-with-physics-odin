package raycasted_maze

import "core:math"
import rl "vendor:raylib"

Player :: struct { 
    x, y, 
    angle: f32,
    restart: bool,
    mazeType: MazeType,
}

Map :: struct {
    size: f32,
    show: bool,
    isTransparent: bool
}

Cursor :: struct {
    x, y: f32,
    tile: i32
}

HandleInputs :: proc(player: ^Player, maze: ^Maze, cursor: ^Cursor, map_: ^Map, deltaTime: f32) {
    walk := f32(0)
    turn := f32(0)

    if rl.IsKeyDown(rl.KeyboardKey.W) do walk =  1
    if rl.IsKeyDown(rl.KeyboardKey.S) do walk = -1
    if rl.IsKeyDown(rl.KeyboardKey.A) do turn = -1
    if rl.IsKeyDown(rl.KeyboardKey.D) do turn =  1

    player.angle += turn * deltaTime

    step := walk * 200 * deltaTime
    next: Vec2f = {
        player.x + math.cos(player.angle) * step,
        player.y + math.sin(player.angle) * step
    }
    
    if !HasWallAt(maze, next) {
        player.x = next.x
        player.y = next.y
    }

    if rl.IsKeyPressed(rl.KeyboardKey.M) do map_.show = !map_.show
    if rl.IsKeyPressed(rl.KeyboardKey.R) do player.restart = true

    if rl.IsKeyPressed(rl.KeyboardKey.KP_1) {
        player.mazeType = .Recursive
        player.restart = true
    }

    if rl.IsKeyPressed(rl.KeyboardKey.KP_2) {
        player.mazeType = .Stack
        player.restart = true
    }

    if rl.IsKeyPressed(rl.KeyboardKey.KP_3) {
        player.mazeType = .Circular
        player.restart = true
    }

    if rl.IsKeyPressed(rl.KeyboardKey.KP_ADD) do map_.size /= 2
    if rl.IsKeyPressed(rl.KeyboardKey.KP_SUBTRACT) do map_.size *= 2
    if map_.size < 8 do map_.size = 8
    if map_.size > 32 do map_.size = 32

    if rl.IsKeyPressed(rl.KeyboardKey.T) do map_.isTransparent = !map_.isTransparent

    if rl.IsKeyPressed(rl.KeyboardKey.P) do PaintWalls(maze)

    if rl.IsKeyPressed(rl.KeyboardKey.C) do Clear(maze)

    if map_.show {
        mp := rl.GetMousePosition()
        cursor.x = mp.x
        cursor.y = mp.y
        mapPos := Vec2i{i32(cursor.x * map_.size) / TILE_SIZE, i32(cursor.y * map_.size) / TILE_SIZE}

        if rl.IsMouseButtonDown(rl.MouseButton.LEFT) do Close(maze, mapPos, cursor.tile)
        if rl.IsMouseButtonDown(rl.MouseButton.RIGHT) do Open(maze, mapPos)

        if rl.GetMouseWheelMove() > 0 {
            cursor.tile += 1
            if cursor.tile > NUM_TILES do cursor.tile = 1
        } 

        if rl.GetMouseWheelMove() < 0 {
            cursor.tile -= 1
            if cursor.tile < 1 do cursor.tile = NUM_TILES
        }
    }
}

