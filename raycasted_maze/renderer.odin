package raycasted_maze

import "core:math"
import rl "vendor:raylib"

SCREEN_WIDTH :: 1024
SCREEN_HEIGHT :: 1024
HALF_SCREEN_WIDTH  :: SCREEN_WIDTH / 2
HALF_SCREEN_HEIGHT :: SCREEN_HEIGHT / 2
DistToProjPlane := HALF_SCREEN_WIDTH / math.tan(f32(HALF_FOV))

Render :: proc(player: Player, rays: Rays, tiles: Tiles, image: ^rl.Image) {
    for i in 0..<SCREEN_WIDTH {
        ray := rays[i]
        wallHeight := TILE_SIZE / ray.dist * DistToProjPlane
        wallHalfHeight := wallHeight * 0.5

        wallTopPx := HALF_SCREEN_HEIGHT - wallHalfHeight
        if wallTopPx < 0 do wallTopPx = 0

        wallBottomPx :=  HALF_SCREEN_HEIGHT + wallHalfHeight
        if wallBottomPx > SCREEN_HEIGHT do wallBottomPx = SCREEN_HEIGHT

        for j in 0..=wallTopPx do rl.ImageDrawPixel(image, i32(i), i32(j), rl.DARKGRAY)

        tileOffset: Vec2f
        tileOffset.x = ray.hit.y if ray.isHitVertical else ray.hit.x

        for j in wallTopPx..<wallBottomPx {
            distFromTop := j + wallHalfHeight - HALF_SCREEN_HEIGHT
            tileOffset.y = distFromTop * TILE_SIZE / wallHeight
            
            texel := tiles[ray.tile-1][i32(tileOffset.y) * TILE_SIZE + i32(tileOffset.x) % TILE_SIZE]

            if ray.isHitVertical do Darken(&texel, 0.666)

            rl.ImageDrawPixel(image, i32(i), i32(j), texel)
        }

        for j in wallBottomPx..<SCREEN_HEIGHT do rl.ImageDrawPixel(image, i32(i), i32(j), rl.GRAY)
    }

    Darken :: proc(c: ^rl.Color, f: f32) {
        c.r = u8(f32(c.r) * f)
        c.g = u8(f32(c.g) * f)
        c.b = u8(f32(c.b) * f)
    }
}

RenderMap :: proc(maze: Maze, player: Player, rays: Rays, colors: MapColors, map_: Map, cursor: Cursor, image: ^rl.Image) {
    scale := f32(1.0) / map_.size
    size := i32(f32(TILE_SIZE) * scale)
    
    for y in 0..<MAZE_HEIGHT {
        for x in 0..<MAZE_WIDTH {
            tile := maze[x + y * MAZE_WIDTH]
            color := colors[tile]

            px := i32(f32(x * TILE_SIZE) * scale)
            py := i32(f32(y * TILE_SIZE) * scale)

            if map_.isTransparent {
                for dy in 0..<size {
                    for dx in 0..<size {
                        current := rl.GetImageColor(image^, px + dx, py + dy)
                        color := rl.ColorAlphaBlend(current, color, rl.WHITE)
                        rl.ImageDrawPixel(image, px + dx, py + dy, color)
                    }
                }
            }
            else {
                rl.ImageDrawRectangle(image, px, py, size, size, color)
            }
        }
    }

    psx := i32(player.x * scale)
    psy := i32(player.y * scale)

    for i in 0..<SCREEN_WIDTH {
        rex := i32(rays[i].hit.x * scale)
        rey := i32(rays[i].hit.y * scale)
        rl.ImageDrawLine(image, psx, psy, rex, rey, rl.RED)
    }

    pex := i32(f32(player.x + math.cos(player.angle) * TILE_SIZE) * scale)
    pey := i32(f32(player.y + math.sin(player.angle) * TILE_SIZE) * scale)
    rl.ImageDrawLine(image, psx, psy, pex, pey, rl.WHITE)

    cx := i32(cursor.x) / i32(size) * i32(size)
    cy := i32(cursor.y) / i32(size) * i32(size)
    rl.ImageDrawRectangle(image, cx, cy, i32(size), i32(size), colors[cursor.tile])
}