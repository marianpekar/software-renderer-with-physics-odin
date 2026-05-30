package raycasted_maze

import "core:math"

FOV :: 60 * math.PI / 180
HALF_FOV :: FOV / 2

Vec2f :: [2]f32

Ray :: struct {
    dist: f32,
    hit: Vec2f,
    isHitVertical: bool,
    isPointingDown: bool,
    isPointingRight: bool,
    tile: i32
}

Rays :: [SCREEN_WIDTH]Ray

CastRays :: proc(player: Player, maze: ^Maze, rays: ^Rays) {
    rayAngle := player.angle - HALF_FOV

    for i in 0..<SCREEN_WIDTH {
        CastRay(player, maze, NormalizeAngle(rayAngle), rays, i)
        rayAngle += FOV / SCREEN_WIDTH
    }

    NormalizeAngle :: proc(angle: f32) -> f32 {
        a := math.remainder(angle, math.TAU)
        if a < 0 {
            a += math.TAU
        }
        return a
    }

    CastRay :: proc(player: Player, maze: ^Maze, rayAngle: f32, rays: ^Rays, rayIdx: int) {
        isRayPoitingDown := rayAngle > 0 && rayAngle < math.PI
        isRayPointingRight := rayAngle < 0.5 * math.PI || rayAngle > 1.5 * math.PI

        hasHorizontalHit := false
        hHit: Vec2f
        hTile: i32
        intercept: Vec2f
        
        intercept.y = math.floor(player.y / TILE_SIZE) * TILE_SIZE + (TILE_SIZE if isRayPoitingDown else 0)
        intercept.x = player.x + (intercept.y - player.y) / math.tan(rayAngle)

        step: Vec2f = {TILE_SIZE / math.tan(rayAngle), TILE_SIZE}
        if !isRayPoitingDown do step.y *= -1
        if !isRayPointingRight && step.x > 0 do step.x *= -1
        if  isRayPointingRight && step.x < 0 do step.x *= -1

        hNext: Vec2f = {intercept.x, intercept.y}
        for IsInBounds(hNext) {
            check: Vec2f = {hNext.x, hNext.y + (-1 if !isRayPoitingDown else 0)}
            if (HasWallAt(maze, check)) {
                hHit = {hNext.x, hNext.y}
                hTile = GetTile(maze, check)
                hasHorizontalHit = true
                break
            }
            hNext += step
        }

        hasVerticalHit := false
        vHit: Vec2f
        vTile: i32

        intercept.x = math.floor(player.x / TILE_SIZE) * TILE_SIZE + (TILE_SIZE if isRayPointingRight else 0)
        intercept.y = player.y + (intercept.x - player.x) * math.tan(rayAngle)

        step.x = TILE_SIZE * (-1 if !isRayPointingRight else 1)
        step.y = TILE_SIZE * math.tan(rayAngle)
        if !isRayPoitingDown && step.y > 0 do step.y *= -1
        if isRayPoitingDown && step.y < 0 do step.y *= -1

        vNext: Vec2f = {intercept.x, intercept.y}
        for IsInBounds(vNext) {
            check: Vec2f = {vNext.x + (-1 if !isRayPointingRight else 0), vNext.y}
            if (HasWallAt(maze, check)) {
                vHit = {vNext.x, vNext.y}
                vTile = GetTile(maze, check)
                hasVerticalHit = true
                break
            }
            vNext += step
        }

        hHitDist := GetDistance({player.x, player.y}, hHit) if hasHorizontalHit else 999_999
        vHitDist := GetDistance({player.x, player.y}, vHit) if hasVerticalHit else 999_999

        if vHitDist < hHitDist {
            rays[rayIdx].dist = vHitDist * math.cos(rayAngle - player.angle)
            rays[rayIdx].hit = vHit
            rays[rayIdx].isHitVertical = true 
            rays[rayIdx].tile = vTile
        } else {
            rays[rayIdx].dist = hHitDist * math.cos(rayAngle - player.angle)
            rays[rayIdx].hit = hHit
            rays[rayIdx].isHitVertical = false 
            rays[rayIdx].tile = hTile
        }

        rays[rayIdx].isPointingDown = isRayPoitingDown
        rays[rayIdx].isPointingRight = isRayPointingRight

        return
        
        IsInBounds :: proc(t: Vec2f) -> bool {
            return t.x >= 0 && t.x <= MAZE_WIDTH * TILE_SIZE &&
                   t.y >= 0 && t.y <= MAZE_HEIGHT * TILE_SIZE
        }

        GetTile :: proc(maze: ^Maze, p: Vec2f) -> i32 {
            tile: Vec2i = { i32(p.x / TILE_SIZE), i32(p.y / TILE_SIZE)}
            return GetItem(maze, tile)
        }

        GetDistance :: proc(a, b: Vec2f) -> f32 {
            return math.sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y))
        }
    }
}

HasWallAt :: proc(maze: ^Maze, p: Vec2f) -> bool {
    if p.x < 0 || p.x > MAZE_WIDTH * TILE_SIZE || p.y < 0 || p.y > MAZE_HEIGHT * TILE_SIZE {
        return true
    }

    tile: Vec2i = { i32(p.x / TILE_SIZE), i32(p.y / TILE_SIZE)}
    return !IsOpen(maze, tile)
}