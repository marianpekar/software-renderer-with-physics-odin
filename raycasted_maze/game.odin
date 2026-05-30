package raycasted_maze

import rl "vendor:raylib"

Game :: struct {
    player: Player,
    cursor: Cursor,
    map_: Map,
    maze: Maze,
    rays: Rays,
    tiles: Tiles,
    mapColors: MapColors
}

MakeGame :: proc() -> Game {
    player: Player
    player.mazeType = .Recursive

    cursor: Cursor
    cursor.tile = 1

    map_: Map
    map_.size = 16
    map_.isTransparent = true
    map_.show = false
    
    maze: Maze
    rays: Rays

    tiles := LoadTiles("raycasted_maze/tiles")
    mapColors := MakeMapColors(tiles)

    game := Game{
        player = player,
        cursor = cursor,
        map_ = map_,
        maze = maze,
        rays = rays,
        tiles = tiles
    }
    
    RestartGame(&game)

    return game
}

RestartGame :: proc(game: ^Game) {
    start := Vec2i{MAZE_WIDTH / 2 + 1, MAZE_HEIGHT / 2 + 1}
    game.maze = GenerateMaze(start, game.player.mazeType)

    game.player.x = f32(start.x) * TILE_SIZE + TILE_SIZE / 2
    game.player.y = f32(start.y) * TILE_SIZE + TILE_SIZE / 2
    game.player.angle = 0
    game.player.restart = false
}

UpdateGame :: proc(game: ^Game, texture: rl.Texture, image: ^rl.Image) {
    HandleInputs(&game.player, &game.maze, &game.cursor, &game.map_, rl.GetFrameTime())

    if game.player.restart do RestartGame(game)

    CastRays(game.player, &game.maze, &game.rays)

    Render(game.player, game.rays, game.tiles, image)
    if game.map_.show {
        RenderMap(game.maze, game.player, game.rays, game.mapColors, game.map_, game.cursor, image)
    }

    rl.UpdateTexture(texture, image.data)
}