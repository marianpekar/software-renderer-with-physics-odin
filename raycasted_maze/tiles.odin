package raycasted_maze

import "core:strings"
import "core:os"
import rl "vendor:raylib"

NUM_TILES :: 8
TILE_SIZE :: 64

Tile :: [^]rl.Color
Tiles :: [NUM_TILES]Tile

LoadTiles :: proc(dir: string) -> Tiles {
    tiles: Tiles

    fis, err := os.read_all_directory_by_path(dir, context.temp_allocator)

    for info, i in fis {
        path := strings.clone_to_cstring(info.fullpath)
        image := rl.LoadImage(path)
        tiles[i] = rl.LoadImageColors(image)
        rl.UnloadImage(image)
    }

    return tiles
}

MapColors :: [NUM_TILES + 1]rl.Color

MakeMapColors :: proc(tiles: Tiles) -> MapColors {
    colors: [NUM_TILES + 1]rl.Color
    fade := f32(0.66)
    colors[0] = rl.Fade(rl.BLACK, fade)
    n := u32(TILE_SIZE * TILE_SIZE)
    i := 1
    for tile in tiles {
        r, g, b, a: u32
        for j in 0..<TILE_SIZE * TILE_SIZE {
            c := tile[j]
            r += u32(c.r)
            g += u32(c.g)
            b += u32(c.b)
        }
        color := rl.Color{u8(r/n), u8(g/n), u8(b/n), 255}
        colors[i] = rl.Fade(color, fade)
        i += 1
    }
    return colors
}