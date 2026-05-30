package raycasted_maze

import "core:fmt"
import "core:math/rand"

Vec2i :: [2]i32

MAZE_WIDTH :: 65
MAZE_HEIGHT :: 65

Maze :: [MAZE_WIDTH * MAZE_HEIGHT]i32

MazeType :: enum {
   Stack,
   Recursive,
   Circular
}

GenerateMaze :: proc(start: Vec2i, type: MazeType) -> Maze {
    maze: Maze

    switch type {
        case .Stack: maze = GenerateMazeStack(start)
        case .Recursive: maze = GenerateMazeRecursive(start)
        case .Circular: maze = GenerateMazeCicrular(start)
    }

    PaintWalls(&maze)

    return maze
}

PaintWalls :: proc(maze: ^Maze) {
    perms := MakePermutations()
    for y in 0..<MAZE_HEIGHT {
        for x in 0..<MAZE_WIDTH {
            if IsOpen(maze, {i32(x), i32(y)}) {
                continue
            }

            maze[x + y * MAZE_WIDTH] = PaintWall(perms, {f32(x), f32(y)})
        }
    }

    PaintWall :: proc(perms: Permutations, p: Vec2f) -> i32 {
        scale: f32 = 0.0001
        s := SampleFractalBrownianMotion(
            perms = perms,
            v = {p.x * scale, p.y * scale},
            octaves = 8,
            persistence = 28,
            low = 1,
            high = NUM_TILES
        )
        return i32(s)
    }
}

GenerateMazeStack :: proc(start: Vec2i) -> Maze {
    Directions :: [4]Vec2i
    AllDirections :: [24]Directions

    maze: Maze = 1
    stack: Stack(Vec2i)
    dirs := MakeAllDirections()

    Open(&maze, start)
    Push(&stack, start)

    for !IsEmpty(&stack) {
        c := Pop(&stack)

        currentDirs := dirs[rand.int31_max(24)]

        for i in 0..<4 {
            d := currentDirs[i]
            n: Vec2i = {c.x + d.x * 2, c.y + d.y * 2}

            if !IsValid(n) do continue

            if !IsOpen(&maze, n) {
                Open(&maze, {(n.x + c.x) / 2, (n.y + c.y) / 2})
                Open(&maze, n)
                Push(&stack, n)
            }
        }
    }

    return maze

    MakeAllDirections :: proc() -> AllDirections {
        all: AllDirections
        base: Directions = {{1,0}, {-1,0}, {0,1}, {0,-1}}
        factorials := [4]int{6, 2, 1, 1}
        for n in 0..<24 {
            rem, d := base, n
            for i in 0..<4 {
                j := d / factorials[i]
                d   %= factorials[i]
                all[n][i] = rem[j]
                for k in j..<(3 - i) { rem[k] = rem[k + 1] }
            }
        }
        return all
    }
}

GenerateMazeRecursive :: proc(start: Vec2i) -> Maze {
    maze: Maze = 1
    Step(&maze, start)
    return maze

    Step :: proc(maze: ^Maze, p: Vec2i) {
        dirs: [4]i32
        for i in 0..<4 {
            dirs[i] = rand.int31_max(4)
        }
    
        for dir in dirs {
            switch dir {
                case 0:
                    if p.y - 2 <= 0 do continue
                    if !IsOpen(maze, {p.x, p.y - 2}) {
                        Open(maze, {p.x, p.y - 2})
                        Open(maze, {p.x, p.y - 1})
                        Step(maze, {p.x, p.y - 2})
                    }
                case 1:
                    if p.x + 2 >= MAZE_WIDTH do continue
                    if !IsOpen(maze, {p.x + 2, p.y}) {
                        Open(maze, {p.x + 2, p.y})
                        Open(maze, {p.x + 1, p.y})
                        Step(maze, {p.x + 2, p.y})
                    }
                case 2:
                    if p.y + 2 >= MAZE_HEIGHT do continue
                    if !IsOpen(maze, {p.x, p.y + 2}) {
                        Open(maze, {p.x, p.y + 2})
                        Open(maze, {p.x, p.y + 1})
                        Step(maze, {p.x, p.y + 2})
                    }                
                case 3:
                    if p.x - 2 <= 0 do continue
                    if !IsOpen(maze, {p.x - 2, p.y}) {
                        Open(maze, {p.x - 2, p.y})
                        Open(maze, {p.x - 1, p.y})
                        Step(maze, {p.x - 2, p.y})
                    }
            }
        }
    }
}

GenerateMazeCicrular :: proc(start: Vec2i) -> Maze {
    maze: Maze = 1
    Clear(&maze)

    maxRing := 0
    for ring in 1..<(MAZE_WIDTH / 2) {
        offset := ring * 2
        if offset > MAZE_WIDTH - 1 - offset || offset > MAZE_HEIGHT - 1 - offset do break
        maxRing = ring
    }

    for ring in 1..=maxRing {
        offset := i32(ring * 2)
        left := offset
        right := MAZE_WIDTH - 1 - offset
        top := offset
        bottom := MAZE_HEIGHT - 1 - offset

        tiles: [256]Vec2i
        numWalls := 0

        for x := left; x <= right; x += 1 {
            tiles[numWalls] = {x, top}
            numWalls += 1
        }

        for y := top + 1; y <= bottom; y += 1 {
            tiles[numWalls] = {right, y}
            numWalls += 1
        }

        if top != bottom {
            for x := right - 1; x >= left; x -= 1 {
                tiles[numWalls] = {x, bottom}
                numWalls += 1
            }
        }

        if left != right {
            for y := bottom - 1; y >= top + 1; y -= 1 {
                tiles[numWalls] = {left, y}
                numWalls += 1
            }
        }

        indices: [256]int
        numValid := 0
        for i in 0..<numWalls {
            c := tiles[i]
            isCorner := (c.x == left && c.y == top)   ||
                        (c.x == right && c.y == top)  ||
                        (c.x == right && c.y == bottom) ||
                        (c.x == left && c.y == bottom)
            if !isCorner {
                indices[numValid] = i
                numValid += 1
            }
        }

        if numValid <= 0 do continue

        nHoles := maxRing - ring + 1
        isHole: [256]bool

        r := rand.int_range(0, numValid)
        for h in 0..<nHoles {
            idx := indices[(r + h * numValid / nHoles) % numValid]
            isHole[idx] = true
        }

        for i in 0..<numWalls {
            if !isHole[i] {
                Close(&maze, tiles[i])
            }
        }
    }

    return maze
}

Open :: proc(maze: ^Maze, p: Vec2i) {
    if p.x <= 0 || p.x >= MAZE_WIDTH - 1 ||
       p.y <= 0 || p.y >= MAZE_HEIGHT - 1 {
        return
    }

    maze[p.x + p.y * MAZE_WIDTH] = 0
}

Close :: proc(maze: ^Maze, p: Vec2i, tile: i32 = 1) {
    if !IsValid(p) do return

    maze[p.x + p.y * MAZE_WIDTH] = tile
}

IsOpen :: proc(maze: ^Maze, p: Vec2i) -> bool {
    if !IsValid(p) do return false

    return maze[p.x + p.y * MAZE_WIDTH] == 0
}

GetItem :: proc(maze: ^Maze, p: Vec2i) -> i32 {
    if !IsValid(p) do return 0

    return maze[p.x + p.y * MAZE_WIDTH]
}

IsValid :: proc(p: Vec2i) -> bool {
    return p.x >= 0 && p.x < MAZE_WIDTH &&
           p.y >= 0 && p.y < MAZE_HEIGHT
}

Clear :: proc(maze: ^Maze) {
    for y in 1..<MAZE_HEIGHT - 1 {
        for x in 1..<MAZE_WIDTH -1 {
            maze[x + y * MAZE_WIDTH] = 0
        }
    }
}

PrintMaze :: proc(maze: ^Maze, title: string) {
    fmt.printf("\n%v:\n", title)
    for i in 0..<len(maze) {
        if maze[i] == 0 do fmt.print(' ')
        else do fmt.print(maze[i])
        if (i + 1) % MAZE_WIDTH == 0 do fmt.print('\n')
    }
}