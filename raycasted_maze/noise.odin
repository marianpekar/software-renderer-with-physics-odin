package raycasted_maze

import "core:math/rand"

Permutations :: [256]i32
Matrix2x2f :: [2][2]f32

Grad :: [2]f32
Grads := [8]Grad{
    {1, 1}, {-1, 1}, {1, -1}, {-1, -1},
    {1, 0}, {-1, 0}, {0,  1}, { 0, -1},
}

MakePermutations :: proc() -> Permutations {
    perms: Permutations

    for &p in perms {
        p = i32(rand.uint32() % 256)
    }

    return perms
}

SampleFractalBrownianMotion :: proc(
    perms: Permutations, 
    v: Vec2f,
    octaves: i32,
    persistence,
    low, high: f32
) -> f32 {
    s: f32 = 0
    fq: f32 = 1
    amp: f32 = 1
    max: f32 = 0

    for i in 0..<octaves {
        s = SamplePerlin(perms, {v.x * fq, v.y * fq}) * amp
        max += amp
        amp *= persistence
        fq *= 2
    }

    s /= max

    return Remap(s, -0.333, 0.333, low, high)

    Remap :: proc(val, min1, max1, min2, max2: f32) -> f32 {
        return min2 + (val - min1) * (max2 - min2) / (max1 - min1)
    }
}

SamplePerlin :: proc(perms: Permutations, v: Vec2f) -> f32 {
    u := Vec2i{i32(v.x), i32(v.y)}
    w := v - Vec2f{f32(u.x), f32(u.y)}

    u.x &= 255 
    u.y &= 255

    g := Grads[perms[(u.x + perms[u.y]) & 255] % 8]

    n: Matrix2x2f
    n[0][0] = Dot(g, w.x,     w.y)
    n[1][0] = Dot(g, w.x - 1, w.y)
    n[0][1] = Dot(g, w.x,     w.y - 1)
    n[1][1] = Dot(g, w.x - 1, w.y - 1)

    f := Vec2f{Fade(w.x), Fade(w.y)}

    l := Vec2f{
        Lerp(n[0][0], n[1][0], f.x),
        Lerp(n[0][1], n[1][1], f.x)
    }

    return Lerp(l.x, l.y, f.y)

    Dot :: proc(g: Grad, x, y: f32) -> f32 {
        return g[0] * x + g[1] * y
    }

    Fade :: proc(t: f32) -> f32 {
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
    }

    Lerp :: proc(a, b, t: f32) -> f32 {
       return (1.0 - t) * a + t * b
    }
}