package main

// ============================================================
// texture.odin
// CPU-side texture representation and loading.
//
// Because this is a software renderer all texturing is done in
// pure CPU code.  The texture pixels are loaded from disk into
// a regular CPU memory buffer (a pointer to an array of rl.Color
// values) so the rasteriser can index directly into the pixel data
// for every fragment it draws.
//
// GPU-side textures (rl.Texture2D) are never used for rendering —
// only for uploading the completed back-buffer each frame.
// ============================================================

import rl "vendor:raylib"

// Texture stores a decoded image's dimensions and a pointer to
// its raw pixel data in CPU memory.
//
// Fields:
//   width  — image width in pixels
//   height — image height in pixels
//   pixels — pointer to a flat array of rl.Color values in row-major
//             order: pixel(x,y) = pixels[y * width + x]
//
// The pixel array is allocated by Raylib (rl.LoadImageColors) and
// lives in heap memory until the program exits.  For the scope of
// this renderer it is never explicitly freed.
Texture :: struct {
    width:  i32,
    height: i32,
    pixels: [^]rl.Color
}

// LoadTextureFromFile loads an image from disk and extracts its pixels
// into CPU memory, returning a Texture ready for software rasterisation.
//
// Steps:
//   1. rl.LoadImage reads the image file from disk and decodes it into
//      an rl.Image (a CPU-side pixel buffer with format metadata).
//      Raylib supports PNG, BMP, TGA, JPG, and several other formats.
//   2. rl.LoadImageColors extracts the decoded pixels as a flat array
//      of RGBA rl.Color values (one per pixel, 4 bytes each), allocating
//      a new heap buffer.
//   3. rl.UnloadImage frees the intermediate rl.Image struct (the pixel
//      data has been transferred to the Colors array, so this is safe).
//   4. The Texture struct is returned holding the dimensions and the
//      pointer to the pixel array.
LoadTextureFromFile :: proc(filename: cstring) -> Texture {
    // Decode the image file into a CPU-side image struct.
    image := rl.LoadImage(filename)

    texture := Texture{
        width  = image.width,
        height = image.height,
        // Extract individual RGBA pixel values into a flat array.
        pixels = rl.LoadImageColors(image)
    }

    // Free the intermediate image data; pixels are now in texture.pixels.
    rl.UnloadImage(image)

    return texture
}
