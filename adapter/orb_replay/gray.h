#ifndef VIO_CAMERA_GRAY_H
#define VIO_CAMERA_GRAY_H
#include <stdint.h>
#include <stddef.h>
#include <arm_neon.h>

/* Raw dimensions include every delivered row. Cropping is always explicit. */
struct gray_layout {
    size_t width, raw_height, stride, crop_top, crop_bottom;
    size_t height, pixels, minimum_source_bytes;
};

static int gray_layout_init(struct gray_layout *g, size_t width, size_t height,
                            size_t stride, size_t top, size_t bottom) {
    if (!g || !width || !height || width % 2u || width > SIZE_MAX / 2u ||
        stride < width * 2u || top >= height || bottom >= height - top ||
        height - 1u > (SIZE_MAX - width * 2u) / stride ||
        height - top - bottom > SIZE_MAX / width)
        return -1;
    *g = (struct gray_layout){width, height, stride, top, bottom,
        height - top - bottom, width * (height - top - bottom),
        (height - 1u) * stride + width * 2u};
    return 0;
}

/* Validate the full raw frame, including cropped rows, before touching memory.
 * Padding after the final row need not be present in bytesused. */
static int extract_gray(uint8_t *gray, size_t gray_length, const uint8_t *src,
                        size_t used, size_t mapped_length, const struct gray_layout *g) {
    if (!g || !gray || !src || gray_length < g->pixels ||
        used < g->minimum_source_bytes || used > mapped_length)
        return -1;
    for (size_t y = 0; y < g->height; ++y) {
        const uint8_t *row = src + (g->crop_top + y) * g->stride;
        uint8_t *out = gray + y * g->width;
        size_t x = 0;
        for (; g->width - x >= 16u; x += 16u) {
            uint8x16x2_t pairs = vld2q_u8(row + 2u * x);
            vst1q_u8(out + x, pairs.val[1]);
        }
        for (; x < g->width; ++x) out[x] = row[2u * x + 1u];
    }
    return 0;
}
#endif

