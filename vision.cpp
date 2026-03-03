#include "vision.h"
#include "config.h"
#include "FS.h"
#include "SD_MMC.h"
#include <math.h>
#include <string.h>
#include "esp_camera.h"
#include "board_config.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

// ============================================================
// External variables from main .ino
// ============================================================
extern bool g_use_mm;
extern float g_mm_per_px;
extern int g_min_area;
extern int g_max_area;
extern int g_min_wh;
extern int g_thresh_offset;
extern uint32_t g_capture_index;
extern bool g_apply_contrast;
extern int g_contrast_strength;
extern bool g_use_crop;
extern int g_crop_size;
extern bool g_calibrated;

// FIX #6 / prev analysis: upgraded to 4x4 grid with mean+stddev
extern uint8_t g_bg_mean[CALIB_GRID][CALIB_GRID];
extern uint8_t g_bg_stddev[CALIB_GRID][CALIB_GRID];
extern float g_k_sigma;

// FIX #13: exposure lock state
extern bool g_exposure_locked;
extern int g_locked_aec_value;
extern int g_locked_agc_gain;

// ============================================================
// Structures
// ============================================================
struct Component
{
    uint32_t *pix;
    int count;
    int xmin, xmax, ymin, ymax;
};

// ============================================================
// Forward Declarations
// ============================================================
static bool floodComponent(uint8_t *bin, int w, int h, int sx, int sy, Component &c);
static void thresholdAdaptive(const uint8_t *gray, uint8_t *bin, int w, int h);
static void dilateEdges(uint8_t *edges, int w, int h, int iterations);
static void erodeEdges(uint8_t *edges, int w, int h, int iterations);
static void fillHoles(uint8_t *bin, int w, int h);
static void removeSmallBlobs(uint8_t *bin, int w, int h);
static bool computeOrientedBox(const Component &c, int w, OrientedBox &ob);

// ============================================================
// Watchdog: yield() is safe on any task without registration
// ============================================================
static void feedWatchdog()
{
    yield();

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
}

// ============================================================
// Utility
// ============================================================
static inline int clampi(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static float convertOutput(float px)
{
    if (!g_use_mm)
        return px;
    return px * g_mm_per_px;
}

// ============================================================
// CROP FUNCTION - Center square
// ============================================================
uint8_t *cropCenterSquare(uint8_t *src, int src_w, int src_h, int crop_size, int &new_size)
{
    if (crop_size <= 0 || crop_size > src_w || crop_size > src_h)
    {
        new_size = src_w;
        uint8_t *dst = (uint8_t *)ps_malloc(src_w * src_h);
        if (dst)
            memcpy(dst, src, src_w * src_h);
        return dst;
    }

    int crop_x = (src_w - crop_size) / 2;
    int crop_y = (src_h - crop_size) / 2;
    new_size = crop_size;

    uint8_t *cropped = (uint8_t *)ps_malloc(crop_size * crop_size);
    if (!cropped)
        return NULL;

    for (int y = 0; y < crop_size; y++)
    {
        memcpy(&cropped[y * crop_size], &src[(crop_y + y) * src_w + crop_x], crop_size);
        if (y % 50 == 0)
            feedWatchdog();
    }

    Serial.printf("Center cropped to %dx%d\n", crop_size, crop_size);
    return cropped;
}

// ============================================================
// SAVE IMAGE
// ============================================================
static void savePGM(const char *prefix, uint8_t *buf, int w, int h)
{
    char name[64];
    snprintf(name, sizeof(name), "/%s_%03lu.pgm", prefix, g_capture_index);

    File f = SD_MMC.open(name, FILE_WRITE);
    if (!f)
        return;

    f.printf("P5\n%d %d\n255\n", w, h);

    int bytes_written = 0;
    int total_bytes = w * h;
    const int chunk_size = 10240;

    while (bytes_written < total_bytes)
    {
        int write_size = min(chunk_size, total_bytes - bytes_written);
        f.write(buf + bytes_written, write_size);
        bytes_written += write_size;
        feedWatchdog();
    }

    f.close();
    Serial.printf("Saved: %s\n", name);
}

// ============================================================
// CONTRAST ENHANCEMENT
// ============================================================
static void enhanceContrast(uint8_t *img, int size, int strength)
{
    if (!g_apply_contrast || strength <= 0)
        return;

    uint8_t min_val = 255, max_val = 0;
    for (int i = 0; i < size; i++)
    {
        if (img[i] < min_val)
            min_val = img[i];
        if (img[i] > max_val)
            max_val = img[i];
        if (i % 10000 == 0)
            feedWatchdog();
    }

    int range = max_val - min_val;
    if (range < 10)
        return;

    float scale = 255.0f / range;
    for (int i = 0; i < size; i++)
    {
        int enhanced = (int)((img[i] - min_val) * scale);
        enhanced = clampi(enhanced, 0, 255);
        int result = (img[i] * (100 - strength) + enhanced * strength) / 100;
        img[i] = (uint8_t)clampi(result, 0, 255);
        if (i % 10000 == 0)
            feedWatchdog();
    }

    Serial.printf("Contrast enhanced: range was %d\n", range);
}

// ============================================================
// FIX #6 + prev analysis improvements:
// ADAPTIVE THRESHOLD using 4x4 grid + per-cell mean/stddev
// + bilinear interpolation to avoid hard zone boundaries
// ============================================================
static void thresholdAdaptive(const uint8_t *gray, uint8_t *bin, int w, int h)
{
    // --- Always print image stats so we can diagnose detection failures ---
    uint32_t img_sum = 0;
    uint8_t img_min = 255, img_max = 0;
    for (int i = 0; i < w * h; i++)
    {
        img_sum += gray[i];
        if (gray[i] < img_min)
            img_min = gray[i];
        if (gray[i] > img_max)
            img_max = gray[i];
        if (i % 20000 == 0)
            feedWatchdog();
    }
    uint8_t img_mean = (uint8_t)(img_sum / (w * h));
    Serial.printf("  Image stats: min=%d, max=%d, mean=%d, range=%d\n",
                  img_min, img_max, img_mean, img_max - img_min);

    if (!g_calibrated)
    {
        Serial.println("WARNING: Not calibrated! Using global mean threshold.");
        uint8_t T = (uint8_t)clampi(img_mean - g_thresh_offset, 0, 255);
        Serial.printf("  Fallback threshold T=%d (mean %d - offset %d)\n",
                      T, img_mean, g_thresh_offset);
        int fg = 0;
        for (int i = 0; i < w * h; i++)
        {
            bin[i] = (gray[i] < T) ? 255 : 0;
            if (bin[i])
                fg++;
            if (i % 10000 == 0)
                feedWatchdog();
        }
        Serial.printf("  Foreground pixels after threshold: %d / %d (%.1f%%)\n",
                      fg, w * h, 100.0f * fg / (w * h));
        return;
    }

    Serial.println("Thresholding with 4x4 adaptive calibration (bilinear)...");

    // Build per-cell threshold: T = mean - k*stddev
    // GUARD: if stddev is 0 (uniform background), fall back to mean - offset
    float cell_T[CALIB_GRID][CALIB_GRID];
    Serial.println("  Thresholds per cell:");
    for (int gy = 0; gy < CALIB_GRID; gy++)
    {
        for (int gx = 0; gx < CALIB_GRID; gx++)
        {
            float t;
            if (g_bg_stddev[gy][gx] < 2)
            {
                // stddev too small — background was too uniform or calibration bad
                // use mean - fallback offset instead
                t = g_bg_mean[gy][gx] - g_thresh_offset;
                Serial.printf("    [%d,%d] mean=%3d std=%d(low)->T=%3d(fallback)\n",
                              gx, gy, g_bg_mean[gy][gx], g_bg_stddev[gy][gx], (int)t);
            }
            else
            {
                t = g_bg_mean[gy][gx] - g_k_sigma * g_bg_stddev[gy][gx];
                Serial.printf("    [%d,%d] mean=%3d std=%3d k=%.1f -> T=%3d\n",
                              gx, gy, g_bg_mean[gy][gx], g_bg_stddev[gy][gx],
                              g_k_sigma, (int)t);
            }
            cell_T[gy][gx] = (float)clampi((int)t, 0, 255);
        }
    }

    // SANITY CHECK: if ALL thresholds are 0 something is wrong
    float tmax = 0;
    for (int gy = 0; gy < CALIB_GRID; gy++)
        for (int gx = 0; gx < CALIB_GRID; gx++)
            if (cell_T[gy][gx] > tmax)
                tmax = cell_T[gy][gx];

    if (tmax < 10)
    {
        Serial.println("  WARNING: All calibration thresholds near 0!");
        Serial.println("  Background may have been too dark, or k_sigma too high.");
        Serial.println("  Falling back to global mean threshold.");
        uint8_t T = (uint8_t)clampi(img_mean - g_thresh_offset, 0, 255);
        Serial.printf("  Fallback T=%d\n", T);
        int fg = 0;
        for (int i = 0; i < w * h; i++)
        {
            bin[i] = (gray[i] < T) ? 255 : 0;
            if (bin[i])
                fg++;
            if (i % 10000 == 0)
                feedWatchdog();
        }
        Serial.printf("  Foreground pixels: %d / %d (%.1f%%)\n",
                      fg, w * h, 100.0f * fg / (w * h));
        return;
    }

    // Bilinear interpolation of threshold across image
    int fg_count = 0;
    for (int y = 0; y < h; y++)
    {
        float fy = (float)y * (CALIB_GRID - 1) / (h > 1 ? h - 1 : 1);
        int gy0 = (int)fy;
        int gy1 = min(gy0 + 1, CALIB_GRID - 1);
        float ty = fy - gy0;

        for (int x = 0; x < w; x++)
        {
            float fx = (float)x * (CALIB_GRID - 1) / (w > 1 ? w - 1 : 1);
            int gx0 = (int)fx;
            int gx1 = min(gx0 + 1, CALIB_GRID - 1);
            float tx = fx - gx0;

            float T =
                cell_T[gy0][gx0] * (1 - tx) * (1 - ty) +
                cell_T[gy0][gx1] * (tx) * (1 - ty) +
                cell_T[gy1][gx0] * (1 - tx) * (ty) +
                cell_T[gy1][gx1] * (tx) * (ty);

            uint8_t pix = gray[y * w + x];
            bin[y * w + x] = (pix < (uint8_t)T) ? 255 : 0;
            if (bin[y * w + x])
                fg_count++;
        }

        if (y % 30 == 0)
            feedWatchdog();
    }

    Serial.printf("  Foreground pixels after threshold: %d / %d (%.1f%%)\n",
                  fg_count, w * h, 100.0f * fg_count / (w * h));

    // GUIDE: if fg > 40% something is wrong (threshold too aggressive)
    if ((float)fg_count / (w * h) > 0.40f)
        Serial.println("  WARNING: >40% foreground - threshold may be too high. Try KSIGMA higher or RECALIBRATE.");
    if (fg_count == 0)
        Serial.println("  WARNING: 0 foreground pixels - threshold too low or object same brightness as background.");
}

// ============================================================
// DILATE
// ============================================================
static void dilateEdges(uint8_t *edges, int w, int h, int iterations)
{
    uint8_t *temp = (uint8_t *)ps_malloc(w * h);
    if (!temp)
        return;

    for (int iter = 0; iter < iterations; iter++)
    {
        memcpy(temp, edges, w * h);
        feedWatchdog();

        for (int y = 1; y < h - 1; y++)
        {
            for (int x = 1; x < w - 1; x++)
            {
                if (temp[y * w + x] == 255)
                {
                    int base = (y - 1) * w + (x - 1);
                    edges[base] = 255;
                    edges[base + 1] = 255;
                    edges[base + 2] = 255;
                    edges[base + w] = 255;
                    edges[base + w + 1] = 255;
                    edges[base + w + 2] = 255;
                    edges[base + 2 * w] = 255;
                    edges[base + 2 * w + 1] = 255;
                    edges[base + 2 * w + 2] = 255;
                }
            }
            if (y % 30 == 0)
                feedWatchdog();
        }
    }
    free(temp);
}

// ============================================================
// FIX #11: ERODE (to balance dilation and keep accurate dims)
// ============================================================
static void erodeEdges(uint8_t *edges, int w, int h, int iterations)
{
    uint8_t *temp = (uint8_t *)ps_malloc(w * h);
    if (!temp)
        return;

    for (int iter = 0; iter < iterations; iter++)
    {
        memcpy(temp, edges, w * h);
        feedWatchdog();

        for (int y = 1; y < h - 1; y++)
        {
            for (int x = 1; x < w - 1; x++)
            {
                // A foreground pixel is eroded away if ANY neighbor is background
                if (temp[y * w + x] == 255)
                {
                    bool border = (temp[(y - 1) * w + x] == 0 || temp[(y + 1) * w + x] == 0 ||
                                   temp[y * w + (x - 1)] == 0 || temp[y * w + (x + 1)] == 0);
                    if (border)
                        edges[y * w + x] = 0;
                }
            }
            if (y % 30 == 0)
                feedWatchdog();
        }
    }
    free(temp);
}

// ============================================================
// FIX #1 + #4: FLOOD COMPONENT - queue allocated in PSRAM,
// sized to w*h to prevent overflow and silent pixel loss
// ============================================================
static bool floodComponent(uint8_t *bin, int w, int h, int sx, int sy, Component &c)
{
    if (bin[sy * w + sx] != 255)
        return false;

    // FIX: allocate queue in PSRAM, large enough for worst case
    uint32_t *queue = (uint32_t *)ps_malloc(sizeof(uint32_t) * w * h);
    if (!queue)
        return false;

    c.pix = (uint32_t *)ps_malloc(sizeof(uint32_t) * 2048);
    if (!c.pix)
    {
        free(queue);
        return false;
    }

    c.count = 0;
    c.xmin = c.xmax = sx;
    c.ymin = c.ymax = sy;
    int capacity = 2048;

    int qh = 0, qt = 0;
    queue[qt++] = (uint32_t)(sy * w + sx);
    bin[sy * w + sx] = 0;

    while (qh < qt)
    {
        uint32_t idx = queue[qh++];
        int y = idx / w;
        int x = idx % w;

        // Grow pixel array if needed
        if (c.count >= capacity)
        {
            int new_cap = capacity * 2;
            if (new_cap > (w * h))
                new_cap = w * h; // hard cap
            uint32_t *np = (uint32_t *)ps_realloc(c.pix, sizeof(uint32_t) * new_cap);
            if (!np)
            {
                free(c.pix);
                free(queue);
                c.pix = NULL;
                return false;
            }
            c.pix = np;
            capacity = new_cap;
        }

        c.pix[c.count++] = idx;

        if (x < c.xmin)
            c.xmin = x;
        if (x > c.xmax)
            c.xmax = x;
        if (y < c.ymin)
            c.ymin = y;
        if (y > c.ymax)
            c.ymax = y;

        // 4-connected neighbors
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; k++)
        {
            int nx = x + dx[k];
            int ny = y + dy[k];
            if (nx >= 0 && nx < w && ny >= 0 && ny < h)
            {
                int n = ny * w + nx;
                if (bin[n] == 255)
                {
                    bin[n] = 0;
                    queue[qt++] = (uint32_t)n; // safe: queue is w*h sized
                }
            }
        }

        if (c.count % 2000 == 0)
            feedWatchdog();
    }

    free(queue);
    return (c.count > 0);
}

// ============================================================
// REMOVE SMALL BLOBS
// ============================================================
static void removeSmallBlobs(uint8_t *bin, int w, int h)
{
    uint8_t *copy = (uint8_t *)ps_malloc(w * h);
    if (!copy)
        return;
    memcpy(copy, bin, w * h);
    feedWatchdog();

    int removed = 0;
    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < w; x++)
        {
            if (copy[y * w + x] != 255)
                continue;

            Component c;
            if (!floodComponent(copy, w, h, x, y, c))
                continue;

            if (c.count < g_min_area)
            {
                for (int i = 0; i < c.count; i++)
                    bin[c.pix[i]] = 0;
                removed++;
            }
            if (c.pix)
                free(c.pix);
        }
        if (y % 30 == 0)
            feedWatchdog();
    }

    if (removed > 0)
        Serial.printf("Removed %d small blobs\n", removed);
    free(copy);
}

// ============================================================
// FIX #4: FILL HOLES - queue allocated in PSRAM (w*h sized)
// ============================================================
static void fillHoles(uint8_t *bin, int w, int h)
{
    uint8_t *visited = (uint8_t *)ps_calloc(w * h, 1);
    if (!visited)
        return;

    // FIX: PSRAM queue sized to w*h - no overflow possible
    uint32_t *queue = (uint32_t *)ps_malloc(sizeof(uint32_t) * w * h);
    if (!queue)
    {
        free(visited);
        return;
    }

    int qh = 0, qt = 0;

    // Seed from all border background pixels
    for (int x = 0; x < w; x++)
    {
        if (bin[x] == 0 && !visited[x])
        {
            visited[x] = 1;
            queue[qt++] = x;
        }
        int bot = (h - 1) * w + x;
        if (bin[bot] == 0 && !visited[bot])
        {
            visited[bot] = 1;
            queue[qt++] = bot;
        }
    }
    for (int y = 1; y < h - 1; y++)
    {
        int lft = y * w;
        int rgt = y * w + (w - 1);
        if (bin[lft] == 0 && !visited[lft])
        {
            visited[lft] = 1;
            queue[qt++] = lft;
        }
        if (bin[rgt] == 0 && !visited[rgt])
        {
            visited[rgt] = 1;
            queue[qt++] = rgt;
        }
        if (y % 50 == 0)
            feedWatchdog();
    }

    // BFS outward from border
    int processed = 0;
    while (qh < qt)
    {
        uint32_t idx = queue[qh++];
        int y = idx / w;
        int x = idx % w;

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; k++)
        {
            int nx = x + dx[k];
            int ny = y + dy[k];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h)
                continue;
            int n = ny * w + nx;
            if (bin[n] == 0 && !visited[n])
            {
                visited[n] = 1;
                queue[qt++] = n; // safe: queue is w*h
            }
        }
        processed++;
        if (processed % 2000 == 0)
            feedWatchdog();
    }

    // Any unvisited background pixel is an enclosed hole — fill it
    int holes = 0;
    for (int i = 0; i < w * h; i++)
    {
        if (bin[i] == 0 && !visited[i])
        {
            bin[i] = 255;
            holes++;
        }
        if (i % 10000 == 0)
            feedWatchdog();
    }

    if (holes > 0)
        Serial.printf("Filled %d holes\n", holes);
    free(visited);
    free(queue);
}

// ============================================================
// PCA ORIENTED BOX
// ============================================================
static bool computeOrientedBox(const Component &c, int w, OrientedBox &ob)
{
    if (c.count == 0)
        return false;

    double sx = 0, sy = 0;
    for (int i = 0; i < c.count; i++)
    {
        sx += c.pix[i] % w;
        sy += c.pix[i] / w;
    }
    ob.cx = (float)(sx / c.count);
    ob.cy = (float)(sy / c.count);

    double mu20 = 0, mu02 = 0, mu11 = 0;
    for (int i = 0; i < c.count; i++)
    {
        double dx = (c.pix[i] % w) - ob.cx;
        double dy = (c.pix[i] / w) - ob.cy;
        mu20 += dx * dx;
        mu02 += dy * dy;
        mu11 += dx * dy;
        if (i % 1000 == 0)
            feedWatchdog();
    }

    if (mu20 == 0 && mu02 == 0)
        return false;
    ob.theta = (float)(0.5 * atan2(2.0 * mu11, mu20 - mu02));

    float cs = cosf(ob.theta);
    float sn = sinf(ob.theta);
    float xminr = 1e9f, xmaxr = -1e9f;
    float yminr = 1e9f, ymaxr = -1e9f;

    for (int i = 0; i < c.count; i++)
    {
        float dx = (float)(c.pix[i] % w) - ob.cx;
        float dy = (float)(c.pix[i] / w) - ob.cy;
        float xr = dx * cs + dy * sn;
        float yr = -dx * sn + dy * cs;
        if (xr < xminr)
            xminr = xr;
        if (xr > xmaxr)
            xmaxr = xr;
        if (yr < yminr)
            yminr = yr;
        if (yr > ymaxr)
            ymaxr = yr;
        if (i % 1000 == 0)
            feedWatchdog();
    }

    ob.xminr = xminr;
    ob.xmaxr = xmaxr;
    ob.yminr = yminr;
    ob.ymaxr = ymaxr;

    float a = xmaxr - xminr;
    float b = ymaxr - yminr;
    ob.length_px = fmaxf(a, b);
    ob.width_px = fminf(a, b);

    return true;
}

// ============================================================
// DRAW ORIENTED BOX
// ============================================================
// Draw a single Bresenham line with given thickness (in pixels)
// ============================================================
static void drawThickLine(uint8_t *img, int w, int h,
                          int x1, int y1, int x2, int y2,
                          int thickness, uint8_t color)
{
    int ddx = abs(x2 - x1), ddy = abs(y2 - y1);
    int sx = x1 < x2 ? 1 : -1;
    int sy = y1 < y2 ? 1 : -1;
    int err = ddx - ddy, steps = 0;
    int half = thickness / 2;

    while (true)
    {
        // Paint a small filled square of (thickness x thickness) centred on each pixel
        for (int ty = -half; ty <= half; ty++)
            for (int tx = -half; tx <= half; tx++)
            {
                int px = clampi(x1 + tx, 0, w - 1);
                int py = clampi(y1 + ty, 0, h - 1);
                img[py * w + px] = color;
            }

        if (x1 == x2 && y1 == y2)
            break;
        int e2 = 2 * err;
        if (e2 > -ddy)
        {
            err -= ddy;
            x1 += sx;
        }
        if (e2 < ddx)
        {
            err += ddx;
            y1 += sy;
        }
        if (++steps % 100 == 0)
            feedWatchdog();
    }
}

// ============================================================
// Draw oriented bounding box — BLACK (value 0), 3 px thick
// ============================================================
void drawOrientedBox(uint8_t *img, int w, int h, const OrientedBox &ob, uint8_t /*unused*/)
{
    // Color is always BLACK (0) for maximum contrast on white background
    const uint8_t BOX_COLOR = 0;
    const int BOX_THICKNESS = 3;

    float cs = cosf(ob.theta);
    float sn = sinf(ob.theta);

    float corners[4][2] = {
        {ob.xminr, ob.yminr}, {ob.xmaxr, ob.yminr}, {ob.xmaxr, ob.ymaxr}, {ob.xminr, ob.ymaxr}};
    int ic[4][2];

    for (int i = 0; i < 4; i++)
    {
        float dx = corners[i][0] * cs - corners[i][1] * sn;
        float dy = corners[i][0] * sn + corners[i][1] * cs;
        ic[i][0] = clampi((int)(ob.cx + dx), 0, w - 1);
        ic[i][1] = clampi((int)(ob.cy + dy), 0, h - 1);
    }

    for (int i = 0; i < 4; i++)
        drawThickLine(img, w, h,
                      ic[i][0], ic[i][1],
                      ic[(i + 1) % 4][0], ic[(i + 1) % 4][1],
                      BOX_THICKNESS, BOX_COLOR);
}

// ============================================================
// 7x9 bitmap font for digits 0-9 (each row is a bitmask, MSB=left)
// Each digit is 7 wide x 9 tall.
// ============================================================
static const uint8_t DIGIT_FONT[10][9] = {
    // 0
    {0b01111100, 0b11000110, 0b11001110, 0b11011110, 0b11110110,
     0b11100110, 0b11000110, 0b01111100, 0b00000000},
    // 1
    {0b00110000, 0b01110000, 0b00110000, 0b00110000, 0b00110000,
     0b00110000, 0b00110000, 0b11111100, 0b00000000},
    // 2
    {0b01111000, 0b11001100, 0b00001100, 0b00011000, 0b00110000,
     0b01100000, 0b11001100, 0b11111100, 0b00000000},
    // 3
    {0b01111000, 0b11001100, 0b00001100, 0b00111000, 0b00001100,
     0b00001100, 0b11001100, 0b01111000, 0b00000000},
    // 4
    {0b00011100, 0b00111100, 0b01101100, 0b11001100, 0b11111110,
     0b00001100, 0b00001100, 0b00011110, 0b00000000},
    // 5
    {0b11111100, 0b11000000, 0b11000000, 0b11111000, 0b00001100,
     0b00001100, 0b11001100, 0b01111000, 0b00000000},
    // 6
    {0b00111000, 0b01100000, 0b11000000, 0b11111000, 0b11001100,
     0b11001100, 0b11001100, 0b01111000, 0b00000000},
    // 7
    {0b11111110, 0b11000110, 0b00000110, 0b00001100, 0b00011000,
     0b00110000, 0b00110000, 0b00110000, 0b00000000},
    // 8
    {0b01111000, 0b11001100, 0b11001100, 0b01111000, 0b11001100,
     0b11001100, 0b11001100, 0b01111000, 0b00000000},
    // 9
    {0b01111000, 0b11001100, 0b11001100, 0b11001100, 0b01111100,
     0b00001100, 0b00011000, 0b01110000, 0b00000000},
};

// Draw a single digit at (ox, oy) scaled by 'scale' (1=7x9, 2=14x18, 3=21x27)
// Colour BLACK (0) with a white halo for legibility on any background
static void drawDigit(uint8_t *img, int w, int h,
                      int ox, int oy, int digit, int scale)
{
    digit = clampi(digit, 0, 9);
    const int FONT_W = 7;
    const int FONT_H = 9;

    // First pass: white halo (1px border around each filled pixel)
    for (int row = 0; row < FONT_H; row++)
    {
        uint8_t mask = DIGIT_FONT[digit][row];
        for (int col = 0; col < FONT_W; col++)
        {
            if (!(mask & (0x80 >> col)))
                continue;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++)
                {
                    int px = ox + col * scale + sx;
                    int py = oy + row * scale + sy;
                    // paint halo
                    for (int hy = -1; hy <= 1; hy++)
                        for (int hx = -1; hx <= 1; hx++)
                        {
                            int hpx = clampi(px + hx, 0, w - 1);
                            int hpy = clampi(py + hy, 0, h - 1);
                            img[hpy * w + hpx] = 255; // white halo
                        }
                }
        }
    }

    // Second pass: black fill
    for (int row = 0; row < FONT_H; row++)
    {
        uint8_t mask = DIGIT_FONT[digit][row];
        for (int col = 0; col < FONT_W; col++)
        {
            if (!(mask & (0x80 >> col)))
                continue;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++)
                {
                    int px = clampi(ox + col * scale + sx, 0, w - 1);
                    int py = clampi(oy + row * scale + sy, 0, h - 1);
                    img[py * w + px] = 0; // black digit
                }
        }
    }
}

// Draw integer number (multi-digit) centred at (cx, cy)
// scale=3 gives approx 21x27px per digit — clearly visible
static void drawObjectNumber(uint8_t *img, int w, int h,
                             int cx, int cy, int number, int scale)
{
    // Break number into digits (max 3 digits = 999 objects)
    int digits[3];
    int ndig = 0;
    if (number >= 100)
    {
        digits[ndig++] = number / 100;
    }
    if (number >= 10)
    {
        digits[ndig++] = (number / 10) % 10;
    }
    digits[ndig++] = number % 10;

    const int FONT_W = 7;
    const int FONT_H = 9;
    int total_w = ndig * FONT_W * scale + (ndig - 1) * scale; // 1px gap between digits
    int total_h = FONT_H * scale;

    int start_x = cx - total_w / 2;
    int start_y = cy - total_h / 2;

    for (int i = 0; i < ndig; i++)
    {
        int ox = start_x + i * (FONT_W * scale + scale);
        drawDigit(img, w, h, ox, start_y, digits[i], scale);
    }
}

// ============================================================
// Public API: draw object number label centred at (x, y)
// Replaces old crosshair / measurement text function
// ============================================================
void drawMeasurementText(uint8_t *img, int w, int h,
                         int x, int y, float /*L*/, float /*W*/,
                         uint8_t /*unused*/)
{
    // This signature is kept for API compatibility.
    // Actual numbering is done via drawObjectNumber() called from
    // processMeasurement() which passes the object index directly.
    // Nothing drawn here — see call site below.
    (void)img;
    (void)w;
    (void)h;
    (void)x;
    (void)y;
}

// Real label drawing — called directly from processMeasurement
static void labelObject(uint8_t *img, int iw, int ih,
                        int cx, int cy, int number)
{
    // Scale 3 = 21x27 px digits, very legible at VGA/SVGA
    drawObjectNumber(img, iw, ih, cx, cy, number, 3);
}

// ============================================================
// MAIN PROCESS
// FIX #8: Correct morphological order:
//   threshold → fillHoles → erode → dilate → removeSmallBlobs
// ============================================================
void processMeasurement(uint8_t *gray, int w, int h)
{
    Serial.printf("Processing %dx%d image...\n", w, h);
    feedWatchdog();

    // --- Crop ---
    uint8_t *working_img = NULL;
    int working_w = w, working_h = h;

    if (g_use_crop)
    {
        int new_size;
        working_img = cropCenterSquare(gray, w, h, g_crop_size, new_size);
        if (working_img)
        {
            working_w = new_size;
            working_h = new_size;
        }
        else
        {
            Serial.println("Crop failed, using full image");
            working_img = (uint8_t *)ps_malloc(w * h);
            if (!working_img)
                return;
            memcpy(working_img, gray, w * h);
        }
    }
    else
    {
        working_img = (uint8_t *)ps_malloc(w * h);
        if (!working_img)
            return;
        memcpy(working_img, gray, w * h);
    }
    feedWatchdog();

    // --- Contrast ---
    if (g_apply_contrast)
        enhanceContrast(working_img, working_w * working_h, g_contrast_strength);
    feedWatchdog();

    size_t size = working_w * working_h;

    uint8_t *binary = (uint8_t *)ps_malloc(size);
    uint8_t *binCopy = (uint8_t *)ps_malloc(size);
    uint8_t *finalImg = (uint8_t *)ps_malloc(size);

    if (!binary || !binCopy || !finalImg)
    {
        Serial.println("Memory allocation failed!");
        free(working_img);
        if (binary)
            free(binary);
        if (binCopy)
            free(binCopy);
        if (finalImg)
            free(finalImg);
        return;
    }
    feedWatchdog();

    if (g_use_crop)
        savePGM("cropped", working_img, working_w, working_h);
    else
        savePGM("original", working_img, working_w, working_h);
    feedWatchdog();

    // --- Adaptive threshold (4x4 grid + bilinear) ---
    thresholdAdaptive(working_img, binary, working_w, working_h);
    feedWatchdog();

    // --- FIX #8: correct morphological sequence ---
    Serial.println("Morphological cleanup...");
    fillHoles(binary, working_w, working_h); // 1. close internal gaps first
    feedWatchdog();
    erodeEdges(binary, working_w, working_h, 1); // 2. erode to remove noise rim
    feedWatchdog();
    dilateEdges(binary, working_w, working_h, 1); // 3. re-dilate to restore size
    feedWatchdog();
    removeSmallBlobs(binary, working_w, working_h); // 4. remove specks
    feedWatchdog();

    savePGM("binary", binary, working_w, working_h);
    feedWatchdog();

    memcpy(binCopy, binary, size);
    memcpy(finalImg, working_img, size);
    feedWatchdog();

    int objectCount = 0;
    int rejectedTooSmall = 0, rejectedTooBig = 0;
    float totalLength = 0, totalWidth = 0;

    Serial.println("\n--- Blob Detection ---");
    Serial.printf("Area filter: min=%d  max=%d px\n", g_min_area, g_max_area);

    for (int y = 0; y < working_h; y++)
    {
        for (int x = 0; x < working_w; x++)
        {
            if (binCopy[y * working_w + x] != 255)
                continue;

            Component c;
            if (!floodComponent(binCopy, working_w, working_h, x, y, c))
                continue;

            // Report every blob found (accepted or rejected) for diagnosis
            Serial.printf("  Blob at (%d,%d): area=%d px  bbox=[%d,%d to %d,%d]",
                          x, y, c.count, c.xmin, c.ymin, c.xmax, c.ymax);

            if (c.count < g_min_area)
            {
                Serial.printf(" -> REJECTED (too small, min=%d)\n", g_min_area);
                rejectedTooSmall++;
                if (c.pix)
                    free(c.pix);
                continue;
            }
            if (c.count > g_max_area)
            {
                Serial.printf(" -> REJECTED (too large, max=%d)\n", g_max_area);
                rejectedTooBig++;
                if (c.pix)
                    free(c.pix);
                continue;
            }
            Serial.println(" -> ACCEPTED");

            OrientedBox ob;
            if (computeOrientedBox(c, working_w, ob))
            {
                objectCount++; // increment first so label starts at 1

                // Draw BLACK box (colour arg is ignored, always 0 inside function)
                drawOrientedBox(finalImg, working_w, working_h, ob, 0);
                feedWatchdog();

                // Draw bold object number centred on the object
                labelObject(finalImg, working_w, working_h,
                            (int)ob.cx, (int)ob.cy, objectCount);
                feedWatchdog();

                float L = convertOutput(ob.length_px);
                float W = convertOutput(ob.width_px);
                totalLength += L;
                totalWidth += W;

                Serial.printf("  -> Object %d: L=%.2f %s, W=%.2f %s\n",
                              objectCount,
                              L, g_use_mm ? "mm" : "px",
                              W, g_use_mm ? "mm" : "px");
            }

            if (c.pix)
                free(c.pix);
            if (objectCount % 5 == 0)
                feedWatchdog();
        }
        if (y % 20 == 0)
            feedWatchdog();
    }

    Serial.printf("\n--- Results ---\n");
    Serial.printf("Valid objects:    %d\n", objectCount);
    Serial.printf("Rejected (small): %d  (increase g_min_area or check threshold)\n", rejectedTooSmall);
    Serial.printf("Rejected (large): %d  (decrease g_max_area or object fills frame)\n", rejectedTooBig);
    if (rejectedTooSmall > 0 && objectCount == 0)
        Serial.println("HINT: All blobs too small. Try: MINAREA 50  or check lighting/calibration.");
    if (rejectedTooBig > 0 && objectCount == 0)
        Serial.println("HINT: All blobs too large. Try: MAXAREA 200000  or use CROP to isolate object.");
    if (objectCount > 0 && g_use_mm)
        Serial.printf("Avg L: %.2f mm, Avg W: %.2f mm\n",
                      totalLength / objectCount, totalWidth / objectCount);

    savePGM(g_use_crop ? "final_cropped" : "final", finalImg, working_w, working_h);

    free(working_img);
    free(binary);
    free(binCopy);
    free(finalImg);

    g_capture_index++;
    feedWatchdog();
}

// ============================================================
// FIX #9: Warm-up frames before actual capture
// FIX #13: Capture with locked exposure
// ============================================================
void captureAndMeasure()
{
    Serial.println("\n--- Capture Start ---");
    Serial.printf("Calibrated: %s\n", g_calibrated ? "YES" : "NO");

    if (!g_calibrated)
        Serial.println("WARNING: Not calibrated! Type CALIBRATE first.");

    // FIX #13: Re-apply locked exposure before capture
    if (g_exposure_locked)
    {
        sensor_t *s = esp_camera_sensor_get();
        if (s)
        {
            s->set_exposure_ctrl(s, 0);
            s->set_aec_value(s, g_locked_aec_value);
            s->set_gain_ctrl(s, 0);
            s->set_agc_gain(s, g_locked_agc_gain);
            Serial.printf("Exposure locked: AEC=%d, gain=%d\n",
                          g_locked_aec_value, g_locked_agc_gain);
        }
    }

    // FIX #9: Discard warm-up frames so AEC has settled
    Serial.println("Warming up (5 frames)...");
    for (int i = 0; i < 5; i++)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb)
            esp_camera_fb_return(fb);
        delay(60);
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Capture failed!");
        return;
    }

    Serial.printf("Image: %dx%d, %d bytes\n", fb->width, fb->height, fb->len);

    if (fb->len == 0 || fb->width == 0 || fb->height == 0)
    {
        Serial.println("Invalid image data!");
        esp_camera_fb_return(fb);
        return;
    }

    processMeasurement(fb->buf, fb->width, fb->height);
    esp_camera_fb_return(fb);
    Serial.println("--- Complete ---\n");
}
