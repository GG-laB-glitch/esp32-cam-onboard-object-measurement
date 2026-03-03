#pragma once

// =============================
// Measurement Mode
// =============================
extern bool g_use_mm;     // true = mm, false = pixels
extern float g_mm_per_px; // mm per pixel (e.g., 0.1)

// =============================
// Filtering Parameters
// =============================
extern int g_min_area;      // Minimum blob area in pixels
extern int g_max_area;      // Maximum blob area in pixels
extern int g_min_wh;        // Minimum width/height in pixels
extern int g_thresh_offset; // Threshold offset from calibration (fallback only)

extern uint32_t g_capture_index;

// =============================
// Image Enhancement
// =============================
extern bool g_apply_contrast;   // Enable contrast enhancement
extern int g_contrast_strength; // 0-100

// =============================
// Square Crop Settings
// =============================
extern bool g_use_crop; // Enable square cropping
extern int g_crop_size; // Size of square crop (e.g., 400)

// =============================
// Calibration Data (4x4 grid)
// =============================
#define CALIB_GRID 4 // 4x4 grid for better vignetting correction

extern bool g_calibrated;
extern uint8_t g_bg_mean[CALIB_GRID][CALIB_GRID];   // Per-cell background mean
extern uint8_t g_bg_stddev[CALIB_GRID][CALIB_GRID]; // Per-cell background stddev
extern float g_k_sigma;                             // Threshold = mean - k*stddev (default 2.5)

// =============================
// Locked Exposure (set after calibration)
// =============================
extern bool g_exposure_locked;
extern int g_locked_aec_value;
extern int g_locked_agc_gain;
