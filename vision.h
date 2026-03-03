#pragma once
#include <Arduino.h>

// Structure forward declarations
struct OrientedBox
{
    float cx, cy;
    float theta;
    float xminr, xmaxr;
    float yminr, ymaxr;
    float length_px;
    float width_px;
};

// Core functions
void captureAndMeasure();
void processMeasurement(uint8_t *gray, int w, int h);

// Drawing functions
void drawOrientedBox(uint8_t *img, int w, int h, const OrientedBox &ob, uint8_t v = 0);
void drawMeasurementText(uint8_t *img, int w, int h, int x, int y, float L, float W, uint8_t color = 0);

// Crop function
uint8_t *cropCenterSquare(uint8_t *src, int src_w, int src_h, int crop_size, int &new_size);
