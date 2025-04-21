#pragma once

#include <Arduino.h>

void printCSVWithTimestamp(float* values, size_t size);

float getSmoothedTemperature(float newTemp);