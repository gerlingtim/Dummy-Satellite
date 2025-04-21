#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 10;
const float SCALE = 1000.0; // Scaling factor for data transmission


void printCSVWithTimestamp(float* values, size_t size);

void initCANBus();
void sendFloatVectorAsInt16(uint16_t canID, float x, float y, float z);
void sendSingleFloatAsInt16(uint16_t canID, float value);