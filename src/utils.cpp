#include "utils.hpp"

const int BUFFER_SIZE = 3;
float tempBuffer[BUFFER_SIZE];
int bufferIndex = 0;

float getSmoothedTemperature(float newTemp) {
    tempBuffer[bufferIndex] = newTemp;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
    float sum = 0.0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      sum += tempBuffer[i];
    }
    return sum / BUFFER_SIZE;
  }

void printCSVWithTimestamp(float* values, size_t size){
  Serial.print(millis()); // Print timestamp
  Serial.print(",");
  for (size_t i = 0; i < size; ++i) {
    Serial.print(values[i], 3);
    if (i < size - 1) Serial.print(",");
  }
  Serial.println();
}