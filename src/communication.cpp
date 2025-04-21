#include "communication.hpp"

MCP_CAN CAN(SPI_CS_PIN);

void printCSVWithTimestamp(float *values, size_t size)
{
    Serial.print(millis()); // Print timestamp
    Serial.print(",");
    for (size_t i = 0; i < size; ++i)
    {
        Serial.print(values[i], 3);
        if (i < size - 1)
            Serial.print(",");
    }
    Serial.println();
}

void initCANBus()
{
    while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
        Serial.println("\n[!] CAN init failed. Retry ...");
        delay(100);
      }
      Serial.println("CAN init successful.");
      CAN.setMode(MCP_NORMAL);
}

void sendFloatVectorAsInt16(uint16_t canID, float x, float y, float z)
{
    byte buf[6];
    int16_t x_ = x * SCALE;
    int16_t y_ = y * SCALE;
    int16_t z_ = z * SCALE;

    buf[0] = highByte(x_);
    buf[1] = lowByte(x_);
    buf[2] = highByte(y_);
    buf[3] = lowByte(y_);
    buf[4] = highByte(z_);
    buf[5] = lowByte(z_);

    byte sndStat = CAN.sendMsgBuf(canID, 0, 6, buf);

    if (sndStat != CAN_OK) {
        Serial.println("\n[!] Sending message error (vector).");
      } 
}

void sendSingleFloatAsInt16(uint16_t canID, float value){
    int16_t val = value * SCALE;
    byte buf[2] = { highByte(val), lowByte(val) };
    byte sndStat = CAN.sendMsgBuf(canID, 0, 2, buf);

    if (sndStat != CAN_OK) {
        Serial.println("\n[!] Sending message error (single).");
      } 

}