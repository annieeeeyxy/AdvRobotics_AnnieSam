#ifndef IMU_MODULE_H
#define IMU_MODULE_H

void setupImu();
bool readImu();
void updateImuCached();
bool isImuReady();
float getYaw();
float getPitch();
float getRoll();
float angleError(float target, float current);

#endif
