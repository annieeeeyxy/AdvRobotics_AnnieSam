#include "bsp_iic.hpp"

int32_t Encoder_Offset[4];
int32_t Encoder_Now[4];

// I2C 写函数	I2C Write Function
int i2cWrite(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
  Wire1.beginTransmission(devAddr);
  Wire1.write(regAddr);
  for (uint8_t i = 0; i < length; i++) {
    Wire1.write(data[i]);
  }
  return Wire1.endTransmission();
}

// I2C 读函数	I2C Read Function
int i2cRead(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
  Wire1.beginTransmission(devAddr);
  Wire1.write(regAddr);
  int status = Wire1.endTransmission(false);
  if (status != 0) {
    return status;
  }

  int received = Wire1.requestFrom(devAddr, length);
  if (received != length) {
    return -1;
  }

  for (uint8_t i = 0; i < length; i++) {
    if (!Wire1.available()) {
      return -2;
    }
    data[i] = Wire1.read();
  }
  return 0;
}
// IIC 初始化	IIC Initialization
void IIC_Init(void)
{			
	Wire1.begin();
	Wire1.setClock(100000);
}
