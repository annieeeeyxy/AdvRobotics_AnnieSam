#ifndef HUSKYLENS_MODULE_H
#define HUSKYLENS_MODULE_H

struct YellowLineBlock {
  int xCenter;
  int yCenter;
  int width;
  int height;
  int id;
  int area;
};

void setupHuskylens();
bool readYellowLineBlock(YellowLineBlock& block, int& error);
void serviceHuskylensColorTest();

#endif
