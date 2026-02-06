#include <math.h>
#include <stdio.h>


float fast_inv_sqrt(float x) {
  long i;
  float y;
  float xhalf;

  xhalf = x * 0.5f;
  y = x;
  i = *(long*)&y; // evil floating point bit hack
  i = 0x5f3759df - (i>>1); // log of gamma constant and half by bit shift to right?
  y = *(float*)&i;
  y = y * (1.5f - (xhalf * y * y)); // Newton aproximation first iteration
  y = y * (1.5f - (xhalf * y * y)); // Second iteration for mor precision, can be removed

  return y;
}

int main() {
  float x = 256;
  float y = 1 / sqrt(x); // true sqrt
  float fast_y = fast_inv_sqrt(x); // faster aproximation
  printf("\nY = %f", y);
  printf("\nFast Y = %f", fast_y);
}
