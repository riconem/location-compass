#include <Arduino.h>
#include <BasicLinearAlgebra.h>

// max calibration data points
#define CALIBRATION_LENGTH 300

// x scale, y scale, z scale, x offset, y offset, z, offset
#define CALIBRATION_AXIS 6

// calibration delay in milliseconds between each measurement
#define CALIBRATION_DELAY 100

using namespace BLA;

void calibrate(int* x, int* y, int* z, float offsets[], float scale[], bool debug = false) {
  // Dynamically allocate memory for H_T and w matrices
  BLA::Matrix<CALIBRATION_AXIS, CALIBRATION_LENGTH>* H_T_ptr = new BLA::Matrix<CALIBRATION_AXIS, CALIBRATION_LENGTH>();
  BLA::Matrix<CALIBRATION_AXIS, CALIBRATION_LENGTH>& H_T = *H_T_ptr;

  BLA::Matrix<CALIBRATION_LENGTH>* w_ptr = new BLA::Matrix<CALIBRATION_LENGTH>();
  BLA::Matrix<CALIBRATION_LENGTH>& w = *w_ptr;

  for (int i = 0; i < CALIBRATION_LENGTH; i++) {
    H_T(0, i) = x[i];
    H_T(1, i) = y[i];
    H_T(2, i) = z[i];
    H_T(3, i) = -y[i] * y[i];
    H_T(4, i) = -z[i] * z[i];
    H_T(5, i) = 1.0;
    w(i) = x[i] * x[i];
  }

  delete x;
  delete y;
  delete z;

  BLA::Matrix<CALIBRATION_LENGTH, CALIBRATION_AXIS>* H_ptr = new BLA::Matrix<CALIBRATION_LENGTH, CALIBRATION_AXIS>();
  BLA::Matrix<CALIBRATION_LENGTH, CALIBRATION_AXIS>& H = *H_ptr;

  BLA::Matrix<CALIBRATION_AXIS, CALIBRATION_AXIS>* H_TH_ptr = new BLA::Matrix<CALIBRATION_AXIS, CALIBRATION_AXIS>();
  BLA::Matrix<CALIBRATION_AXIS, CALIBRATION_AXIS>& H_TH = *H_TH_ptr;

  // Transpose H_T
  H = ~H_T;
  H_TH = H_T * H;
  
  delete H_ptr;

  BLA::Matrix<CALIBRATION_AXIS>* b_ptr = new BLA::Matrix<CALIBRATION_AXIS>();
  BLA::Matrix<CALIBRATION_AXIS>& b = *b_ptr;

  b = H_T * w;

  delete H_T_ptr;
  delete w_ptr;

  BLA::Matrix<CALIBRATION_AXIS, CALIBRATION_AXIS> H_TH_inv = H_TH;
  Invert(H_TH_inv);

  // Calculate least squares solution
  BLA::Matrix<CALIBRATION_AXIS> X = H_TH_inv * b;
  
  // Extract offsets and scales from the solution
  float OSx = X(0) / 2.0;
  float OSy = X(1) / (2.0 * X(3));
  float OSz = X(2) / (2.0 * X(4));
  
  float A = X(5) + OSx * OSx + X(3) * OSy * OSy + X(4) * OSz * OSz;
  float B = A / X(3);
  float C = A / X(4);
  
  float SCx = sqrt(A);
  float SCy = sqrt(B);
  float SCz = sqrt(C);
  
  // Assign offsets and scale to output arrays
  offsets[0] = OSx;
  offsets[1] = OSy;
  offsets[2] = OSz;
  
  scale[0] = SCx;
  scale[1] = SCy;
  scale[2] = SCz;
}
