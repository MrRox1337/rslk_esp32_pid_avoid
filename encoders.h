#pragma once
// Quadrature encoders counted in hardware by the PCNT peripheral (4x, no ISRs).

void encodersBegin();
void readEncoders(long &left, long &right);   // raw counts, no FLIP_ENC_x applied
