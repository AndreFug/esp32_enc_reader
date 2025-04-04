#ifndef ENC_READER_H
#define ENC_READER_H

void setupEncoders();
void updateEncoderStats();
float getAverageRPM();
float getRPMDifference();
bool getEncoder1Direction();
bool getEncoder2Direction();
long getEncoder1Count();
long getEncoder2Count();

#endif
