# Encoder and motor control ESP32

The ESP32 receives serial commands from a Raspberry Pi in a `bytearray(6)` format:

```cpp
int id1 = (int) buffer[0];
int dir1 = (int) buffer[1];
float spd1 = (int) buffer[2];
int id2 = (int) buffer[3];
int dir2 = (int) buffer[4];
float spd2 = (int) buffer[5];
```

To controll two dc motors. The motors are then controlled by a PI regulator which regulates the applied voltage to the motors. The set point is in mm per sec for each motor.

## After testing
The PI regulator is aggresive and has a high `I term`, this makes the motor respond quick when applying friction to the motors.