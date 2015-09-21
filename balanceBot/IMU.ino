void IMU_fillter(void)
{
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  if (FLAG & START)
    PID_angle_compute();

  double dt = (double)(micros() - IMU_timer) / 1000000;
  IMU_timer = micros();

  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double gyroYrate = -gyroY / 131.0;

  if (FLAG & START)
    PID_angle_compute();

  gyroYangle += gyroYrate * dt;
  compAngleY = 0.98 * (compAngleY + gyroYrate * dt) + 0.02 * pitch;

  if (FLAG & START)
    if (  compAngleY > 40 ||  compAngleY < -40)
    {
      MOTOR(0, 0);
      MOTOR(1, 0);
      while (1);
    }
  //Serial.println(compAngleY);
}

