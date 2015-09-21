void MOTOR(char TYPE, int SPEED)
{
  if (TYPE)
  {
    encoder.SetPWM(1, -SPEED);
  }
  else
  {
    encoder.SetPWM(0, SPEED);
  }
}

