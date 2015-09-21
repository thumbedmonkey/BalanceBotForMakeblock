void get_cmd(void)
{
  while (Serial.available() > 0)
  {
    char nullByte = char(Serial.read());
    if (nullByte == ';')
    {
      comdata[data_p] = nullByte;
      data_p = 0;
      FLAG |= COMDONE;
    }
    else
    {
      comdata[data_p] = nullByte ;
      data_p++ ;
    }
  }
}

void set_value(void)
{
  if (FLAG & COMDONE)
  {
    strtok(comdata, ",");
    joy_x = atof(strtok(NULL, ","));
    joy_y = atof(strtok(NULL, ";"));
    switch (comdata[1])
    {
      case 'J':
        FLAG |= MOVING;
        PID_speed.Setpoint = joy_y * 200;
        if (joy_x > 0.2 || joy_x < -0.2)
        {
          if (PID_speed.Setpoint > 0)
            PID_turn.Output  = joy_x * 50;
          else
            PID_turn.Output  = -joy_x * 50;
        }
        else
          PID_turn.Output  = 0;
        break;
      case 'S':
        PID_speed.Setpoint = 0;
        PID_turn.Output  = 0;
        FLAG &= ~MOVING;
        break;
    }
    FLAG &= ~COMDONE;
  }
}
