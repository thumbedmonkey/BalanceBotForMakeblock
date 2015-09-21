
#define LEONARDO_PORT1

#if defined(UNO_PORT1) //10:PB3 11:PB2
#define SDA_PORT PORTB
#define SDA_PIN 3
#define SCL_PORT PORTB
#define SCL_PIN 2
#define I2C_SLOWMODE 1
#elif defined(UNO_PORT2) //3:PD3 9:PB1
#define SDA_PORT PORTD
#define SDA_PIN 3
#define SCL_PORT PORTB
#define SCL_PIN 1
#define I2C_SLOWMODE 1
#elif defined(LEONARDO_PORT1) //11:PB7 8:PB4
#define SDA_PORT PORTB
#define SDA_PIN 7
#define SCL_PORT PORTB
#define SCL_PIN 4
#define I2C_SLOWMODE 1
#elif defined(LEONARDO_PORT2) //13:PC7 12:PD6
#define SDA_PORT PORTC
#define SDA_PIN 7
#define SCL_PORT PORTD
#define SCL_PIN 6
#define I2C_SLOWMODE 1
#else
// change sda scl if you use a different pin mapping
#endif

#include "MeLegoEncoder.h"
//#include "SoftI2CMaster.h"
#include <Wire.h>
#define SPEED_UPDATE_CYCLIC 100 // the speed is updated every 100ms
#define HOLD 0x40
#define SYNC 0x80
// move state and function
#define CMD_RESET 0x00
#define CMD_MOVE_TO 0x01
#define CMD_BREAK 0x02
#define CMD_MOVE_SPD 0x03
// config function
#define CMD_SET_PID 0x10
#define CMD_SET_HOLD 0x11
#define CMD_SET_POWER 0x12
#define CMD_SET_MODE 0x13
#define CMD_SET_PWM 0x14
#define CMD_SET_RATIO 0x15
// get motor status
#define CMD_GET_PID 0x20
#define CMD_GET_POWER 0x21
#define CMD_GET_POS 0x22
#define CMD_GET_SPEED 0x23
#define CMD_GET_RATIO 0x24

MeLEGOEncoder::MeLEGOEncoder()
{
  address = 0x9;
  // the default ratio
  //  ratio[0] = 46.666;
  //  ratio[1] = 46.666;
}

void MeLEGOEncoder::begin()
{
  //  i2c_init();
  Wire.begin();
}

void MeLEGOEncoder::MoveTo(uint8_t motor, long angle)
{
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_MOVE_TO;
  //  angle = (long)((float)angle*ratio[motor]*PULSE_PER_C/360);
  memcpy(cmdBuf + 2, &angle, 4);
  sendCmd();
}

void MeLEGOEncoder::MoveSpeed(uint8_t motor, int rpm)
{
  int speed;
  //        speed = (int)((float)rpm*ratio[motor]*12/600);// change to degree per 100ms
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_MOVE_SPD;
  memcpy(cmdBuf + 2, &rpm, 2);
  sendCmd();
}

void MeLEGOEncoder::SetHold(uint8_t motor, uint8_t hold)
{
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_SET_HOLD;
  cmdBuf[2] = hold;
  sendCmd();
}

void MeLEGOEncoder::ResetMotor(uint8_t motor)
{
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_RESET;
  sendCmd();
}

void MeLEGOEncoder::SetPID(uint8_t motor, float p, float i, float d, float s)
{
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_SET_PID;
  //  cmdBuf[2] = p;
  //  cmdBuf[3] = i;
  //  cmdBuf[4] = d;
  //  cmdBuf[5] = s;
  memcpy(&cmdBuf[2], &p, 4);
  memcpy(&cmdBuf[6], &i, 4);
  memcpy(&cmdBuf[10], &d, 4);
  memcpy(&cmdBuf[14], &s, 4);
  sendCmd();
}

void MeLEGOEncoder::SetMaxPower(uint8_t motor, int8_t maxPower)
{
  //        int power;
  //        power = maxSpeed*255/100;
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_SET_POWER;
  cmdBuf[2] = maxPower;
  sendCmd();
}

void MeLEGOEncoder::SetMode(uint8_t motor, uint8_t mode)
{
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_SET_MODE;
  cmdBuf[2] = mode;
  sendCmd();
}
void MeLEGOEncoder::SetPWM(uint8_t motor, int pwm)
{
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_SET_PWM;
  memcpy(cmdBuf + 2, &pwm, 2);
  //  cmdBuf[2] = pwm;
  sendCmd();
}
long MeLEGOEncoder::GetAngle(uint8_t motor)
{
  //  char buf[8];
  //  i2c_rep_start(address<<1); // I2C write direction
  //  i2c_write(motor);
  //  i2c_write(CMD_GET_POS);
  //  int len = i2c_read_to_buf(address, buf, 4);
  //
  //  return *(long*)buf;
  long pos;
  char buf[8];
  Wire.beginTransmission(address);
  Wire.write(motor);
  Wire.write(CMD_GET_POS);
  Wire.endTransmission(0);
  Wire.requestFrom(address, (uint8_t)4);
  for (int i = 0; i < 4; i++)
  {
    buf[i] = Wire.read();
  }
  pos = *(long*)buf;
  //      pos = (long)((float)pos/PULSE_PER_C*360/ratio[motor]);
  return pos;
}

void MeLEGOEncoder::GetPID(uint8_t motor, float * p, float * i, float * d, float * s)
{
  //          char buf[8];
  //  i2c_start(address<<1); // I2C write direction
  //  i2c_write(motor);
  //  i2c_write(CMD_GET_PID);
  //  int len = i2c_read_to_buf(address, buf, 4);
  //        *p = buf[0];
  //        *i = buf[1];
  //        *d = buf[2];
  //        *s = buf[3];
  char buf[8];

  Wire.beginTransmission(address);
  Wire.write(motor);
  Wire.write(CMD_GET_PID);
  Wire.endTransmission(0);
  Wire.requestFrom(address, (uint8_t)16);

  //  for(int i=0;i<4;i++)
  //  {
  //    buf[i] = Wire.read();
  //  }
  //  *p = buf[0];
  //  *i = buf[1];
  //  *d = buf[2];
  //  *s = buf[3];
  for (int i = 0; i < 4; i++)
  {
    buf[i] = Wire.read();
  }
  *p = *(float*)buf;
  for (int i = 0; i < 4; i++)
  {
    buf[i] = Wire.read();
  }
  *i = *(float*)buf;
  for (int i = 0; i < 4; i++)
  {
    buf[i] = Wire.read();
  }
  *d = *(float*)buf;
  for (int i = 0; i < 4; i++)
  {
    buf[i] = Wire.read();
  }
  *s = *(float*)buf;
}

int MeLEGOEncoder::GetSpeed(uint8_t motor)
{
  //  char buf[8];
  //        int speed,rpm;
  //  i2c_rep_start(address<<1); // I2C write direction
  //  i2c_write(motor);
  //  i2c_write(CMD_GET_SPEED);
  //  int len = i2c_read_to_buf(address, buf, 2);
  //        speed = *(int*)buf;
  //        rpm = (int)((float)speed*10*60/360); // change to rpm
  //  return rpm;
  char buf[8];
  int speed, rpm;
  Wire.beginTransmission(address);
  Wire.write(motor);
  Wire.write(CMD_GET_SPEED);
  Wire.endTransmission(0);
  Wire.requestFrom(address, (uint8_t)2);
  for (int i = 0; i < 2; i++)
  {
    buf[i] = Wire.read();
  }
  speed = *(int*)buf;
  // change to degree
  //        rpm = (int)((float)speed*600/12/ratio[motor]); // change to rpm
  return speed;
}

int8_t MeLEGOEncoder::GetPower(uint8_t motor)
{
  //  char buf[8];
  //        int maxSpeed;
  //  i2c_rep_start(address<<1); // I2C write direction
  //  i2c_write(motor);
  //  i2c_write(CMD_GET_POWER);
  //  int len = i2c_read_to_buf(address, buf, 2);
  //        maxSpeed = (*(int*)buf)*100/255;
  //  return maxSpeed;
  //  char buf[8];
  //        int power;
  Wire.beginTransmission(address);
  Wire.write(motor);
  Wire.write(CMD_GET_POWER);
  Wire.endTransmission(0);
  Wire.requestFrom(address, (uint8_t)1);
  return (int8_t)Wire.read();
  //        buf[0] = Wire.read();
  //        for(int i=0;i<2;i++)
  //        {
  //        buf[i] = Wire.read();
  //        }
  //        maxPower = (*(int*)buf)*100/255;
  //  return power;
}

//size_t MeLEGOEncoder::i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
//  i2c_rep_start((add<<1) | 1);  // I2C read direction
//  size_t bytes_read = 0;
//  uint8_t *b = (uint8_t*)buf;
//  while (size--) {
//    /* acknowledge all but the final byte */
//    *b++ = i2c_read(!(size > 0));
//    /* TODO catch I2C errors here and abort */
//    bytes_read++;
//  }
//  i2c_stop();
//  return bytes_read;
//}

void MeLEGOEncoder::sendCmd()
{
  //  int i;
  //  i2c_rep_start(address<<1); // I2C write direction
  //  for(i=0;i<10;i++){
  //    i2c_write(cmdBuf[i]);
  //  }
  //  i2c_stop();
  Wire.beginTransmission(address);
  for (int i = 0; i < 18; i++)
    Wire.write(cmdBuf[i]);
  Wire.endTransmission();
}

float MeLEGOEncoder::GetRatio(uint8_t motor)
{
  char buf[8];
  float ratio;
  Wire.beginTransmission(address);
  Wire.write(motor);
  Wire.write(CMD_GET_RATIO);
  Wire.endTransmission(0);
  Wire.requestFrom(address, (uint8_t)4);
  for (int i = 0; i < 4; i++)
  {
    buf[i] = Wire.read();
  }
  ratio = *(float*)buf;
  return ratio;
}

void MeLEGOEncoder::SetRatio(uint8_t motor, float ratio)
{
  cmdBuf[0] = motor;
  cmdBuf[1] = CMD_SET_RATIO;
  memcpy(cmdBuf + 2, &ratio, 4);
  sendCmd();
}






