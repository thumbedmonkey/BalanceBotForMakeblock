#ifndef ME_LEGOENCODER_H
#define ME_LEGOENCODER_H
#include <Arduino.h>

#define PULSE_PER_C 12

class MeLEGOEncoder {
  public:
    MeLEGOEncoder();
    void begin();
    void MoveTo(uint8_t motor, long angle);
    void MoveSpeed(uint8_t motor, int rpm);
    void SetHold(uint8_t motor, uint8_t hold);
    void SetPID(uint8_t motor, float p, float i, float d, float s);
    void SetMaxPower(uint8_t motor, int8_t maxPower);
    long GetAngle(uint8_t motor);
    int8_t GetPower(uint8_t motor);
    void GetPID(uint8_t motor, float * p, float * i, float * d, float * s);
    int GetSpeed(uint8_t motor);
    void ResetMotor(uint8_t motor);
    float GetRatio(uint8_t motor);
    void SetRatio(uint8_t motor, float r);
    void SetMode(uint8_t motor, uint8_t mode);
    void SetPWM(uint8_t motor, int pwm);
  private:
    void sendCmd();
    uint8_t address;
    char cmdBuf[18];
};

#endif

