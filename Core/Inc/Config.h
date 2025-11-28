#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

typedef struct
{
    float ratio;
    int encoderResolution;
} GearBox;

static const GearBox GearBox_Motor370_12VDC_72rpm{64, 13};
static const GearBox GearBox_Motor_12VDC_80rpm{98.775, 1};

typedef struct
{
    float min;
    float max;
    float zero;
} PositionLimit;


#endif