#ifndef IR_ARRAY_DRIVER_H
#define IR_ARRAY_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define IR_CHANNEL_COUNT       5
#define IR_MAX_READ_FAILURES   3


/* Return codes */

#define IR_SUCCESS              0
#define IR_ERROR               -1
#define IR_NOT_INITIALIZED     -2
#define IR_INVALID_ARGUMENT    -3
#define IR_GPIO_INIT_FAILED    -4
#define IR_GPIO_CONFIG_FAILED  -5
#define IR_GPIO_READ_FAILED    -6
#define IR_SENSOR_FAULT        -7
#define IR_LINE_LOST           -8


/* Line position */

typedef enum
{
    IR_POSITION_UNKNOWN = 0,

    IR_POSITION_FAR_LEFT,
    IR_POSITION_LEFT,
    IR_POSITION_SLIGHT_LEFT,

    IR_POSITION_CENTER,

    IR_POSITION_SLIGHT_RIGHT,
    IR_POSITION_RIGHT,
    IR_POSITION_FAR_RIGHT,

    IR_POSITION_LOST

} IR_Position;


/* Sensor reading */

typedef struct
{
    uint8_t sensor[IR_CHANNEL_COUNT];

    IR_Position position;

    uint8_t valid;
    uint8_t line_detected;

    uint32_t sequence;
    uint32_t read_failures;

} IR_Reading;


/* Configuration */

typedef struct
{
    int gpio[IR_CHANNEL_COUNT];

    /*
     * 0 = HIGH means detected
     * 1 = LOW means detected
     */
    uint8_t active_low;

} IR_Config;


/* API */

int IRArray_Init(const IR_Config *config);

int IRArray_Read(IR_Reading *reading);

IR_Position IRArray_GetPosition(
    const IR_Reading *reading
);

const char *IRArray_PositionString(
    IR_Position position
);

int IRArray_IsHealthy(void);

void IRArray_Close(void);


#ifdef __cplusplus
}
#endif

#endif
