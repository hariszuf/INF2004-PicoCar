
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdbool.h>

// IR Sensor GPIO Pins
#define IR_MAIN_PIN     2
#define IR_BACKUP_PIN   0
#define IR_BARCODE_PIN  26

// Sensor outputs HIGH when detecting black
#define IR_BLACK_LEVEL  1

// Possible line detection states
typedef enum {
    LINE_LOST,
    LINE_BOTH_BLACK,
    LINE_MAIN_ONLY,
    LINE_BACKUP_ONLY
} line_status_t;

// Store all sensor readings
typedef struct {
    bool main_black;
    bool backup_black;
    bool barcode_black;
    line_status_t line_status;
} ir_data_t;

// Function declarations
void ir_sensor_init(void);
ir_data_t ir_sensor_read(void);
void ir_sensor_print(const ir_data_t *data);
void ir_sensor_log_changes(const ir_data_t *data);

#endif
