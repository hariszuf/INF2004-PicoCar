
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define IR_MAIN_PIN     2
#define IR_BACKUP_PIN  26
#define IR_BARCODE_PIN  0

#define IR_BLACK_LEVEL  0

typedef enum {
    LINE_LOST,
    LINE_BOTH_BLACK,
    LINE_MAIN_ONLY,
    LINE_BACKUP_ONLY
} line_status_t;

typedef struct {
    bool main_black;
    bool backup_black;
    bool barcode_black;
    line_status_t line_status;
} ir_data_t;

// Initialise all three IR sensors
void ir_sensor_init(void);

// Read the sensors and determine line status
ir_data_t ir_sensor_read(void);

// Print a sensor reading
void ir_sensor_print(const ir_data_t *data);

// Print only when sensor readings change
void ir_sensor_log_changes(const ir_data_t *data);

#endif
