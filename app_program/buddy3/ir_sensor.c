
#include <stdio.h>
#include "pico/stdlib.h"
#include "ir_sensor.h"

// Initialise all IR sensors
void ir_sensor_init(void)
{
    gpio_init(IR_MAIN_PIN);
    gpio_set_dir(IR_MAIN_PIN, GPIO_IN);

    gpio_init(IR_BACKUP_PIN);
    gpio_set_dir(IR_BACKUP_PIN, GPIO_IN);

    gpio_init(IR_BARCODE_PIN);
    gpio_set_dir(IR_BARCODE_PIN, GPIO_IN);

    printf("[IR] Sensors Initialised\n");
}

// Read IR sensor values
ir_data_t ir_sensor_read(void)
{
    ir_data_t data;

    data.main_black =
        gpio_get(IR_MAIN_PIN) == IR_BLACK_LEVEL;

    data.backup_black =
        gpio_get(IR_BACKUP_PIN) == IR_BLACK_LEVEL;

    data.barcode_black =
        gpio_get(IR_BARCODE_PIN) == IR_BLACK_LEVEL;

    // Determine line detection status
    if (data.main_black && data.backup_black)
    {
        data.line_status = LINE_BOTH_BLACK;
    }
    else if (data.main_black)
    {
        data.line_status = LINE_MAIN_ONLY;
    }
    else if (data.backup_black)
    {
        data.line_status = LINE_BACKUP_ONLY;
    }
    else
    {
        data.line_status = LINE_LOST;
    }

    return data;
}

// Display IR sensor readings
void ir_sensor_print(const ir_data_t *data)
{
    printf("\n===== IR SENSOR STATUS =====\n");

    printf("Main GP2: %s\n",
           data->main_black ? "BLACK" : "WHITE");

    printf("Backup GP0: %s\n",
           data->backup_black ? "BLACK" : "WHITE");

    printf("Barcode GP26: %s\n",
           data->barcode_black ? "BLACK" : "WHITE");

    // Line detection results
    switch (data->line_status)
    {
        case LINE_BOTH_BLACK:
            printf("[LINE] Both sensors detect BLACK\n");
            break;

        case LINE_MAIN_ONLY:
            printf("[LINE] Main sensor detects BLACK\n");
            break;

        case LINE_BACKUP_ONLY:
            printf("[LINE] Backup sensor detects BLACK\n");
            break;

        case LINE_LOST:
        default:
            printf("[LINE] No black line detected\n");
            break;
    }

    printf("============================\n");
}

// Print only when readings change
void ir_sensor_log_changes(const ir_data_t *data)
{
    static bool first_read = true;
    static bool last_main = false;
    static bool last_backup = false;
    static bool last_barcode = false;

    if (first_read ||
        data->main_black != last_main ||
        data->backup_black != last_backup ||
        data->barcode_black != last_barcode)
    {
        ir_sensor_print(data);

        last_main = data->main_black;
        last_backup = data->backup_black;
        last_barcode = data->barcode_black;

        first_read = false;
    }
}
