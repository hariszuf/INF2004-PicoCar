
#include <stdio.h>
#include "pico/stdlib.h"

#define IR_MAIN_PIN     2
#define IR_BACKUP_PIN   0
#define IR_BARCODE_PIN  26

int main(void)
{
    stdio_init_all();

    gpio_init(IR_MAIN_PIN);
    gpio_set_dir(IR_MAIN_PIN, GPIO_IN);

    gpio_init(IR_BACKUP_PIN);
    gpio_set_dir(IR_BACKUP_PIN, GPIO_IN);

    gpio_init(IR_BARCODE_PIN);
    gpio_set_dir(IR_BARCODE_PIN, GPIO_IN);

    sleep_ms(3000);

    printf("IR RAW SENSOR TEST\n");

    while (true)
    {
        int main = gpio_get(IR_MAIN_PIN);
        int backup = gpio_get(IR_BACKUP_PIN);
        int barcode = gpio_get(IR_BARCODE_PIN);

        printf("Main GP2: %d | Backup GP0: %d | Barcode GP26: %d\n",
               main, backup, barcode);

        sleep_ms(500);
    }
}
