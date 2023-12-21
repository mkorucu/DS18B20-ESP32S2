/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define GPIO_DS18B20_0       (GPIO_NUM_36)
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_10_BIT)
#define SAMPLE_PERIOD        (1000)   // milliseconds

extern "C"{
    void app_main();
}
void temp_measure(void *arg)
{
    float *temp = (float *)arg;
    gpio_pulldown_dis(GPIO_DS18B20_0);
    gpio_pullup_en(GPIO_DS18B20_0);


    // Stable readings require a brief period before communication
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    printf("Find device:\n");
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    if(found)
    {
        num_devices += 1;
        printf("Found device\n");
        // For a single device only:
        OneWireBus_ROMCode rom_code;
        owb_status status = owb_read_rom(owb, &rom_code);
        if (status == OWB_STATUS_OK)
        {
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
            printf("Single device %s present\n", rom_code_s);
        }
        else
        {
            printf("An error occurred reading ROM code: %d", status);
        }
    }
    // Create DS18B20 devices on the 1-Wire bus
    DS18B20_Info * device = ds18b20_malloc();
    printf("Single device optimisations enabled\n");
    ds18b20_init_solo(device, owb);          // only one device on bus
    ds18b20_use_crc(device, true);           // enable CRC check on all reads
    ds18b20_set_resolution(device, DS18B20_RESOLUTION);

    // Read temperatures more efficiently by starting conversions on all devices at the same time
    if (num_devices > 0)
    {
        TickType_t last_wake_time = xTaskGetTickCount();
        int sample_count = 0;
        while (1)
        {
            ds18b20_convert_all(owb);
            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
            ds18b20_wait_for_conversion(device);

            // Read the results immediately after conversion otherwise it may fail
            // (using printf before reading may take too long)
            float reading = 0;
            DS18B20_ERROR err;

            err = ds18b20_read_temp(device, &reading);

            // Print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C): sample %d\n", ++sample_count);
            if (err != DS18B20_OK)
            {
                printf("error while reading!\n");
                continue;
            }
            printf("%.2fC\n", reading);
            *temp = reading;
            vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        }
    }
    else
    {
        printf("\nNo DS18B20 devices detected!\n");
    }
    // clean up dynamically allocated data
    for (int i = 0; i < num_devices; ++i)
    {
        ds18b20_free(&device);
    }
    owb_uninitialize(owb);

    printf("Restarting now.\n");
    fflush(stdout);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_restart();
}

void app_main()
{
    float temp;
    xTaskCreate(temp_measure, "temp_measure", configMINIMAL_STACK_SIZE * 3, &temp, 5, NULL);
}
