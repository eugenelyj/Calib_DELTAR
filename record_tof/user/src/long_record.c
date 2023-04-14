#include <conio.h>
#include <dlfcn.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "vl53l5cx_api.h"

int long_record(VL53L5CX_Configuration* p_dev) {
    /*********************************/
    /*   VL53L5CX ranging variables  */
    /*********************************/

    uint8_t status, loop, isAlive, isReady, i;
    uint32_t integration_time_ms;
    VL53L5CX_ResultsData Results; /* Results data from VL53L5CX */

    /*********************************/
    /*   Power on sensor and init    */
    /*********************************/

    /* (Optional) Check if there is a VL53L5CX sensor connected */
    status = vl53l5cx_is_alive(p_dev, &isAlive);
    if (!isAlive || status) {
        printf("VL53L5CX not detected at requested address\n");
        return status;
    }

    /* (Mandatory) Init VL53L5CX sensor */
    status = vl53l5cx_init(p_dev);
    if (status) {
        printf("VL53L5CX ULD Loading failed\n");
        return status;
    }

    /*********************************/
    /*        Set some params        */
    /*********************************/

    /* Set resolution in 8x8. WARNING : As others settings depend to this
     * one, it must be the first to use.
     */
    status = vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8);
    if (status) {
        printf("vl53l5cx_set_resolution failed, status %u\n", status);
        return status;
    }

    /* Set ranging frequency to 10Hz.
     * Using 4x4, min frequency is 1Hz and max is 60Hz
     * Using 8x8, min frequency is 1Hz and max is 15Hz
     */
    status = vl53l5cx_set_ranging_frequency_hz(p_dev, 10);
    if (status) {
        printf("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);
        return status;
    }

    // /* Set target order to closest */
    // status = vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    // if(status)
    // {
    // 	printf("vl53l5cx_set_target_order failed, status %u\n", status);
    // 	return status;
    // }

    /* Get current integration time */
    status = vl53l5cx_get_integration_time_ms(p_dev, &integration_time_ms);
    if (status) {
        printf("vl53l5cx_get_integration_time_ms failed, status %u\n", status);
        return status;
    }
    printf("Current integration time is : %d ms\n", integration_time_ms);

    /*********************************/
    /*         Ranging loop          */
    /*********************************/

    status = vl53l5cx_start_ranging(p_dev);
    printf("VL53L5CX ULD ready ! (Version : %s)\n", VL53L5CX_API_REVISION);
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    double time_start = ts.tv_sec;
    char filename[40];
    sprintf(filename, "long_record/%ld.txt", ts.tv_sec);
    FILE* fp = fopen(filename, "w");
    while (1) {
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN A3
         * (GPIO 1) when a new measurement is ready */
        status = vl53l5cx_check_data_ready(p_dev, &isReady);

        if (isReady) {
            vl53l5cx_get_ranging_data(p_dev, &Results);

            /* As the sensor is set in 8x8 mode, we have a total
             * of 64 zones to print. For this example, only the data of
             * first zone are print */
            // printf("Print data no : %3u\n", p_dev->streamcount);
            timespec_get(&ts, TIME_UTC);
            fprintf(fp, "N %3u %ld.%09ld\n", p_dev->streamcount, ts.tv_sec,
                    ts.tv_nsec);
            for (i = 0; i < 64; i++) {
                printf("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
                       Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                       Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
                fprintf(fp, "Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
                        Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                        Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
            }
            printf("\n");
            fprintf(fp, "\n");
            fflush(fp);
        }
        WaitMs(&p_dev->platform, 5);
        /* Wait a few ms to avoid too high polling (function in platform
         * file, not in API) */
    }

    status = vl53l5cx_stop_ranging(p_dev);
    printf("End of ULD demo\n");
    return status;
}

int main(int argc, char** argv) {
    int status;
    VL53L5CX_Configuration Dev;

    /*********************************/
    /*   Power on sensor and init    */
    /*********************************/

    /* Initialize channel com */
    status = vl53l5cx_comms_init(&Dev.platform);
    if (status) {
        printf("VL53L5CX comms init failed\n");
        return -1;
    }

    printf("Start recording data with ULD version %s\n", VL53L5CX_API_REVISION);
    status = long_record(&Dev);

    vl53l5cx_comms_close(&Dev.platform);

    return 0;
}
