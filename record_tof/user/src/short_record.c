#include <conio.h>
#include <dlfcn.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "vl53l5cx_api.h"

#define KRED "\x1B[31m"
#define KNRM "\x1B[0m"

int rot[8][8];

int short_record(VL53L5CX_Configuration* p_dev) {
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            rot[i][j] = (7 - i) + 8 * j;
        }
    }

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
    printf("please input how many seconds to record ");
    float time_length;
    scanf("%f", &time_length);
    getch();
    struct timespec ts;
    while (1) {
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN A3
         * (GPIO 1) when a new measurement is ready */
        printf("Press s to start\n");
        char command = getch();
        if (command == 's') {
            timespec_get(&ts, TIME_UTC);
            char filename[40];
            sprintf(filename, "short_record/%ld.txt", ts.tv_sec);
            FILE* fp = fopen(filename, "w");
            double time_start = ts.tv_sec;
            // record data for time_length seconds
            while (1) {
                timespec_get(&ts, TIME_UTC);
                if (ts.tv_sec - time_start > time_length) break;
                status = vl53l5cx_check_data_ready(p_dev, &isReady);

                if (isReady) {
                    vl53l5cx_get_ranging_data(p_dev, &Results);

                    /* As the sensor is set in 8x8 mode, we have a total
                     * of 64 zones to print. For this example, only the data of
                     * first zone are print */
                    // printf("Print data no : %3u\n", p_dev->streamcount);
                    timespec_get(&ts, TIME_UTC);
                    fprintf(fp, "N %3u %ld.%09ld\n", p_dev->streamcount,
                            ts.tv_sec, ts.tv_nsec);
                    float valid_count = 0;
                    printf("\033c");   // clear screen
                    for (i = 0; i < 64; i++) {
                        for (int j = 0; j < VL53L5CX_NB_TARGET_PER_ZONE; j++) {
                            uint16_t idx = VL53L5CX_NB_TARGET_PER_ZONE * i + j;

                            // Display
                            int row = i / 8, col = i % 8;
                            int print_id = rot[row][col];
                            if (Results.target_status[print_id] != 5) {
                                printf("%s%4dmm[%3u]  %s", KRED,
                                       Results.distance_mm[print_id],
                                       Results.target_status[print_id], KNRM);
                            } else {
                                printf("%4dmm[%3u]  ",
                                       Results.distance_mm[print_id],
                                       Results.target_status[print_id]);
                                valid_count++;
                            }
                            if (col == 7) printf("\n");
                            // Record
                            fprintf(fp,
                                    "Zone : %3d, Status : %3u, Distance : %4d "
                                    "mm, Sigma : %hu mm\n",
                                    i, Results.target_status[idx],
                                    Results.distance_mm[idx],
                                    Results.range_sigma_mm[idx]);
                        }
                    }
                    printf("Valid Ratio:%4.2f%%\n", valid_count / 64 * 100);
                    fprintf(fp, "\n");
                    fflush(fp);
                }
                WaitMs(&p_dev->platform, 5);
            }
            fclose(fp);
        }
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
    status = short_record(&Dev);
    vl53l5cx_comms_close(&Dev.platform);

    return 0;
}
