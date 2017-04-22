/* ========================================================================== *
 * Description: Project:    01_object_detection_test
 *              File:       detect.cpp
 * ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "pixy.h"

#define BLOCK_BUFFER_SIZE          25

#define PIXY_X_CENTER              ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define PIXY_Y_CENTER              ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

// Pixy Block Buffer //
struct Block  blocks [BLOCK_BUFFER_SIZE];

static bool run_flag = true;

void handle_SIGINT(int unused) {
    // On CTRL+C - abort! //

    run_flag = false;
}

int main(int argc, char* argv[]) {
    int pixy_init_status;
    char buf[128];
    int frame_index = 0;
    int result;
    int blocks_copied;
    int index;


    printf("+ Pixy Ball Detection Test Started +\n");
    fflush(stdout);

    // Catch CTRL+C (SIGINT) signals //
    signal(SIGINT, handle_SIGINT);

    // Connect to Pixy //
    pixy_init_status = pixy_init();

    // Was there an error initializing pixy? //
    if(pixy_init_status != 0) {
        // Error initializing Pixy //
        printf("Error: pixy_init() [%d] ", pixy_init_status);
        pixy_error(pixy_init_status);

        return pixy_init_status;
    }

    // Run loop
    while (run_flag) {

        // Wait for new blocks to be available //
        while (!pixy_blocks_are_new() && run_flag);

        // Get blocks from Pixy //
        blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);

        if (blocks_copied < 0) {
            // Error: pixy_get_blocks //
            printf("Error: pixy_get_blocks() [%d] ", blocks_copied);
            pixy_error(blocks_copied);
            fflush(stdout);
        }

        if (blocks_copied > 0) {
            // Do something with these blocks
        }

        if (frame_index % 50 == 0) {
            // Display received blocks //
            printf("frame %d:\n", frame_index);
            for (index = 0; index != blocks_copied; index++) {
                printf("  sig:%2d x:%4d y:%4d width:%4d height:%4d\n",
                        blocks[index].signature,
                        blocks[index].x,
                        blocks[index].y,
                        blocks[index].width,
                        blocks[index].height);
            }
            fflush(stdout);
        }

        frame_index++;
    }
    pixy_close();

    return 0;
}

// BC and TJ 2017 //
