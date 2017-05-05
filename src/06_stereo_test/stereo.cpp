#include <stdio.h>
#include <algorithm>
#include <timer.h>
#include <Pixy.h>

// Camera intrinsics
const double fx = 218.75;
const double fy = 210;
const double cx = 160;
const double cy = 92;
// Physical parameters
const double tx = 8.5 * 25.4; // camera horizontal separation
const double height = 260; // height of camera center from ground

// Validation
const double tol = 1;
const double z_max = 2000;

const uint8_t addr0 = 0x54;
const uint8_t addr1 = 0x55;

int main() {
    Pixy pixy0(addr0);
    Pixy pixy1(addr1);

    if (pixy0.getLink().getAddr() == addr0) {
        printf("Pixy 0 successfully initialized with I2C address %x.\n", addr0);
    }
    if (pixy1.getLink().getAddr() == addr1) {
        printf("Pixy 1 successfully initialized with I2C address %x.\n", addr1);
    }

    while (true) {
        // Timing for FPS
        timer_obj t1;
        timer_start(&t1);

        // Wait until blocks detected on both cameras
        int nBlocks0 = 0;
        int nBlocks1 = 0;
        while (!nBlocks0) { nBlocks0 = pixy0.getBlocks(); }
        while (!nBlocks1) { nBlocks1 = pixy1.getBlocks(); }

        // Filter roughly round objects
        int ind0 = -1;
        int ind1 = -1;
        for (int i = 0; i < nBlocks0; i++) {
            int w = pixy0.blocks[i].width;
            int h = pixy0.blocks[i].height;
            if (w * (1.0 - tol) < h && h < w * (1.0 + tol)) {
                ind0 = i;
                break;
            } else {
                printf("[!] Discarded object %d from cam 0 because of dimensions\n", i);
            }
        }
        for (int i = 0; i < nBlocks1; i++) {
            int w = pixy1.blocks[i].width;
            int h = pixy1.blocks[i].height;
            if (w * (1.0 - tol) < h && h < w * (1.0 + tol)) {
                ind1 = i;
                break;
            } else {
                printf("[!] Discarded object %d from cam 1 because of dimensions\n", i);
            }
        }
        if (ind0 < 0 || ind1 < 0) continue;

        // Get x/y coordinates of ball by averaging coordinates
        double x_avg = ((double) (pixy0.blocks[ind0].x) + (double) (pixy1.blocks[ind1].x)) / 2.0;
        double y_avg = ((double) (pixy0.blocks[ind0].y) + (double) (pixy1.blocks[ind1].y)) / 2.0;

        // Get z world coordinates using stereo
        double x_diff = (double) (pixy0.blocks[ind0].x) - (double) (pixy1.blocks[ind1].x);
        double z_world = fx * tx / x_diff;

        // Validate
        if (x_diff < 0) {
            printf("[!] Discarded frame because of inconsistent object detection\n");
            continue;
        }
        if (z_world > z_max) {
            printf("[!] Discarded frame because z exceeded maximum\n");
            continue;
        }

        // Convert x and y image coordinates to world coordinates
        double x_world = (x_avg - cx) * z_world / fx;
        double y_world = height - (y_avg - cy) * z_world / fy;

        // Timing
        double fps = 1.0 / (double) (timer_elapsed(&t1));

        // Print
        std::string zs(std::min((int) (z_world / 20), 200), ' ');
        printf("x: %5.1f, y: %5.1f, z: %8.1f (%4.1f fps) %so\n", x_world, y_world, z_world, fps, zs.c_str());
    }

    return 0;
}

