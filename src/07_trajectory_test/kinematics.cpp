#include <cstdio>
#include <cmath>
#include <algorithm>
#include <deque>

#include <armadillo>

#include <timer.h>
#include <Pixy.h>

struct Point3D {
    private:
        double x;
        double y;
        double z;
        double t;
    public:
        /* Constructors */
        Point3D() :
            x(0), y(0), z(0), t(0) {};

        Point3D(double _x, double _y, double _z) :
            x(_x), y(_y), z(_z), t(0) {};

        Point3D(double _x, double _y, double _z, double _t) :
            x(_x), y(_y), z(_z), t(_t) {};

        /* Getters */
        double getX() { return x; }
        double getY() { return y; }
        double getZ() { return z; }
        double getT() { return t; }

        /* Operator overloads */
        // Addition and subtraction
        Point3D operator+(const Point3D& p) {
            return Point3D(this->x + p.x, this->y + p.y, this->z + p.z, this->t + p.t);
        }
        Point3D operator-(const Point3D& p) {
            return Point3D(this->x - p.x, this->y - p.y, this->z - p.z, this->t - p.t);
        }
        // Scalar multiplication and division
        Point3D operator*(const double s) {
            return Point3D(this->x * s, this->y * s, this->z * s, this->t * s);
        }
        Point3D operator/(const double s) {
            return Point3D(this->x / s, this->y / s, this->z / s, this->t / s);
        }

        /* Methods */
        double maxAbsSpatial() {
            double abs_x = std::abs(this->x);
            double abs_y = std::abs(this->y);
            double abs_z = std::abs(this->z);
            return std::max(std::max(abs_x, abs_y), abs_z);
        }
};

// Camera intrinsics
const double fx = 218.75;
const double fy = 215; // originally calibrated to 210
const double cx = 160;
const double cy = 100; // need to recalibrate; cy = 92 when fy = 215
// Physical parameters
const double tx = 8.5 * 25.4; // camera horizontal separation
const double height = 260; // height of camera center from ground

// Validation
const double tol = 1;
const double z_max = 2000;
const double MAX_POS_DIFF = 50; // mm
const int MAX_MISSES = 3;
int numMisses = 0;

const uint8_t addr0 = 0x54;
const uint8_t addr1 = 0x55;

int main() {
    // Connect cameras
    Pixy pixy0(addr0);
    Pixy pixy1(addr1);

    if (pixy0.getLink().getAddr() == addr0) {
        // printf("Pixy 0 successfully initialized with I2C address %x.\n", addr0);
    }
    if (pixy1.getLink().getAddr() == addr1) {
        // printf("Pixy 1 successfully initialized with I2C address %x.\n", addr1);
    }

    // Timing
    timer_obj t0;
    timer_start(&t0);

    // Kinematics
    std::deque<Point3D> points;

    while (true) {
        /* Timing */
        timer_obj t1;
        timer_start(&t1);

        /* Detect ball in 3D */
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
                // printf("[!] Discarded object %d from cam 0 because of dimensions\n", i);
            }
        }
        for (int i = 0; i < nBlocks1; i++) {
            int w = pixy1.blocks[i].width;
            int h = pixy1.blocks[i].height;
            if (w * (1.0 - tol) < h && h < w * (1.0 + tol)) {
                ind1 = i;
                break;
            } else {
                // printf("[!] Discarded object %d from cam 1 because of dimensions\n", i);
            }
        }
        if (ind0 < 0 || ind1 < 0) continue;

        // Get x/y coordinates of ball by averaging coordinates
        double x_avg = ((double) (pixy0.blocks[ind0].x) + (double) (pixy1.blocks[ind1].x)) / 2.0;
        double y_avg = ((double) (pixy0.blocks[ind0].y) + (double) (pixy1.blocks[ind1].y)) / 2.0;

        // Get z world coordinates using stereo
        double x_diff = (double) (pixy0.blocks[ind0].x) - (double) (pixy1.blocks[ind1].x);
        double z_world = fx * tx / x_diff;

        // Convert x and y image coordinates to world coordinates
        double x_world = (x_avg - cx) * z_world / fx;
        double y_world = height - (y_avg - cy) * z_world / fy;

        // Timing
        double t_elapsed = timer_elapsed(&t0);

        // Prepare current position in car coordinates (x, y are horizontal plane)
        Point3D ptNow = Point3D(x_world, z_world, y_world, t_elapsed);

        // Flush points[] if too many misses
        if (numMisses > MAX_MISSES) {
            points.clear();
            numMisses = 0;
        }

        // Validate
        if (x_diff < 0) { // detected wrong objects
            numMisses++;
            continue;
        }
        if (z_world > z_max) { // max z
            numMisses++;
            continue;
        }
        if (points.size() > 0) {
            // Max change in position
            if ((ptNow - points.back()).maxAbsSpatial() > MAX_POS_DIFF) {
                numMisses++;
                continue; 
            }
        }

        // Add to points[]
        points.push_back(ptNow);
        if (points.size() > 100) { // keep maximum size at 100
            points.pop_front();
        }

        /* Predict trajectory */
        // Populate predictor matrices and response vectors
        arma::mat t(points.size(), 2); // for global x, y
        arma::mat t2(points.size(), 3); // for global z
        arma::vec res_x(points.size()); // global x
        arma::vec res_y(points.size()); // global y
        arma::vec res_z(points.size()); // global z
        for (int i = 0; i < points.size(); i++) {
            // Populate predictor matrices
            t(i, 0) = 1; // constant for bias/offset
            t2(i, 0) = 1;
            t(i, 1) = points[i].getT() - points.front().getT(); // t (linear term)
            t2(i, 1) = t(i, 1);
            t2(i, 2) = std::pow(t2(i, 1), 2); // t^2 (quadratic term)

            // Populator response vectors
            res_x(i) = points[i].getX();
            res_y(i) = points[i].getY();
            res_z(i) = points[i].getZ();
        }

        if (points.size() <= 2) continue; // not enough points for regression

        // Perform regression
        arma::vec beta_x = (t.t() * t).i() * t.t() * res_x;
        arma::vec beta_y = (t.t() * t).i() * t.t() * res_y;
        arma::vec beta_z = (t2.t() * t2).i() * t2.t() * res_z;

        // Validate results
        // if (beta_z(2)

        /* Timing */
        // Print
        printf("%.3f, %.1f, %.1f, %.1f, %.3f (%d) g = %.3f\n",
                ptNow.getT(), ptNow.getX(), ptNow.getY(), ptNow.getZ(),
                timer_elapsed(&t1), points.size(), beta_z(2) * 2 / 1000);
    }

    return 0;
}

