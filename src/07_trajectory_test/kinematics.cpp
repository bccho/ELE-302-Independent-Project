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
const double R = 0.4; // coefficient of restitution
const double g_acc = 9.8; // gravitational field strength

// Trajectory prediction
arma::vec beta_x(2); // [0] = offset x(0), [1] = vx(0)
arma::vec beta_y(2); // [0] = offset y(0), [1] = vy(0)
arma::vec beta_z(3); // [0] = offset z(0), [1] = vz(0), [2] = acceleration g

// Predict time when ball will first bounce
//   bz: 3-vector, z coefficients for naive kinematic fit
double predictBounceTime(arma::vec& bz, double z0) {
    // Solves quadratic equation
    //   z(t) = z(0) + vz(0) t + 0.5 g t^2 = 0
    double g = g_acc;
    return (-1.0 / g) * (bz(1) + std::sqrt(std::pow(bz(1), 2) - 2 * g * z0));
}

// Predict location of first bounce
//   bx, by, bz: vectors of length 2, 2, 3 respectively;
//               coefficients of naive kinematic fit
//   pt1: position of ball at time 0; point on which bx, by, bz are based
Point3D predictBounceLocation(arma::vec& bx, arma::vec& by, arma::vec& bz,
        Point3D& pt1) {
    double t_bounce = predictBounceTime(bz, pt1.getZ());
    double newx = pt1.getX() + bx(1) * (t_bounce - pt1.getT());
    double newy = pt1.getY() + by(1) * (t_bounce - pt1.getT());
    double newz = pt1.getZ() + bz(1) * (t_bounce - pt1.getT())
        + bz(2) * std::pow(t_bounce - pt1.getT(), 2);

    return Point3D(newx, newy, newz, t_bounce);
}

// Predict position of ball at time t, using the bouncing ball model
// (handles up to one bounce)
//   bx, by, bz: vectors of length 2, 2, 3 respectively;
//               coefficients of naive kinematic fit
//   pt1: position of ball at time 0; point on which bx, by, bz are based
Point3D predictPosition(arma::vec& bx, arma::vec& by, arma::vec& bz,
        Point3D& pt1, double t) {
    double newx = pt1.getX() + bx(1) * (t - pt1.getT());
    double newy = pt1.getY() + by(1) * (t - pt1.getT());
    double newz = pt1.getZ() + bz(1) * (t - pt1.getT())
        + bz(2) * std::pow(t - pt1.getT(), 2);

    if (newz < 0) { // the ball will have bounced
        // Calculate the time of the first bounce
        double t_bounce = predictBounceTime(bz, pt1.getZ());
        // Calculate z velocity at time of bounce
        double vz = bz(0) + bz(2) * (t_bounce - pt1.getT());
        // New z velocity after bounce
        vz = std::sqrt(R) * vz;
        newz = 0 + vz * (t - t_bounce) + bz(2) * std::pow(t - t_bounce, 2);
    }

    return Point3D(newx, newy, newz, t);
}

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
        fprintf(stderr, "Pixy 0 successfully initialized with I2C address %x.\n", addr0);
    }
    if (pixy1.getLink().getAddr() == addr1) {
        fprintf(stderr, "Pixy 1 successfully initialized with I2C address %x.\n", addr1);
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
                fprintf(stderr, "[!] Discarded object %d from cam 0 because of dimensions\n", i);
            }
        }
        for (int i = 0; i < nBlocks1; i++) {
            int w = pixy1.blocks[i].width;
            int h = pixy1.blocks[i].height;
            if (w * (1.0 - tol) < h && h < w * (1.0 + tol)) {
                ind1 = i;
                break;
            } else {
                fprintf(stderr, "[!] Discarded object %d from cam 1 because of dimensions\n", i);
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
        if (x_diff < 0) { // x_diff makes no sense
            numMisses++;
            continue;
        }
        if (z_world > z_max) { // max z
            numMisses++;
            continue;
        }
        if (points.size() > 0) {
            // Max change in position from expected next position
            Point3D ptFirst = points.front();
            double t_first = ptFirst.getT();
            Point3D ptExp = predictPosition(beta_x, beta_y, beta_z, ptFirst, t_elapsed);
            printf("%.3f, %.1f, %.1f, %.1f",
                    ptNow.getT(), ptExp.getX(), ptExp.getY(), ptExp.getZ());
            if ((ptNow - ptExp).maxAbsSpatial() > MAX_POS_DIFF) {
                numMisses++;
                //printf("\n");
                //continue;
            }
        } else {
            printf("%.3f,,,", ptNow.getT());
        }
        numMisses = 0;

        // Add to points[]
        points.push_back(ptNow);
        if (points.size() > 100) { // keep maximum size at 100
            points.pop_front();
        }

        // Print new point data
        printf(", %.1f, %.1f, %.1f, %d",
                ptNow.getX(), ptNow.getY(), ptNow.getZ(), points.size());

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

        if (points.size() <= 2) { // not enough points for regression
            printf("\n");
            continue;
        }

        // Perform regression
        beta_x = (t.t() * t).i() * t.t() * res_x;
        beta_y = (t.t() * t).i() * t.t() * res_y;
        beta_z = (t2.t() * t2).i() * t2.t() * res_z;
        double acc = beta_z(2) * 2 / 1000;

        // Validate results
        if (acc > -3.0) {
            // flush points[]
            points.clear();
            // add back current point
            points.push_back(ptNow);
            numMisses = 0;
        }

        // Print regression results
        printf(", %.3f, %.3f",
                timer_elapsed(&t1), acc);

        // Predict bounce location
        Point3D ptBounce = predictBounceLocation(beta_x, beta_y, beta_z, points.front());
        printf(", %.1f, %.1f, %.1f, %.3f\n",
                ptBounce.getX(), ptBounce.getY(), ptBounce.getZ(), ptBounce.getT());
    }

    return 0;
}

