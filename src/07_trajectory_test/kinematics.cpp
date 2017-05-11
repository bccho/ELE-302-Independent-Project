#include <cstdio>
#include <cmath>
#include <algorithm>
#include <queue>
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

/* Constants */
// I2C
const uint8_t ADDR0 = 0x54;
const uint8_t ADDR1 = 0x55;

// Camera intrinsics
const double fx = 218.75;
const double fy = 215; // originally calibrated to 210
const double cx = 160;
const double cy = 100; // need to recalibrate; cy = 92 when fy = 215

// Physical parameters
const double CAM_HOR_SEP = 8.5 * 25.4; // camera horizontal separation
const double CAM_HEIGHT = 260; // height of camera center from ground
const double BALL_DIAM = 1.5 * 25.4; // diameter of ball
const double COEF_REST = 0.59; 
// coefficient of restitution
const double G_ACC = -9.8; // gravitational field strength

// Trajectory prediction
const int REG_POINTS_TO_USE = 10;

// Validation
const double SIZE_TOL = 1;
const double Z_MAX = 2000;
const double MAX_POS_DIFF = 50; // mm
const int MAX_MISSES = 3;

/* Global variables */
// Estimates of x/y/z kinematic parameters
arma::vec beta_x(2); // [0] = offset x(0), [1] = vx(0)
arma::vec beta_y(2); // [0] = offset y(0), [1] = vy(0)
arma::vec beta_z(3); // [0] = offset z(0), [1] = vz(0), [2] = acceleration g
double 
int numMisses = 0;

/* Helper functions */
// Predict time when ball will first bounce
//   bz: 3-vector, z coefficients for naive kinematic fit
double predictBounceTime(arma::vec& bz, double z0) {
    // Solves quadratic equation
    //   z(t) = z(0) + vz(0) t + 0.5 g t^2 = 0
    double g = bz(2) * 2; // G_ACC * 1000;
    return (-1.0 / g) * (bz(1) + std::sqrt(std::pow(bz(1), 2) - 2 * g * (z0 - BALL_DIAM / 2)));
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
    double g = bz(2) * 2; // G_ACC * 1000; // bz(2) * 2
    double newx = pt1.getX() + bx(1) * (t - pt1.getT());
    double newy = pt1.getY() + by(1) * (t - pt1.getT());
    double newz = pt1.getZ() + bz(1) * (t - pt1.getT())
        + 0.5 * g * std::pow(t - pt1.getT(), 2);

    if (newz < 0) { // the ball will have bounced
        // Calculate the time of the first bounce
        double t_bounce = predictBounceTime(bz, pt1.getZ()) + pt1.getT();
        // fprintf(stderr, "bounce at %.3f\n", t_bounce);
        // Calculate z velocity at time of bounce
        double vz = bz(1) + g * (t_bounce - pt1.getT());
        // New z velocity after bounce
        vz = -std::sqrt(COEF_REST) * vz;
        newz = 0 + vz * (t - t_bounce) + 0.5 * g * std::pow(t - t_bounce, 2);
    }

    return Point3D(newx, newy, newz, t);
}

/* Main method */
int main() {
    // Connect cameras
    Pixy pixy0(ADDR0);
    Pixy pixy1(ADDR1);

    if (pixy0.getLink().getAddr() == ADDR0) {
        fprintf(stderr, "Pixy 0 successfully initialized with I2C address %x.\n", ADDR0);
    }
    if (pixy1.getLink().getAddr() == ADDR1) {
        fprintf(stderr, "Pixy 1 successfully initialized with I2C address %x.\n", ADDR1);
    }

    // Timing
    timer_obj t0;
    timer_start(&t0);

    // Kinematics
    std::deque<Point3D> points;

    while (true) {
        /* Get data from cameras */
        // Wait until blocks detected on both cameras
        int nBlocks0 = 0;
        int nBlocks1 = 0;
        while (!nBlocks0) { nBlocks0 = pixy0.getBlocks(); }
        while (!nBlocks1) { nBlocks1 = pixy1.getBlocks(); }

        /* Timing */
        timer_obj t1;
        timer_start(&t1);

        /* Detect ball in 3D */
        std::queue<std::pair<int, int> > plausiblePairs;
        for (int ind0 = 0; ind0 < nBlocks0; ind0++) {
            for (int ind1 = 0; ind1 < nBlocks0; ind1++) {
                int w0 = pixy0.blocks[ind0].width;
                int h0 = pixy0.blocks[ind0].height;
                int w1 = pixy1.blocks[ind1].width;
                int h1 = pixy1.blocks[ind1].height;

                // Filter roughly round objects
                if (!(w0 * (1.0 - SIZE_TOL) < h0 && h0 < w0 * (1.0 + SIZE_TOL))) {
                    continue;
                }
                if (!(w1 * (1.0 - SIZE_TOL) < h1 && h1 < w1 * (1.0 + SIZE_TOL))) {
                    continue;
                }

                // Add plausible pair to queue (implicitly in decreasing order of size)
                plausiblePairs.push(std::make_pair(ind0, ind1));
            }
        }

        /* Loop through each plausible pair */
        bool missed = true;
        while (!plausiblePairs.empty()) {
            std::pair<int, int> pp = plausiblePairs.front();
            int ind0 = pp.first, ind1 = pp.second;
            plausiblePairs.pop();

            /* Calculate position of ball in car coordinates (x, z are horizontal plane */
            // Get x/y coordinates of ball by averaging coordinates
            double x_avg = ((double) (pixy0.blocks[ind0].x) +
                    (double) (pixy1.blocks[ind1].x)) / 2.0;
            double y_avg = ((double) (pixy0.blocks[ind0].y) +
                    (double) (pixy1.blocks[ind1].y)) / 2.0;

            // Get z world coordinates using stereo
            double x_diff = (double) (pixy0.blocks[ind0].x) - (double) (pixy1.blocks[ind1].x);
            double z_world = fx * CAM_HOR_SEP / x_diff;

            // Convert x and y image coordinates to world coordinates
            double x_world = (x_avg - cx) * z_world / fx;
            double y_world = CAM_HEIGHT - (y_avg - cy) * z_world / fy;

            // Calculate real width/height
            /* double distTo0 = std::sqrt(std::pow(x_world + CAM_HOR_SEP/2, 2) + */
            /*         std::pow(y_world - CAM_HEIGHT, 2) + std::pow(z_world, 2)); */
            /* double distTo1 = std::sqrt(std::pow(x_world - CAM_HOR_SEP/2, 2) + */
            /*         std::pow(y_world - CAM_HEIGHT, 2) + std::pow(z_world, 2)); */
            /* double real_w0 = (pixy0.blocks[ind0].width) * z_world / fx; */
            /* double real_h0 = (pixy0.blocks[ind0].height) * z_world / fy; */
            /* double real_w1 = (pixy1.blocks[ind1].width) * z_world / fx; */
            /* double real_h1 = (pixy1.blocks[ind1].height) * z_world / fy; */
            double real_w = (double) (pixy0.blocks[ind0].width + pixy1.blocks[ind1].width) / 2
                * z_world / fx;
            double real_h = (double) (pixy0.blocks[ind0].height + pixy1.blocks[ind1].height) / 2
                * z_world / fy;

            // Timing
            double t_elapsed = timer_elapsed(&t0);

            // Prepare current ball position in car coordinates
            Point3D ptNow = Point3D(x_world, z_world, y_world, t_elapsed);

            /* Validate with stereo results */
            if (x_diff < 0) { // x_diff makes no sense
                continue;
            }
            if (z_world > Z_MAX) { // max z
                continue;
            }
            if (!(BALL_DIAM * (1.0-SIZE_TOL) < real_w && real_w < BALL_DIAM * (1.0+SIZE_TOL))) {
                // wrong width
                continue;
            }
            if (!(BALL_DIAM * (1.0-SIZE_TOL) < real_h && real_h < BALL_DIAM * (1.0+SIZE_TOL))) {
                // wrong height
                continue;
            }

            /* Validate with trajectory prediction */
            Point3D ptExp;
            if (points.size() > 1) { // don't check trajectory if there is only one point
                // Max change in position from expected next position
                Point3D ptFirst = points.front();
                double t_first = ptFirst.getT();
                ptExp = predictPosition(beta_x, beta_y, beta_z, ptFirst, t_elapsed);
                // Current ball position is too far from expected position
                if ((ptNow - ptExp).maxAbsSpatial() > MAX_POS_DIFF) {
                    continue;
                }
            }

            /* From here on, we have accepted this point */
            missed = false;
            numMisses = 0;

            // Add to points[]
            points.push_back(ptNow);
            if (points.size() > 100) { // keep maximum size at 100
                points.pop_front();
            }

            /* Predict trajectory */
            if (points.size() > 2) { // ensure enough points for regression
                int regNum = points.size(); // std::min(REG_POINTS_TO_USE, (int) points.size());
                // Populate predictor matrices and response vectors
                arma::mat t(regNum, 2); // for global x, y
                arma::mat t2(regNum, 3); // for global z
                arma::vec res_x(regNum); // global x
                arma::vec res_y(regNum); // global y
                arma::vec res_z(regNum); // global z
                // Calculate bounce time
                double t_bounce = predictBounceTime(beta_z, points.front().getZ());
                for (int i = 0; i < regNum; i++) {
                    int ind = points.size() - regNum + i;
                    // Populate predictor matrices
                    t(i, 0) = 1; // constant for bias/offset
                    t2(i, 0) = 1;
                    t(i, 1) = points[ind].getT() - points.front().getT(); // t (linear term)
                    t2(i, 1) = t(i, 1);
                    t2(i, 2) = std::pow(t2(i, 1), 2); // t^2 (quadratic term)

                    // Populate response vectors
                    res_x(i) = points[ind].getX();
                    res_y(i) = points[ind].getY();
                    res_z(i) = points[ind].getZ();

                    // Has it bounced?
                    if (t(i, 1) > t_bounce) { // yes
                        // Transform z, t to lie on the original parabola
                        double tnew = t_bounce - (t(i, 1) - t_bounce);
                        double znew = res_z(i) / COEF_REST;
                        printf(", %.3f, %.3f, %.3f\n", t_bounce, tnew, znew);
                        t(i, 1) = tnew;
                        t2(i, 1) = tnew;
                        t2(i, 2) = std::pow(tnew, 2);
                        res_z(i) = znew;
                        /* res_z(i) = (beta_z(1) + (1-std::sqrt(COEF_REST))*beta_z(2)*t_bounce) */
                        /*     * t(i, 1) - res_z(i); */
                    }
                }

                // Perform regression
                arma::vec new_beta_x = (t.t() * t).i() * t.t() * res_x;
                arma::vec new_beta_y = (t.t() * t).i() * t.t() * res_y;
                arma::vec new_beta_z = (t2.t() * t2).i() * t2.t() * res_z;
                printf(", %.3f, %.3f, %.3f\n", new_beta_z(0), new_beta_z(1), new_beta_z(2));

                /* Validate results */
                double acc = new_beta_z(2) * 2 / 1000;
                if (acc > -3.0) {
                    // Reset regression variables
                    new_beta_x.fill(0);
                    new_beta_y.fill(0);
                    new_beta_z.fill(0);
                    // Flush points[]
                    points.clear();
                    // But don't count this as a miss - we just need to restart our model
                }

                /* Update regression parameters */
                beta_x = new_beta_x;
                beta_y = new_beta_y;
                beta_z = new_beta_z;
            }

            // Predict bounce location
            /* Point3D ptBounce = predictBounceLocation(beta_x, beta_y, beta_z, points.front()); */

            /* Print all info */
/* printData: */
            printf("%.3f", ptNow.getT());
            printf(", %.3f, %.3f, %.3f", beta_z(0), beta_z(1), beta_z(2));
            printf(", %.3f, %.3f, %.3f, %d",
                    ptExp.getX(), ptExp.getY(), ptExp.getZ(), numMisses);
            printf(", %.3f, %.3f, %.3f, %d",
                    ptNow.getX(), ptNow.getY(), ptNow.getZ(), points.size());

            /* printf(", %.3f, %.3f, %.3f, %.3f\n", */
            /*         ptBounce.getX(), ptBounce.getY(), ptBounce.getZ(), ptBounce.getT()); */
            printf(", %.5f\n", timer_elapsed(&t1));

            /* Break so that it examines no further plausible pairs and examines next frame */
            break;
        }
        // If all plausible pairs were rejected, we missed.
        if (missed) { numMisses++; }
        /* Flush points[] if too many misses */
        if (numMisses > MAX_MISSES) {
            points.clear();
        }
    }

    return 0;
}

