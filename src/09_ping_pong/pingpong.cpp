#include <cstdio>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>

#include <armadillo>
#include <wiringSerial.h>

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

        double distTo(Point3D& p) {
            return (p - *this).norm();
        }

        double norm() {
            return std::sqrt(std::pow(this->x, 2) +
                    std::pow(this->y, 2) +
                    std::pow(this->z, 2));
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
const double PADDLE_X = -230;
const double PADDLE_Y = -130;
const double PADDLE_Z = +110;
const double BALL_DIAM = 1.5 * 25.4; // diameter of ball
const double COEF_REST = 0.3; // coefficient of restitution
const double G_ACC = -9.8; // gravitational field strength

// Trajectory prediction
const int NUM_REGRESSION_POINTS = 10;
const int NUM_COEF_REST_POINTS = 20;
const double MIN_COEF_REST = 0.2;
const double MAX_COEF_REST = 0.8;

// Validation
const double SIZE_TOL = 1;
const double Z_MAX = 2000;
const double MAX_POS_DIFF = 50; // mm
const int MAX_MISSES = 3;

/* Global variables */
// General execution
int verbose = 0; // higher values = more verbose
// Estimates of x/y/z kinematic parameters
arma::vec beta_x(2); // [0] = offset x(0), [1] = vx(0)
arma::vec beta_y(2); // [0] = offset y(0), [1] = vy(0)
arma::vec beta_z(3); // [0] = offset z(0), [1] = vz(0), [2] = acceleration g
double beta_R = COEF_REST;
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
    double newx = pt1.getX() + bx(1) * t_bounce;
    double newy = pt1.getY() + by(1) * t_bounce;
    double newz = pt1.getZ() + bz(1) * t_bounce
        + bz(2) * std::pow(t_bounce, 2);

    return Point3D(newx, newy, newz, t_bounce);
}

// Predict position of ball at time t, using the bouncing ball model
// (handles up to one bounce)
//   bx, by, bz: vectors of length 2, 2, 3 respectively;
//               coefficients of naive kinematic fit
//   pt1: position of ball at time 0; point on which bx, by, bz are based
Point3D predictPosition(arma::vec& bx, arma::vec& by, arma::vec& bz, double bR,
        Point3D& pt1, double t) {
    double g = bz(2) * 2; // G_ACC * 1000; // bz(2) * 2
    double newx = pt1.getX() + bx(1) * (t - pt1.getT());
    double newy = pt1.getY() + by(1) * (t - pt1.getT());
    double newz = pt1.getZ() + bz(1) * (t - pt1.getT())
        + 0.5 * g * std::pow(t - pt1.getT(), 2);

    if (newz < 0) { // the ball will have bounced
        // Calculate the time of the first bounce
        double t_bounce = predictBounceTime(bz, pt1.getZ()) + pt1.getT();
        // Calculate z velocity at time of bounce
        double vz = bz(1) + g * (t_bounce - pt1.getT());
        // New z velocity after bounce
        vz = -std::sqrt(bR) * vz;
        newz = 0 + vz * (t - t_bounce) + 0.5 * g * std::pow(t - t_bounce, 2);
    }

    return Point3D(newx, newy, newz, t);
}

// Predict time and location that the ball will reach a given height (if it will),
// _after_ the first bounce, then returns the point that the robot should move to.
//   bx, by, bz: vectors of length 2, 2, 3 respectively;
//               coefficients of naive kinematic fit
//   pt1: position of ball at time 0; point on which bx, by, bz are based
//   height: target height of ball
Point3D predictLocationGoto(arma::vec& bx, arma::vec& by, arma::vec& bz, double bR,
        Point3D& pt1, double height) {
    // time of bounce
    double t_bounce = predictBounceTime(bz, pt1.getZ());
    // z velocity before bounce
    double z_vel_orig = bz(1) + 2 * bz(2) * (t_bounce - pt1.getT());
    // z velocity after bounce
    double z_vel_new = z_vel_orig * std::sqrt(bR);
    // times for ball at target height
    double discr = std::pow(z_vel_new, 2)
        + 4 * height * bz(2); // discriminant of quadratic formula
    if (discr < 0) { // will not reach this height
        // so just drive to landing location
        Point3D ptLand = predictBounceLocation(bx, by, bz, pt1);
        return ptLand - Point3D(PADDLE_X, PADDLE_Y, 0);
    }
    double t_target0 = t_bounce + (-z_vel_new + std::sqrt(discr)) / (2 * bz(2));
    double t_target1 = t_bounce + (-z_vel_new - std::sqrt(discr)) / (2 * bz(2));
    // locations of these target points
    Point3D loc_target0 = Point3D(pt1.getX() + bx(1) * t_target0,
            pt1.getY() + by(1) * t_target0,
            pt1.getZ() + bz(1) * t_target0 + bz(2) * std::pow(t_target0, 2), t_target0);
    Point3D loc_target1 = Point3D(pt1.getX() + bx(1) * t_target1,
            pt1.getY() + by(1) * t_target1,
            pt1.getZ() + bz(1) * t_target1 + bz(2) * std::pow(t_target1, 2), t_target1);
    // distances to target points
    double dist_target0 = loc_target0.norm();
    double dist_target1 = loc_target1.norm();
    // choose closer target
    Point3D ptTarget = dist_target0 < dist_target1 ? loc_target0 : loc_target1;
    return ptTarget - Point3D(PADDLE_X, PADDLE_Y, 0);
}

/* Main method */
int main(int argc, char *argv[]) {
    verbose = 0;
    if (argc > 0) {
        verbose = atoi(argv[0]);
    }

    // Connect cameras
    Pixy pixy0(ADDR0);
    Pixy pixy1(ADDR1);

    if (pixy0.getLink().getAddr() != ADDR0 && verbose > 0) {
        fprintf(stderr, "Pixy 0 failed to initialize with I2C address %x.\n", ADDR0);
    }
    if (pixy1.getLink().getAddr() != ADDR1 && verbose > 0) {
        fprintf(stderr, "Pixy 1 failed to initialize with I2C address %x.\n", ADDR1);
    }

    // Connect to PSoC
    int baud = 9600;
    int psoc_handle = serialOpen("/dev/ttyAMA0", baud);
    if (psoc_handle < 0 && verbose > 0) {
        fprintf(stderr, "PSoC UART channel failed to initialize.\n");
    }

    // Timing
    timer_obj t0;
    timer_start(&t0);

    // Kinematics
    std::deque<Point3D> points;

    bool stopped = false;
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
                ptExp = predictPosition(beta_x, beta_y, beta_z, beta_R, ptFirst, t_elapsed);
                // Current ball position is too far from expected position
                if ((ptNow - ptExp).maxAbsSpatial() > MAX_POS_DIFF) {
                    /* continue; */
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
                arma::vec best_beta_x;
                arma::vec best_beta_y;
                arma::vec best_beta_z;
                double best_beta_R = COEF_REST;
                double best_mse = INFINITY;
                // Calculate bounce time
                double t_bounce = predictBounceTime(beta_z, points.front().getZ());
                // Regress using current coefficient of restitution
                double new_beta_R = beta_R;
                /* Populate predictor matrices and response vectors */
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

                    // Populate response vectors
                    res_x(i) = points[i].getX();
                    res_y(i) = points[i].getY();
                    res_z(i) = points[i].getZ();

                    // Has it bounced?
                    if (t(i, 1) > t_bounce) { // If so...
                        // Transform z, t to lie on the original parabola
                        double tnew = t_bounce - (t(i, 1) - t_bounce) / std::sqrt(new_beta_R);
                        double znew = res_z(i) / new_beta_R;
                        /* printf(", %10.3f, %10.3f, %10.3f\n", t_bounce, tnew, znew); */
                        t2(i, 1) = tnew;
                        t2(i, 2) = std::pow(tnew, 2);
                        res_z(i) = znew;
                        /* res_z(i) = (beta_z(1) + (1-std::sqrt(beta_R))*beta_z(2)*t_bounce) */
                        /*     * t(i, 1) - res_z(i); */
                    }
                }

                /* Perform regression to estimate new kinematic coefficients */
                arma::vec new_beta_x = (t.t() * t).i() * t.t() * res_x;
                arma::vec new_beta_y = (t.t() * t).i() * t.t() * res_y;
                arma::vec new_beta_z = (t2.t() * t2).i() * t2.t() * res_z;
                /* printf(", %10.3f, %10.3f, %10.3f\n", new_beta_z(0), new_beta_z(1), new_beta_z(2)); */

                // Iterate through potential coefficients of restitution; choose the one with the smallest mean-squared error
                /* Only do this if we are pretty sure it's bounced */
                arma::uvec inds_after_bounce = arma::find(t.col(1) > t_bounce);
                if (inds_after_bounce.n_elem >= 3) {
                    for (int i = 0; i < NUM_COEF_REST_POINTS; i++) {
                        double new_beta_R = MIN_COEF_REST
                            + (MAX_COEF_REST - MIN_COEF_REST) * i / (NUM_COEF_REST_POINTS - 1);
                        /* Populate predictor matrices and response vectors */
                        for (int i = 0; i < points.size(); i++) {
                            // Populate predictor matrices
                            t(i, 0) = 1; // constant for bias/offset
                            t2(i, 0) = 1;
                            t(i, 1) = points[i].getT() - points.front().getT(); // t (linear term)
                            t2(i, 1) = t(i, 1);
                            t2(i, 2) = std::pow(t2(i, 1), 2); // t^2 (quadratic term)

                            // Populate response vectors
                            res_x(i) = points[i].getX();
                            res_y(i) = points[i].getY();
                            res_z(i) = points[i].getZ();

                            // Has it bounced?
                            if (t(i, 1) > t_bounce) { // If so...
                                // Transform z, t to lie on the original parabola
                                double tnew = t_bounce - (t(i, 1) - t_bounce) / std::sqrt(new_beta_R);
                                double znew = res_z(i) / new_beta_R;
                                if (verbose > 3) {
                                    printf(", %10.3f, %10.3f, %10.3f\n", t_bounce, tnew, znew);
                                }
                                t2(i, 1) = tnew;
                                t2(i, 2) = std::pow(tnew, 2);
                                res_z(i) = znew;
                                /* res_z(i) = (beta_z(1) + (1-std::sqrt(beta_R))*beta_z(2)*t_bounce) */
                                /*     * t(i, 1) - res_z(i); */
                            }
                        }

                        /* Perform regression to estimate new kinematic coefficients */
                        new_beta_x = (t.t() * t).i() * t.t() * res_x;
                        new_beta_y = (t.t() * t).i() * t.t() * res_y;
                        new_beta_z = (t2.t() * t2).i() * t2.t() * res_z;
                        /* printf(", %10.3f, %10.3f, %10.3f\n", new_beta_z(0), new_beta_z(1), new_beta_z(2)); */

                        /* Calculate MSE for z coordinate and select best MSE */
                        arma::vec est_z = t2 * new_beta_z;
                        double mse_z = std::pow(arma::norm(res_z - est_z), 2) / res_z.n_elem;
                        if (verbose > 3) {
                            printf(", %.3f, %.3f\n", new_beta_R, mse_z);
                        }
                        if (mse_z < best_mse) {
                            best_mse = mse_z;
                            best_beta_x = new_beta_x;
                            best_beta_y = new_beta_y;
                            best_beta_z = new_beta_z;
                            best_beta_R = new_beta_R;
                        }
                    }
                } else {
                    best_beta_x = new_beta_x;
                    best_beta_y = new_beta_y;
                    best_beta_z = new_beta_z;
                    best_beta_R = new_beta_R;
                }

                /* Validate results */
                double acc = best_beta_z(2) * 2 / 1000;
                if (acc > -3.0) {
                    // Reset regression variables
                    /* best_beta_x.fill(0); */
                    /* best_beta_y.fill(0); */
                    /* best_beta_z.fill(0); */
                    /* best_beta_R = COEF_REST; */
                    best_beta_x = beta_x;
                    best_beta_y = beta_y;
                    best_beta_z = beta_z;
                    best_beta_R = beta_R;
                    // Flush points[]
                    points.clear();
                    // But don't count this as a miss - we just need to restart our model
                }

                /* Update regression parameters */
                beta_x = best_beta_x;
                beta_y = best_beta_y;
                beta_z = best_beta_z;
                beta_R = best_beta_R;
            }

            /* // Predict bounce location */
            /* Point3D ptBounce = predictBounceLocation(beta_x, beta_y, beta_z, points.front()); */

            // Predict location to need to drive to
            Point3D ptTarget = predictLocationGoto(beta_x, beta_y, beta_z, beta_R,
                    points.front(), PADDLE_Z);

            /* Print all info */
            if (verbose > 1) {
                printf("%10.3f", ptNow.getT());
                printf(", %10.3f, %10.3f, %10.3f, %10.3f", beta_z(0), beta_z(1), beta_z(2), beta_R);
                printf(", %10.3f, %10.3f, %10.3f, %2d",
                        ptExp.getX(), ptExp.getY(), ptExp.getZ(), numMisses);
                printf(", %10.3f, %10.3f, %10.3f, %3d",
                        ptNow.getX(), ptNow.getY(), ptNow.getZ(), points.size());

                /* printf(", %10.3f, %10.3f, %10.3f, %10.3f\n", */
                /*         ptBounce.getX(), ptBounce.getY(), ptBounce.getZ(), ptBounce.getT()); */
                printf(", %7.5f\n", timer_elapsed(&t1));
            }
            /* double timeLeft = ptBounce.getT() - ptNow.getT() + points.front().getT(); */
            /* if (points.size() > 2) { */
            /*     printf("%10.3f, %10.3f, %10.3f, %10.3f\n", */
            /*             timeLeft, ptBounce.getX(), ptBounce.getY(), ptBounce.getZ()); */
            /* } */
            double timeLeft = ptTarget.getT() - ptNow.getT() + points.front().getT();
            if (points.size() > 2) {
                printf("%10.3f, %10.3f, %10.3f, %10.3f\n",
                        timeLeft, ptTarget.getX(), ptTarget.getY(), ptTarget.getZ());
            }

            /* Send instructions to PSoC */
            if (points.size() > 4 && timeLeft > 0.05) {
                /* serialFlush(psoc_handle); */
                /* printf("GO(%.3f, %.3f, %.3f, %.3f)\n", */
                /*         ptBounce.getX() - PADDLE_X, ptBounce.getY() - PADDLE_Y - 100, */
                /*         timeLeft, 0.6); */
                /* serialPrintf(psoc_handle, "GO(%.3f, %.3f, %.3f, %.3f)\n\0", */
                /*         ptBounce.getX() - PADDLE_X, ptBounce.getY() - PADDLE_Y - 100, */
                /*         timeLeft, 0.6); */
                printf("GO(%.3f, %.3f, %.3f, %.3f)\n",
                        ptTarget.getX(), ptTarget.getY(), timeLeft, 0.6);
                serialPrintf(psoc_handle, "GO(%.3f, %.3f, %.3f, %.3f)\n\0",
                        ptTarget.getX(), ptTarget.getY(), timeLeft, 0.6);
                stopped = true;
                break;
            }

            /* Break so that it examines no further plausible pairs and examines next frame */
            break;
        }
        // If all plausible pairs were rejected, we missed.
        if (missed) { numMisses++; }
        /* Flush points[] if too many misses */
        if (numMisses > MAX_MISSES) {
            points.clear();
        }
        if (stopped) {
            std::string temp; std::cin >> temp;
            // Reset: flush points[] and reset betas
            points.clear();
            beta_x.fill(0);
            beta_y.fill(0);
            beta_z.fill(0);
            beta_R = COEF_REST;
            stopped = false;
        }
    }

    return 0;
}

