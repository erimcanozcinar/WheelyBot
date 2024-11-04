#include "math.hpp"

Eigen::Matrix3d mathOperations::RotateRoll(double thetha)
{
    double RotX[3][3];
    Eigen::Matrix3d rollMatrix;

    RotX[0][0] = 1;
    RotX[0][1] = 0;
    RotX[0][2] = 0;

    RotX[1][0] = 0;
    RotX[1][1] = cos(thetha);
    RotX[1][2] = -sin(thetha);

    RotX[2][0] = 0;
    RotX[2][1] = sin(thetha);
    RotX[2][2] = cos(thetha);

    rollMatrix << RotX[0][0], RotX[0][1], RotX[0][2], RotX[1][0], RotX[1][1], RotX[1][2], RotX[2][0], RotX[2][1], RotX[2][2];
    return rollMatrix;
}

Eigen::Matrix3d mathOperations::RotatePitch(double thetha)
{
    double RotY[3][3];
    Eigen::Matrix3d pitchMatrix;

    RotY[0][0] = cos(thetha);
    RotY[0][1] = 0;
    RotY[0][2] = sin(thetha);

    RotY[1][0] = 0;
    RotY[1][1] = 1;
    RotY[1][2] = 0;

    RotY[2][0] = -sin(thetha);
    RotY[2][1] = 0;
    RotY[2][2] = cos(thetha);

    pitchMatrix << RotY[0][0], RotY[0][1], RotY[0][2], RotY[1][0], RotY[1][1], RotY[1][2], RotY[2][0], RotY[2][1], RotY[2][2];
    return pitchMatrix;
}

Eigen::Matrix3d mathOperations::RotateYaw(double thetha)
{
    double RotZ[3][3];
    Eigen::Matrix3d yawMatrix;

    RotZ[0][0] = cos(thetha);
    RotZ[0][1] = -sin(thetha);
    RotZ[0][2] = 0;

    RotZ[1][0] = sin(thetha);
    RotZ[1][1] = cos(thetha);
    RotZ[1][2] = 0;

    RotZ[2][0] = 0;
    RotZ[2][1] = 0;
    RotZ[2][2] = 1;

    yawMatrix << RotZ[0][0], RotZ[0][1], RotZ[0][2], RotZ[1][0], RotZ[1][1], RotZ[1][2], RotZ[2][0], RotZ[2][1], RotZ[2][2];
    return yawMatrix;
}

Eigen::Vector3d mathOperations::quat2euler(double w, double x, double y, double z)
{
    double roll, pitch, yaw;
    Eigen::Vector3d orientation;
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    // pitch = asin(2 * (w * y - z * x));
    pitch = 2*atan2(sqrt(1+2*(w*y-x*z)),sqrt(1-2*(w*y-x*z))) - M_PI_2;
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    orientation << roll, pitch, yaw;
    return orientation;
}

Eigen::Matrix3d mathOperations::quat2RotMat(double qw, double qx, double qy, double qz)
{
    double r31, r32, r33, r11, r12, r13, r21, r22, r23;
    Eigen::Matrix3d rMat;
    r11 = 1 - 2 * qy * qy - 2 * qz * qz;
    r12 = 2 * (qx * qy - qz * qw);
    r13 = 2 * (qx * qz + qy * qw);
    r21 = 2 * (qx * qy + qz * qw);
    r22 = 1 - 2 * qx * qx - 2 * qz * qz;
    r23 = 2 * (qy * qz - qx * qw);
    r31 = 2 * (qx * qz - qy * qw);
    r32 = 2 * (qy * qz + qx * qw);
    r33 = 1 - 2 * qx * qx - 2 * qy * qy;
    rMat << r11, r12, r13, r21, r22, r23, r31, r32, r33;
    return rMat;
}

double mathOperations::Numdiff(double currX, double prevX, double dt)
{
    return (currX - prevX) / dt;
}

double mathOperations::Numdiff2nd(double currX, double prevX, double preprevX, double dt)
{
    return (currX - 2*prevX + preprevX) / pow(dt,2);
}

double mathOperations::numIntegral(double In, double prevIn, double prevOut, double dt)
{
    double Out = prevOut + (In + prevIn) * dt / 2;
    return Out;
}

double mathOperations::funcUnwrap(double data, double prevData)
{
    k = 0;
    deltaCorr = 0;


    deltaCorr = (data - prevData)/(2*M_PI);
    k = round(deltaCorr);
    cumSumdeltaCorr = cumSumdeltaCorr + k;

    unwrapedOut = data - 2*M_PI*cumSumdeltaCorr;
    return unwrapedOut;
}

double mathOperations::LowPassFilter(double Input, double prevOut, double freq, double dt)
{
    double out = (dt * freq * Input + prevOut) / (1 + freq * dt);
    return out;
}

Eigen::MatrixXd mathOperations::solveContinuousARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    const double tolerance = 1e-20;
    const int max_iterations = 10;
    int n = A.rows();
    int m = B.cols();

    Eigen::MatrixXd X = Q;
    Eigen::MatrixXd X_new;
    Eigen::MatrixXd K;

    for (int i = 0; i < max_iterations; ++i) {
        // Compute the gain matrix K
        K = (R + B.transpose() * X * B).inverse() * (B.transpose() * X * A);
        // Compute the new X matrix
        X_new = Q + K.transpose() * R * K + (A - B * K).transpose() * X * (A - B * K);
        // Check for convergence
        if ((X_new - X).norm() < tolerance) {
            break;
        }
        X = X_new;
    }

    return X;
}

