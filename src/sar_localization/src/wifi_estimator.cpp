#include "wifi_estimator.h"

WiFiEstimator::WiFiEstimator():
    frame_count(0), current_time(-1)
{
    odometry[0](0) = 0.0;
    odometry[0](1) = 10.0;
    odometry[0](2) = 10.0;
    odometry[0](3) = 0;
    odometry[0](4) = 0;
    odometry[0](5) = 0;
    odometry[0](6) = 0;
    odometry[0](7) = 0;
    odometry[0](8) = -9.8;

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        IMU_linear[i].setZero();
        IMU_angular[i].setIdentity();
        IMU_cov[i].setZero();
        IMU_cov_nl[i].setZero();
    }
    Rs[0].setIdentity();
}

void WiFiEstimator::processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;
    if (frame_count != 0)
    {
        Quaterniond q(IMU_angular[frame_count]);
        Quaterniond dq(1,
                       angular_velocity(0) * dt / 2,
                       angular_velocity(1) * dt / 2,
                       angular_velocity(2) * dt / 2);
        dq.w() = 1 - dq.vec().transpose() * dq.vec();
        IMU_angular[frame_count] = (q * dq).normalized();

        Rs[frame_count] = Rs[frame_count - 1] * IMU_angular[frame_count];

        IMU_linear[frame_count].segment<3>(0) += IMU_linear[frame_count].segment<3>(3) * dt + IMU_angular[frame_count] * linear_acceleration * dt * dt / 2;
        IMU_linear[frame_count].segment<3>(3) += IMU_angular[frame_count] * linear_acceleration * dt;
        IMU_linear[frame_count](6) += dt;

        {
            Matrix<double, 6, 6> F = Matrix<double, 6, 6>::Identity();
            F.block<3, 3>(0, 3) = dt * Matrix3d::Identity();

            Matrix<double, 6, 3> G = Matrix<double, 6, 3>::Zero();
            G.block<3, 3>(0, 0) = 0.5 * dt * dt * Matrix3d::Identity();
            G.block<3, 3>(3, 0) = dt * Matrix3d::Identity();

            IMU_cov[frame_count] = F * IMU_cov[frame_count] * F.transpose() + G * acc_cov * G.transpose();
        }
    }
}

SolutionContainer WiFiEstimator::processWiFi(const vector<pair<int, Vector3d>> &wifi)
{
    ROS_INFO("Adding AP measurements %lu", wifi.size());
    ROS_INFO("Solving %d", frame_count);

    for (int i = 0; i < int(wifi.size()); i++)
        wifi_measurement[frame_count][wifi[i].first] = wifi[i].second;

    SolutionContainer solution = solveOdometry();
    frame_count++;
    return solution;
}

SolutionContainer WiFiEstimator::solveOdometry()
{
    if (frame_count < MIN_FRAME_CNT)
        return SolutionContainer();
    else
        solveOdometryLinear();
    for (int i = 0; i <= frame_count; i++)
        odometry[i] = x.segment<9>(i * 9);

    SolutionContainer solution_container;
    solution_container.p = odometry[frame_count].segment<3>(0);
    solution_container.v = odometry[frame_count].segment<3>(3);
    solution_container.g = odometry[frame_count].segment<3>(6);
    solution_container.q = Rs[frame_count];
    for (int i = 0; i < NUMBER_OF_AP; i++)
    {
        //    solution_container.p_ap[i] = x.segment<3>((frame_count + 1) * 9 + i * 3);
    }
    return solution_container;
}

void WiFiEstimator::solveOdometryLinear()
{
    int n_state = (frame_count + 1) * 9 + NUMBER_OF_AP * 3;

    MatrixXd A(n_state, n_state);
    VectorXd b(n_state);
    A.setZero();
    b.setZero();

    A.topLeftCorner(3, 3) = Matrix<double, 3, 3>::Identity() * 10000;
    b.head(3) = odometry[0].head(3) * 10000;

    for (int i = 0; i < frame_count; i++)
    {
        MatrixXd tmp_A(9, n_state);
        tmp_A.setZero();
        VectorXd tmp_b(9);
        tmp_b.setZero();

        double dt = IMU_linear[i + 1](6);
        ROS_DEBUG("%d dt: %lf", i, dt);
        tmp_A.block<3, 3>(0, i * 9)           = -Rs[i].inverse();
        tmp_A.block<3, 3>(0, i * 9 + 3)       = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, i * 9 + 6)       = dt * dt / 2 * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, (i + 1) * 9)     = Rs[i].inverse();
        tmp_b.block<3, 1>(0, 0)               = IMU_linear[i + 1].segment<3>(0);

        tmp_A.block<3, 3>(3, (i + 1) * 9 + 3) = Rs[i].inverse() * Rs[i + 1];
        tmp_A.block<3, 3>(3, i * 9 + 3)       = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, i * 9 + 6)       = dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0)               = IMU_linear[i + 1].segment<3>(3);

        tmp_A.block<3, 3>(6, (i + 1) * 9 + 6) = Rs[i].inverse() * Rs[i + 1];
        tmp_A.block<3, 3>(6, i * 9 + 6)       = -Matrix3d::Identity();
        tmp_b.block<3, 1>(6, 0)               = Vector3d::Zero();

        //Matrix<double, 9, 9> cov = Matrix<double, 9, 9>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //cov.block<3, 3>(6, 6) = gra_cov;

        //MatrixXd cov_inv = cov.inverse();

        MatrixXd r_A = tmp_A.block<9, 18>(0, i * 9).transpose() /* cov_inv */ * tmp_A.block<9, 18>(0, i * 9);
        VectorXd r_b = tmp_A.block<9, 18>(0, i * 9).transpose() /* cov_inv */ * tmp_b;
        A.block<18, 18>(i * 9, i * 9) += r_A;
        b.segment<18>(i * 9)          += r_b;
    }


    int MAX_BOX = 10;
    Vector3d ap[NUMBER_OF_AP];
    ap[0] = Vector3d( MAX_BOX, -MAX_BOX,  MAX_BOX);
    ap[1] = Vector3d(-MAX_BOX,  MAX_BOX,  MAX_BOX);
    ap[2] = Vector3d(-MAX_BOX, -MAX_BOX, -MAX_BOX);
    ap[3] = Vector3d( MAX_BOX,  MAX_BOX, -MAX_BOX);

    for (int i = 0; i <= frame_count; i++)
        for (int j = 0; j < NUMBER_OF_AP; j++)
        {
            //Vector3d sar = Rs[i].inverse() * (ap[j] - x.segment<3>(i * 9));
            //ROS_DEBUG("%d %d", i, j);
            //ROS_DEBUG_STREAM("sar1" << sar.normalized().transpose());
            //ROS_DEBUG_STREAM("sar2" << wifi_measurement[i][j].transpose());

            MatrixXd tmp_A(2, 6);
            tmp_A.setZero();

            MatrixXd reduce(2, 3);
            reduce <<
                   wifi_measurement[i][j](2), 0, -wifi_measurement[i][j](0),
                                    0, wifi_measurement[i][j](2), -wifi_measurement[i][j](1);

            tmp_A.block<2, 3>(0, 0) = reduce * Rs[i].inverse() * -Matrix3d::Identity();
            tmp_A.block<2, 3>(0, 3) = reduce * Rs[i].inverse() * Matrix3d::Identity();
            MatrixXd r_A = tmp_A.transpose() * tmp_A;
            int ii = i * 9, jj = (frame_count + 1) * 9 + j * 3;
            A.block<3, 3>(ii, ii) += r_A.block<3, 3>(0, 0);
            A.block<3, 3>(ii, jj) += r_A.block<3, 3>(0, 3);
            A.block<3, 3>(jj, ii) += r_A.block<3, 3>(3, 0);
            A.block<3, 3>(jj, jj) += r_A.block<3, 3>(3, 3);
        }
    x = A.llt().solve(b);
}

