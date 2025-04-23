#include "engineering_robot_controller/engineering_robot_controller.hpp"

namespace Engineering_robot_RM2025_Pnx {
/**
 * @brief 将欧拉角转换为四元数
 * @param roll 绕X轴的旋转角度(弧度)
 * @param pitch 绕Y轴的旋转角度(弧度)
 * @param yaw 绕Z轴的旋转角度(弧度)
 * @return 对应的四元数 [w, x, y, z]
 */
std::vector<double> eulerToQuaternion(double roll, double pitch, double yaw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    std::vector<double> q(4);
    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z

    return q;
}

/**
 * @brief 将欧拉角转换为四元数(向量输入版本)
 * @param euler 欧拉角向量 [roll, pitch, yaw] (弧度)
 * @return 对应的四元数 [w, x, y, z]
 * @throws std::invalid_argument 如果输入向量大小不为3
 */
std::vector<double> eulerToQuaternion(const std::vector<double>& euler) {
    if (euler.size() != 3) {
        throw std::invalid_argument("Euler angles vector must have exactly 3 elements");
    }
    return eulerToQuaternion(euler[0], euler[1], euler[2]);
}

/**
 * @brief 将四元数转换为欧拉角
 * @param q 四元数 [w, x, y, z]
 * @return 对应的欧拉角 [roll, pitch, yaw] (弧度)
 */
std::vector<double> quaternionToEuler(const std::vector<double>& q) {
    std::vector<double> angles(3);

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

}// Engineering_robot_RM2025_Pnx