#include "controller/stanley.h"

Stanley::Stanley(double k, double ks) {
    // Vehicle Config
    this->WB = this->car.WB;

    // Stanley Parameters
    this->k = k;
    this->ks = ks;

    this->relative_yaw = M_PI / 2.0;
    this->target_node = 0;
}

Stanley::Stanley() {}
Stanley::~Stanley() {}

void Stanley::stanley_control(std::vector<Path> refPoses, double current_velocity) {
    this->refPoses = refPoses;
    this->v = current_velocity;
    this->update_state();

    this->stanley_steer_calc();
}

void Stanley::update_state() {
    this->front_x = ((this->WB / 2.0) * cos(this->relative_yaw));
    this->front_y = ((this->WB / 2.0) * sin(this->relative_yaw));
}

void Stanley::stanley_steer_calc() {
    double dx = this->refPoses[this->target_node].x;
    double dy = this->refPoses[this->target_node].y;

    double front_axle_vec_rot_90_x = cos(this->relative_yaw - M_PI / 2.0);
    double front_axle_vec_rot_90_y = sin(this->relative_yaw - M_PI / 2.0);

    double e = dx * front_axle_vec_rot_90_x + dy * front_axle_vec_rot_90_y;

    double theta_e = pi_2_pi(this->refPoses[target_node].yaw - relative_yaw);

    this->delta = theta_e + atan2(this->k * e, this->v + ks);
}

double Stanley::pi_2_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double Stanley::getDelta() {
    return delta;
}