#pragma once

#include "utils/car_struct.h"
#include "utils/msg_structs.h"

#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <unistd.h>

class Stanley {

public:
    Stanley(double k, double ks);

    Stanley();
    ~Stanley();

    void stanley_control(std::vector<Path> refPoses, double current_velocity);

    double getDelta();

private:
    // Struct
    Car car;

    // Vehicle State    
    double front_x;
    double front_y;

    // Vehicle Config
    double WB;

    // Stanley Parameters
    double k;
    double ks;

    // Path
    std::vector<Path> refPoses;

    // Target Status
    double delta;
    double v;

    // etc
    double relative_yaw;
    int target_node;

    void update_state();
    void stanley_steer_calc();
    double pi_2_pi(double angle);
};