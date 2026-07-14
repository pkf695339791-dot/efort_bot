#pragma once

#include <array>
#include <string>

namespace tie {

struct CartesianPose {
    double x{};
    double y{};
    double z{};
    double a{180.0};
    double b{};
    double c{180.0};
    int cfgx{};
    int cfg1{};
    int cfg4{};
    int cfg6{};

    CartesianPose shifted(double dz) const;
};

struct JointPose {
    std::array<double, 6> joints{};
};

struct TiePoint {
    std::string point_id;
    CartesianPose pose;
    double confidence{1.0};
    std::string source{"offline"};
};

}  // namespace tie
