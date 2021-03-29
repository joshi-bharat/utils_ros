#pragma once

#include <iostream>

#include <tf2/LinearMath/Quaternion.h>

namespace utils{

    void printTf2Quat(const tf2::Quaternion& q, const std::string& info = ""){

        std::cout << info << q.x() << "\t" << q.y() << "\t" << q.z() << "\t" << q.w() << "\n";
    }
}