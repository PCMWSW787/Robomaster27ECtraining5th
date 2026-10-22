//
// Created by Siwei Wang on 2026/10/22.
//

#include "../Inc/quaternion.hpp"
#include "math.h"

Quaternion Quaternion::operator*(const Quaternion& q) const
{
    return Quaternion(
        w_ * q.w_ - x_ * q.x_ - y_ * q.y_ - z_ * q.z_,
        w_ * q.x_ + x_ * q.w_ + y_ * q.z_ - z_ * q.y_,
        w_ * q.y_ - x_ * q.z_ + y_ * q.w_ + z_ * q.x_,
        w_ * q.z_ + x_ * q.y_ - y_ * q.x_ + z_ * q.w_
        );
}

Quaternion Quaternion::operator+(const Quaternion& q) const
{
    return Quaternion(
        w_ + q.w_, x_+ q.x_, y_+ q.y_, z_+ q.z_
        );
}

Quaternion Quaternion::operator-(const Quaternion& q) const
{
    return Quaternion(
        w_ - q.w_, x_ - q.x_, y_ - q.y_, z_ - q.z_
        );
}


Quaternion Quaternion::operator*(const float& f) const
{
    return Quaternion(
        w_ * f, x_ * f, y_ * f, z_ * f
        );
}

void Quaternion::normalize()
{
    float l = sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
    if (l <= 0.0f) return;
    w_ /= l;
    x_ /= l;
    y_ /= l;
    z_ /= l;
}
