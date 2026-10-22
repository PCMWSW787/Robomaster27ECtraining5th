//
// Created by Siwei Wang on 2026/10/22.
//

#ifndef IMU_PROJECT_QUATERNION_HPP
#define IMU_PROJECT_QUATERNION_HPP

struct Quaternion
{
    Quaternion(float w, float x, float y, float z):w_(w), x_(x), y_(y), z_(z) {}
    Quaternion operator*(const Quaternion &q) const;
    Quaternion operator+(const Quaternion &q) const;
    Quaternion operator-(const Quaternion &q) const;
    Quaternion operator*(const float  &f) const;
    void normalize();
    float w_ = 0.0f, x_ = 0.0f, y_ = 0.0f, z_ = 0.0f;
};

#endif //IMU_PROJECT_QUATERNION_HPP
