#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "ray.hpp"

#include <string>



class RayFunctions {
public:
    // 计算漫反射方向
    Vector3f r_diffuse(const Vector3f &dir, const Vector3f &normal) {
        return dir - 2 * Vector3f::dot(dir, normal) * normal;
    }

    // 计算反射方向
    Vector3f reflect_d(const Vector3f &dir, const Vector3f &normal) {
        return dir - 2 * Vector3f::dot(dir, normal) * normal;
    }

    // 计算折射方向
    Vector3f refract_d(const Vector3f &dir, const Vector3f &normal, float refractive_index) {
        Vector3f n = normal;
        float eta = refractive_index;
        float cos_i = Vector3f::dot(dir, n);

        // 判断是否从内部射出
        if (cos_i > 0) {
            n = -n;              // 翻转法线
            eta = 1 / eta;       // 折射率取倒数
            cos_i = Vector3f::dot(dir, n);  // 更新 cos_i
        }

        float sin2_t = eta * eta * (1 - cos_i * cos_i);
        if (sin2_t > 1) {
            return reflect_d(dir, n); // 全反射
        }
        float cos_t = sqrt(1 - sin2_t);
        return eta * dir - (eta * cos_i + cos_t) * n;
    }
};