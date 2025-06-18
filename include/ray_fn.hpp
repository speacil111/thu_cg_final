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

    // 计算反射方向 没问题
    Vector3f reflect_d(const Vector3f &dir, const Vector3f &normal) {
        return (dir-2.0f*Vector3f::dot(dir, normal) * normal).normalized();
    }

    // 计算折射方向
    Vector3f refract_d(const Vector3f &dir, const Vector3f &normal, float refractive_index) {
        Vector3f n = normal;
        float cosi = Vector3f::dot(dir, n);  // 入射角的余弦
        float eta = refractive_index; // η = n1 / n2

        // 如果从内部向外部走，调整法线方向 & η 取倒数
        if (cosi <0) {
            cosi = -cosi;
            eta = 1.0f / eta;
        }
        else{
            n=-n;
        }

        float sin2t = pow(eta,2.0f) * (1.0f - pow(cosi,2.0f));
        if (sin2t > 1.0f) {
            return reflect_d(dir, n.normalized()); // 全反射
        }

        float cost = sqrt(1.0f - sin2t);
        return (dir * eta + n * (eta * cosi - cost)).normalized();
    }

};