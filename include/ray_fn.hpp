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
        float cosi = -Vector3f::dot(dir, n);  // 入射角的余弦
        float eta = refractive_index; // η = n1 / n2

        if(cosi>=0.0f){
            eta= 1.0f / eta; // 如果入射角在法线外侧，交换折射率

        }
        else{
            n = -n; // 如果入射角在法线内侧，法线方向取反
            cosi = -cosi; // 入射角的余弦取正
        }

        float sin2t = pow(eta,2.0f) * (1.0f - pow(cosi,2.0f));
        if (sin2t > 1.0f) {
            return reflect_d(dir, n.normalized()); // 全反射
        }

        float cost = sqrt(1.0f - sin2t);
        return (dir * eta + n * (eta * cosi - cost)).normalized();
    }

};