#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include "object3d.hpp"

enum L_T{DIR,POINT,AREA};
class Light {
public:
    Light() = default;
    virtual Vector3f getPosition() const { return Vector3f::ZERO; }
    virtual ~Light() = default;
    virtual L_T getType() const = 0; 
    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const = 0;
};


class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c) {
        direction = d.normalized();
        color = c;
    }

    ~DirectionalLight() override = default;

    ///@param p unsed in this function
    ///@param distanceToLight not well defined because it's not a point light
    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = -direction;
        col = color;
    }
    L_T getType() const override {
        return DIR;
    }

private:

    Vector3f direction;
    Vector3f color;

};

class PointLight : public Light {
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c) {
        position = p;
        color = c;
    }

    ~PointLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }

    L_T getType() const override {
        return POINT;
    }

    Vector3f getPosition() const override {
        return position;
    }

    Vector3f getColor() const {
        return color;
    }
private:

    Vector3f position;
    Vector3f color;

};



// 该部分代码无用！！！！！
class AreaLight : public Light {
public:
    AreaLight() = delete;

    // 构造函数：位置、法线方向、宽度、高度、颜色
    AreaLight(const Vector3f &c, float r, const Vector3f &col,const Vector3f &emis,const Vector3f &n) {
        center = c;
        radius = r;
        color = col;
        emission= emis;
        // Intensity=emis;
        normal = n.normalized(); 
        Vector3f tmp = fabs(normal.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0);
        x_axis = Vector3f::cross(tmp, normal).normalized();
        y_axis = Vector3f::cross(normal, x_axis).normalized();
    }

    ~AreaLight() override = default;

    L_T getType() const override {
        return AREA;
    }

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        Vector3f rp = random_point();
        Vector3f toLight = rp - p;
        float dist2 = toLight.squaredLength();
        dir = toLight.normalized();
        float cos_theta_light = Vector3f::dot(-dir, normal);  
        if (cos_theta_light <= 0) {
            col = Vector3f::ZERO;
            return;
        }

        float factor = (Intensity * cos_theta_light) / dist2;
        col = color * factor * getArea(); 
    }

    float getArea() const {
        return 3.14159*radius * radius;
    }

    Vector3f getPosition() const override{
        return center;
    }

    Vector3f getColor() const {
        return color;
    }
    Vector3f getNormal() const {
        return normal;
    }
    float getIntensity() const {
        return Intensity;
    }

    Vector3f getemission() const {
        return emission;
    }

    Vector3f random_point() const {
        float r1 = static_cast<float>(rand()) / RAND_MAX;
        float r2 = static_cast<float>(rand()) / RAND_MAX;
        float r = radius * sqrt(r1);
        float theta = 2 * 3.1415926f * r2;
        float x = r * cos(theta);
        float y = r * sin(theta);
        return center + x * x_axis + y * y_axis;
    }





private:
    float radius;
    Vector3f center;
    Vector3f color;
    Vector3f normal; // 法线方向
    float Intensity;
    Vector3f emission;
    Vector3f x_axis;
    Vector3f y_axis;

};

#endif // LIGHT_H
