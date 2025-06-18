#ifndef SQUARE_H
#define SQUARE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Square: public Object3D {

public:
	Square() {
        center = Vector3f(0, 0, 0);
        edge_length = 1.0f;
        normal = Vector3f(0, -1,0); 
    }

    // a b c are three vertex positions of the triangle
    Square(const Vector3f& center, float edge_length,const Vector3f& normal,  Material* material): Object3D(material){
        this->center = center;
        this->edge_length = edge_length;
        this->normal = normal.normalized(); // 确保法向量是单位向量

    }

    ~Square() override = default;

	ObjectType getType() const override {
		return SQUARE;
	}

    Vector3f getCenter() const {
        return center;
    }
    float getEdgeLength() const {
        return edge_length;
    }


    Vector3f getNormal() const {
        return normal;
    }

    float getArea() const override{
        return edge_length * edge_length;
    }

    bool intersect(const Ray &ray, Hit &hit, float tmin) override {
        Vector3f ray_o = ray.getOrigin();
        Vector3f ray_d = ray.getDirection();

        float denom = Vector3f::dot(ray_d, normal);
        if (fabs(denom) < 1e-6) return false; // 平行，不相交

        float t = Vector3f::dot(center - ray_o, normal) / denom;
        if (t < tmin || t > hit.getT()) return false;

        Vector3f p = ray.pointAtParameter(t); // 交点坐标

        // 构造方形坐标系（两个边方向）
        Vector3f u = Vector3f::cross(normal, Vector3f(1, 0, 0));
        if (u.squaredLength() < 1e-6)
            u = Vector3f::cross(normal, Vector3f(0, 1, 0));
        u.normalize();
        Vector3f v = Vector3f::cross(normal, u).normalized();

        // 投影到 u-v 平面坐标系
        Vector3f local = p - center;
        float u_dist = Vector3f::dot(local, u);
        float v_dist = Vector3f::dot(local, v);

        float half = edge_length / 2.0f;
        if (fabs(u_dist) > half || fabs(v_dist) > half) return false; // 落在正方形之外

        // 命中，更新 hit
        hit.set(t, this->material, normal);
        //printf("hit !!!!"); 可以命中
        return true;
    }


    Vector3f getRandomPoint() const override {
        // 构造正方形的局部坐标系
        Vector3f u = Vector3f::cross(normal, Vector3f(1, 0, 0));
        if (u.squaredLength() < 1e-6)
            u = Vector3f::cross(normal, Vector3f(0, 1, 0));
        u.normalize();
        Vector3f v = Vector3f::cross(normal, u).normalized();

        float half = edge_length / 2.0f;
        float u_off = ((float)rand() / RAND_MAX) * edge_length - half;
        float v_off = ((float)rand() / RAND_MAX) * edge_length - half;

        return center + u * u_off + v * v_off;
    }

protected:
    Vector3f center;
    float edge_length;
    Vector3f normal; 

};

#endif //TRIANGLE_H
