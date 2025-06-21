#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() {
        normal=Vector3f(0,1,0);
        d=0;

    }

    Plane(const Vector3f &normal, float d, Material *m) : Object3D(m) {
        this->normal=normal.normalized();
        this->d=d;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin,float time) override {
        Vector3f ray_o=r.getOrigin();
        Vector3f ray_d=r.getDirection();
        float n=Vector3f::dot(normal,ray_d);
        if(fabs(n)<1e-6) return false;
        float t=(d-Vector3f::dot(normal,ray_o))/n;
        if(t>tmin&&t<h.getT()){
            h.set(t,material,normal);
            return true;
        }
        return false;
    }

    virtual ObjectType getType() const override {
        return PLANE;
    }
protected:
    Vector3f normal;
    float d;

};

#endif //PLANE_H
		

