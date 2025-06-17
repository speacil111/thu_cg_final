#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D {
public:

    Sphere() {
        // unit ball at the center
        C=Vector3f(0);
        R=1;
        velocity = Vector3f(0, 0, 0);
    }



    Sphere(const Vector3f &center, float radius, const Vector3f &v,Material *material) : Object3D(material) {
        // 
        C=center;
        R=radius;
        velocity=v;
    }

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        Vector3f ray_o = r.getOrigin();
        Vector3f ray_d = r.getDirection().normalized();
        Vector3f o_c = ray_o - C;
        float a = 1.0f;
        float b =2.0f*Vector3f::dot(ray_d,o_c);
        float c=o_c.squaredLength()-R*R;
        float delta=b*b-4*a*c;
        if(delta<0) return false;
        else{
            float sqrt_delta=sqrt(delta);
            float t1=(-b-sqrt_delta)/(2.0f*a);
            float t2=(-b+sqrt_delta)/(2.0f*a);
            float t=t1;
            if(t>tmin&&t<h.getT()){
                Vector3f p=ray_o+t*ray_d;
                if(o_c.squaredLength()>R*R){
                    Vector3f n=(p-C).normalized();
                    h.set(t,material,n);
                    return true;
                }
                Vector3f n=(C-p).normalized();
                h.set(t,material,n);
                return true;
            }
            t=t2;
            if(t>tmin&&t<h.getT()){
                Vector3f p=ray_o+t*ray_d;
                if(o_c.squaredLength()>R*R){
                    Vector3f n=(p-C).normalized();
                    h.set(t,material,n);
                    return true;
                }
                Vector3f n=(p-C).normalized();
                h.set(t,material,n);
                return true;
            }
        }
        //
        return false;
    }


    void update_center(float t) {
        C += velocity * t;
    }

    ObjectType getType() const override {
        return SPHERE;
    }

protected:
    Vector3f C;
    float R;
    Vector3f velocity;
};


#endif
