#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>

// TODO: Implement Shade function that computes Phong introduced in class.

enum Refl_T{ DIFF,SPEC,RFRE };

class Material {
public:

    explicit Material(const Vector3f &obj_color, Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0,
                      const Refl_T &t, float r = 1.0f, Vector3f e = Vector3f::ZERO) :
            color(obj_color),diffuseColor(d_color), specularColor(s_color), shininess(s),
            type(t), refractive(r),emission(e) {

    }

    virtual ~Material() = default;

    virtual Vector3f getcolor() const {
        return color;
    }
    
    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    virtual Refl_T getType() const {
        return type;
    }

    virtual Vector3f getemission()const {
        return emission;
    }
    virtual float getRefractive() const {
        return refractive;
    }


    //PA1 shaded :仅用于漫反射
    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f shaded = Vector3f::ZERO;
        // 
        Vector3f N=hit.getNormal().normalized();
        Vector3f V=-ray.getDirection().normalized();
        Vector3f L=dirToLight.normalized(); 
        Vector3f R=2*Vector3f::dot(N,L)*N-L;

        float dif_col=Vector3f::dot(N,L);
        float dif_clamp = std::max(Vector3f::dot(N, L), 0.0f);

        shaded+=dif_clamp*lightColor*diffuseColor;

        float spe_col=Vector3f::dot(V,R);
        float spe_clamp = std::max(Vector3f::dot(V, R), 0.0f);
        shaded+=lightColor*specularColor*pow(spe_clamp,shininess);
        
        return shaded;
    }

    // 计算反射，折射方向
    Vector3f r_diffuse(const Vector3f &dir, const Vector3f &normal) {
        return dir - 2 * Vector3f::dot(dir, normal) * normal;
    }

    Vector3f reflect_d(const Vector3f &dir, const Vector3f &normal) {
        return dir - 2 * Vector3f::dot(dir, normal) * normal;
    }

    Vector3f refract_d(const Vector3f &dir, const Vector3f &normal, float refractive) {
        float cos=Vector3f::dot(dir,normal);
        float sin2=1-cos*cos;
        float sin2_t=sin2/refractive/refractive;
        if(sin2_t>1){ //退化成全反射
            return reflect_d(dir,normal);
        }
        float cos_t=sqrt(1-sin2_t);
        return dir*refractive-normal*(cos-cos_t);
    }

protected:
    Refl_T type; //材质类型
    Vector3f diffuseColor;
    Vector3f specularColor;
    Vector3f emission; //自发光
    Vector3f color;
    float shininess;
    float refractive; //折射率
    float Intensity;// 光强度

};


#endif // MATERIAL_H
