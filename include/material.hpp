#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"

#include <iostream>

enum Refl_T {
    DIFF,  
    SPEC,  
    RFRE,  
    GLOS   
}; 
// TODO: Implement Shade function that computes Phong introduced in class.
class Material {
public:

    explicit Material( float s = 10,Refl_T type = DIFF,const Vector3f &em=Vector3f::ZERO,
                        const Vector3f &color=Vector3f::ZERO,float refra=1.0f) : 
        type(type), shininess(s), emission(em),color(color) ,refractive(refra){}

    virtual ~Material() = default;

    // virtual Vector3f getDiffuseColor() const {
    //     return diffuseColor;
    // }

    virtual float getShininess() const {
        return shininess;
    }
    
    virtual Refl_T getType() const {
        return type;
    }

    virtual Vector3f getemission() const {
        // if(emission !=Vector3f::ZERO){
        //     printf("emission color: %f %f %f\n", emission.x(), emission.y(), emission.z());
        // } 会打中！
        return emission;
    }

    virtual Vector3f getcolor() const {
        return color;
    }

    virtual float getRefractive() const {
        return refractive;
    }

    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f shaded = Vector3f::ZERO;
        // 
        Vector3f N=hit.getNormal().normalized();
        Vector3f V=-ray.getDirection().normalized();
        Vector3f L=dirToLight.normalized(); 
        Vector3f R=2*Vector3f::dot(N,L)*N-L;
        diffuseColor = color;
        specularColor = Vector3f::ZERO;

        float dif_col=Vector3f::dot(N,L);
        float dif_clamp = fabs(Vector3f::dot(N, L));

        shaded+=dif_clamp*lightColor*diffuseColor;

        float spe_col=Vector3f::dot(V,R);
        float spe_clamp = fabs(Vector3f::dot(V, R));
        shaded+=lightColor*specularColor*pow(spe_clamp,shininess);

        return shaded;
    }

protected:
    Refl_T type;  
    Vector3f emission;//发光系数
    Vector3f color;
    Vector3f diffuseColor; 
    Vector3f specularColor; 
    float shininess;
    float refractive;

};


#endif // MATERIAL_H
