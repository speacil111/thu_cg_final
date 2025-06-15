#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>

// TODO: Implement Shade function that computes Phong introduced in class.
class Material {
public:

    explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0) :
            diffuseColor(d_color), specularColor(s_color), shininess(s) {

    }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }


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

protected:
    Vector3f diffuseColor;
    Vector3f specularColor;
    float shininess;
};


#endif // MATERIAL_H
