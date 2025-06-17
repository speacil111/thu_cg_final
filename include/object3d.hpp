#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"

enum ObjectType {
    SPHERE,
    GROUP,
    PLANE,
    TRIANGLE,
    MESH,
    TRANSFORM
};
// Base class for all 3d entities.
class Object3D {
public:
    Object3D() : material(nullptr) {}

    virtual ~Object3D() = default;

    explicit Object3D(Material *material) {
        this->material = material;
    }
    virtual ObjectType getType() const = 0; // Return the type of the object (e.g., SPHERE, GROUP, etc.)
    // Intersect Ray with this object. If hit, store information in hit structure.
    virtual bool intersect(const Ray &r, Hit &h, float tmin) = 0;
protected:
    
    Material *material;
};

#endif

