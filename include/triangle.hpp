#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;
// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0]=a;
		vertices[1]=b;
		vertices[2]=c;
		normal=Vector3f::cross(b-a,c-a).normalized();
	}

	virtual ObjectType getType() const override {
		return TRIANGLE;
	}
	bool intersect( const Ray& ray,  Hit& hit , float tmin,float time) override {
		
		Vector3f ray_o=ray.getOrigin();
		Vector3f ray_d=ray.getDirection();
		Vector3f edge1=vertices[1]-vertices[0];
		Vector3f edge2=vertices[2]-vertices[0];

		Vector3f p=Vector3f::cross(ray_d,edge2);
		float det=Vector3f::dot(edge1,p);
		if(fabs(det)<1e-6) return false;
		
		float invdet=1.0f/det;
		Vector3f tvec=ray_o-vertices[0];

		float alpha=Vector3f::dot(tvec,p)*invdet;
		if(alpha<0.0f||alpha>1.0f) return false;

		Vector3f q=Vector3f::cross(tvec,edge1);
		float beta=Vector3f::dot(ray_d,q)*invdet;
		if(beta<0.0f||beta+alpha>1.0f) return false;

		float t=Vector3f::dot(edge2,q)*invdet;

		// 判断交点是否在三角形内部
		if (t>tmin&&t<hit.getT()) {
			hit.set(t, material, normal);
			return true;
		}
	
        return false;
	}

	Vector3f normal;
	Vector3f vertices[3];
protected:
};

#endif //TRIANGLE_H
