#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>

class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
    virtual Ray generateRay_Dof(const Vector2f &point,double len_rad,double foc_dis) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle) : Camera(center, direction, up, imgW, imgH) {
        // angle is in radian.
        ang=angle;
    }

    Ray generateRay(const Vector2f &point) override {
        float fx=(width/2.0f)/tan(ang/2.0f);
        float fy = (height / 2.0f) / tan(ang / 2.0f);
        float cx = width / 2.0f;
        float cy = height / 2.0f;
        Vector3f dr((point.x() - cx) / fx, (cy-point.y()) / fy, 1);
        Vector3f dRc=dr.normalized();
        Vector3f dRw=Matrix3f(horizontal,-up,direction)*dRc;
        return Ray(center,dRw.normalized());
    }

    //景深效果模拟
    Ray generateRay_Dof(const Vector2f &point,double len_rad,double foc_dis) override{
        float fx=(width/2.0f)/tan(ang/2.0f);
        float fy = (height / 2.0f) / tan(ang / 2.0f);
        float cx = width / 2.0f;
        float cy = height / 2.0f;
        Vector3f dr((point.x() - cx) / fx, (cy-point.y()) / fy, 1);
        Vector3f dRc=dr.normalized();
        Vector3f dRw=Matrix3f(horizontal,-up,direction)*dRc;

        Vector3f foc_p=center+dRw *foc_dis;

        double rx= (double)rand() / RAND_MAX;
        double ry= (double)rand() / RAND_MAX;
        double theta= 2* 3.14159 *rx;
        double r=len_rad *sqrt(ry);

        Vector3f offset = r * cos(theta) * horizontal + r * sin(theta) * up;
        Vector3f new_ori = center + offset;
        Vector3f new_dir=(foc_p-new_ori).normalized();

        return Ray(new_ori, new_dir);
    }
protected:
    float ang;
};

#endif //CAMERA_H
