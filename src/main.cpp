#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"

#include <string>

#define PI 3.1415926
//定义避免自相交的常亮0.001f
#define EPSILON 0.001f

using namespace std;

enum Refl_T{ DIFF,SPEC,RFRE };

inline double clamp(double x) {
    return x < 0 ? 0 : (x > 1 ? 1 : x);
}

inline int toInt(double x){ return int(pow(clamp(x),1/2.2)*255+.5); }

Vector3f Raytrace(const Ray &ray,SceneParser&sp,int depth){ //光线追踪
    Hit hit;
    bool isIntersect = sp.getGroup()->intersect(ray, hit, 0.0f);
    if(!isIntersect) {
        return sp.getBackgroundColor(); //如果没有交点，返回背景色
    }
    Vector3f hit_p= ray.pointAtParameter(hit.getT());//碰撞点
    Vector3f hit_n=hit.getNormal().normalized(); // 碰撞点法线
    Vector3f hit_d=ray.getDirection().normalized();// 射线方向
    Vector3f hit_e= hit.getMaterial()->getemission(); // 碰撞点发光系数
    Vector3f c = hit.getMaterial()->getcolor();// 碰撞点颜色 color

    double p=std::max({c.x(), c.y(), c.z()});

    if (++depth > 5) { 
        if ((double)rand() / RAND_MAX < p) { // 俄罗斯轮盘算法
            c = c * (1 / p); // 调整材质颜色
        } else {
            return hit_e; // 返回发光系数
        }
    }
    Refl_T hit_type= hit.getMaterial()->getType();// 碰撞点材质类型
    
    if (hit_type == DIFF) { // 漫反射 smallpt
        //shadowray判断
        Vector3f finalcolor=Vector3f::ZERO; // 初始化总光照
        for(int li=0;li<sp.getNumLights();li++){
            Light* light=sp.getLight(li);
            Vector3f L,LightColor;
            light->getIllumination(hit_p, L, LightColor);
            Ray shadow_ray(hit_p, L.normalized());
            Hit shadow_hit;
            if(!sp.getGroup()->intersect(shadow_ray, shadow_hit, EPSILON)) {
                // 如果没有交点，说明光线没有被遮挡
                finalcolor += hit.getMaterial()->Shade(shadow_ray, hit, L, LightColor);
            }
        }
        return finalcolor;
    }
    //折射与反射部分参考了smallpt代码

    else if(hit_type==SPEC){ //镜面反射
    //设置光线新方向
        Ray reflect_ray(hit_p , hit.getMaterial()->reflect_d(hit_d, hit_n));
        return c*Raytrace(reflect_ray, sp, depth + 1); //递归调用
    }
    else{
        float refra_ratio= hit.getMaterial()->getRefractive(); // 折射率
        Vector3f refract_dir = hit.getMaterial()->refract_d(hit_d, hit_n, refra_ratio); // 计算折射方向
        Ray refract_ray(hit_p , refract_dir);
        return c*Raytrace(refract_ray, sp, depth + 1); //递归调用
    }

}





Vector3f Pathtrace(const Ray &ray, SceneParser& sp, int depth) { //路径追踪
    Hit hit;
    bool isIntersect = sp.getGroup()->intersect(ray, hit, 0.0f);
    if(!isIntersect) {
        return sp.getBackgroundColor();
    }

    Vector3f hit_p = ray.pointAtParameter(hit.getT());
    Vector3f hit_n = hit.getNormal().normalized();
    Vector3f hit_d = ray.getDirection().normalized();
    Vector3f hit_e = hit.getMaterial()->getemission();
    Vector3f c = hit.getMaterial()->getcolor();
    Refl_T hit_type = hit.getMaterial()->getType();

    // 如果击中光源，直接返回发光强度
    if (hit_e.length() > 0) {
        return hit_e;
    }

    // 俄罗斯轮盘赌策略
    double p = std::max({c.x(), c.y(), c.z()});
    if (++depth > 5) {
        if ((double)rand() / RAND_MAX < p) {
            c = c * (1 / p);
        } else {
            return Vector3f::ZERO;
        }
    }

    if (hit_type == DIFF) {
        // 确保法线方向正确（朝向入射光线）
        Vector3f nl = Vector3f::dot(hit_n, hit_d) < 0 ? hit_n : -hit_n;
        
        double r1 = 2 * PI * ((double)rand() / RAND_MAX);
        double r2 = (double)rand() / RAND_MAX;
        double r2s = sqrt(r2);

        Vector3f w = nl;
        Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
        Vector3f v = Vector3f::cross(w, u);

        Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
        Ray new_ray(hit_p, d);

        // 递归计算radiance
        return hit_e + c * Pathtrace(new_ray, sp, depth);
    }
    else if(hit_type == SPEC) {
        // 镜面反射
        Ray reflect_ray(hit_p, hit.getMaterial()->reflect_d(hit_d, hit_n));
        return hit_e + c * Pathtrace(reflect_ray, sp, depth);
    }
    else {
        // 折射
        float refra_ratio = hit.getMaterial()->getRefractive();
        Vector3f refract_dir = hit.getMaterial()->refract_d(hit_d, hit_n, refra_ratio);
        Ray refract_ray(hit_p, refract_dir);
        return hit_e + c * Pathtrace(refract_ray, sp, depth);
    }
}

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.

    // TODO: Main RayCasting Logic
    // First, parse the scene using SceneParser
    SceneParser sp(inputFile.c_str());
    Camera *camera = sp.getCamera();
    Group *group = sp.getGroup();
    Image I(camera->getWidth(), camera->getHeight());
    // Then loop over each pixel in the image, shooting a ray

    // for(int x=0;x<camera->getWidth();x++){
    //     for(int y=0;y<camera->getHeight();y++){
    //         Ray camRay = sp.getCamera()->generateRay(Vector2f(x, y)) ;
    //         Vector3f color = Raytrace(camRay, sp, 0);
    //         // 对颜色进行钳制
    //         color = Vector3f(clamp(color.x()), clamp(color.y()), clamp(color.z()));
    //         I.SetPixel(x,y,color);
    //     }
    // }

    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.

    //Path trace 版本 参考了small_pt
    int samples=4;
    for(int x = 0; x < camera->getWidth(); x++) {
        for(int y = 0; y < camera->getHeight(); y++) {
            Vector3f color = Vector3f::ZERO;
            for (int sy = 0; sy < 2; sy++) {
                for (int sx = 0; sx < 2; sx++) {
                    Vector3f subColor = Vector3f::ZERO;
                    for (int s = 0; s < samples / 4; s++) {
                        float r1 = 2 * ((float)rand() / RAND_MAX);
                        float dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                        float r2 = 2 * ((float)rand() / RAND_MAX);
                        float dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                        float u = x + (sx + 0.5f + dx) / 2.0f;
                        float v = y + (sy + 0.5f + dy) / 2.0f;
                        Vector2f pixelPos(u, v);
                        Ray camRay = camera->generateRay(pixelPos);
                        subColor += Pathtrace(camRay, sp, 0);
                    }
                    color += subColor * 0.25f / (samples / 4); // 0.25 for 2x2 subpixel average
                }
            }

        color = Vector3f(
            clamp( color.x()),
            clamp( color.y()),
            clamp( color.z())
        );
    }
    I.SaveImage(argv[2]);
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

