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

using namespace std;

enum Refl_T{ DIFF,SPEC,RFRE };

inline double clamp(double x) {
    return x < 0 ? 0 : (x > 1 ? 1 : x);
}

inline int toInt(double x){ return int(pow(clamp(x),1/2.2)*255+.5); }

Vector3f Pathtrace(const Ray &ray,SceneParser&sp,int depth){ //光线追踪
    Hit hit;
    bool isIntersect = sp.getGroup()->intersect(ray, hit, 0.0f);

    //smallpt
    if(!isIntersect) {
        return sp.getBackgroundColor(); //如果没有交点，返回背景色
    }
    Vector3f hit_p= ray.pointAtParameter(hit.getT());//碰撞点
    Vector3f hit_n=hit.getNormal().normalized(); // 碰撞点法线
    Vector3f hit_d=ray.getDirection().normalized();// 射线方向
    Vector3f hit_e= hit.getMaterial()->getemission(); // 碰撞点发光系数
    Vector3f c = hit.getMaterial()->getcolor();// 碰撞点颜色 color
    //颜色获取存疑

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
        Vector3f total_li=Vector3f::ZERO; // 初始化总光照
        for(int li=0;li<sp.getNumLights();li++){
            Light* light=sp.getLight(li);
            Vector3f L,LightColor;
            light->getIllumination(hit_p, L, LightColor);
            Ray shadow_ray(hit_p, L.normalized());
            Hit shadow_hit;
            if(!sp.getGroup()->intersect(shadow_ray, shadow_hit, 0.001f)) {
                // 如果没有交点，说明光线没有被遮挡
                total_li += hit.getMaterial()->Shade(shadow_ray, hit, L, LightColor);
            }
        }
        
        double r1 = 2 * PI * ((double)rand() / RAND_MAX);
        double r2 = (double)rand() / RAND_MAX;
        double r2s = sqrt(r2);

        Vector3f w = hit_n;
        Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
        Vector3f v = Vector3f::cross(u, w);

        Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
        Ray diffuse_ray(hit_p, d);


        return hit_e +total_li+ c * Raytrace(diffuse_ray, sp, depth + 1);
    }

    else if(hit_type==SPEC){ //镜面反射
    //设置光线新方向
        Ray reflect_ray(hit_p , hit.getMaterial()->reflect_d(hit_d, hit_n));
        return hit_e+c*Raytrace(reflect_ray, sp, depth + 1); //递归调用
    }
    else{
        float refra_ratio= hit.getMaterial()->getRefractive(); // 折射率
        Vector3f refract_dir = hit.getMaterial()->refract_d(hit_d, hit_n, refra_ratio); // 计算折射方向
        Ray refract_ray(hit_p , refract_dir);
        return hit_e+c*Raytrace(refract_ray, sp, depth + 1); //递归调用
    }

}



Vector3f Raytrace(const Ray &ray,SceneParser&sp,int depth){ //光线追踪
    Hit hit;
    bool isIntersect = sp.getGroup()->intersect(ray, hit, 0.0f);

    //smallpt
    if(!isIntersect) {
        return sp.getBackgroundColor(); //如果没有交点，返回背景色
    }
    Vector3f hit_p= ray.pointAtParameter(hit.getT());//碰撞点
    Vector3f hit_n=hit.getNormal().normalized(); // 碰撞点法线
    Vector3f hit_d=ray.getDirection().normalized();// 射线方向
    Vector3f hit_e= hit.getMaterial()->getemission(); // 碰撞点发光系数
    Vector3f c = hit.getMaterial()->getcolor();// 碰撞点颜色 color
    //颜色获取存疑

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
            if(!sp.getGroup()->intersect(shadow_ray, shadow_hit, 0.001f)) {
                // 如果没有交点，说明光线没有被遮挡
                finalcolor += hit.getMaterial()->Shade(shadow_ray, hit, L, LightColor);
            }
        }
        return finalcolor;
    }

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
    for(int x=0;x<camera->getWidth();x++){
        for(int y=0;y<camera->getHeight();y++){
            Ray camRay = sp.getCamera()->generateRay(Vector2f(x, y)) ;
            Vector3f color = Raytrace(camRay, sp, 0); // 递归调用Raytrace函数
            //clamp color
            color=Vector3f(clamp(color.x()), clamp(color.y()), clamp(color.z()));
            I.SetPixel(x,y,color);
        }
    }
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.
    //Path trace 版本





    I.SaveImage(argv[2]);
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

