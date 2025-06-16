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
#include "ray_fn.hpp"

#include <string>
#define PI 3.1415926
#define EPSILON 0.0001f
#define MAX_DEPTH 20

inline double max_c(double a,double b,double c){
    return a > b ? (a > c ? a : c) : (b > c ? b : c);
}

class Tracer{
    private:
        RayFunctions rf;
    public:
        Vector3f Raytrace(const Ray &ray,SceneParser&sp,int depth){ //光线追踪
            if(depth>= MAX_DEPTH) { 
                return Vector3f::ZERO; 
            }

            Hit hit;
            bool isIntersect = sp.getGroup()->intersect(ray, hit, 0.0f);
            if(!isIntersect) {
                return sp.getBackgroundColor(); 
            }
            Vector3f hit_p= ray.pointAtParameter(hit.getT());//碰撞点
            Vector3f hit_n=hit.getNormal().normalized(); // 碰撞点法线
            Vector3f hit_d=ray.getDirection().normalized();// 射线方向
            Vector3f c = hit.getMaterial()->getcolor();// 碰撞点颜色 color

            double p=max_c(c.x(), c.y(), c.z()); 

            double factor=1.0; 
            if (depth > 5) {
                double rn= (double)rand() / RAND_MAX;
                //printf("depth: %d, p: %f, rn: %f\n", depth, p, rn);
                if (rn < p) {
                    factor /=p;
                } else {
                    return Vector3f::ZERO; 
                }
            }
            Vector3f finalcolor=Vector3f::ZERO; // 初始化总光照
            Refl_T type= hit.getMaterial()->getType(); // 获取材质类型
            if(type == DIFF) {
                for(int li=0;li<sp.getNumLights();li++){
                    Light* light=sp.getLight(li);
                    Vector3f L,LightColor;
                    light->getIllumination(hit_p, L, LightColor);  
                    //阴影 shadow ray
                    Ray shadow_ray(hit_p+L*EPSILON, L.normalized());
                    Hit shadow_hit;
                    if(!sp.getGroup()->intersect(shadow_ray, shadow_hit, EPSILON)) {
                        finalcolor += hit.getMaterial()->Shade(ray, hit, L, LightColor);
                    }
                }
                return finalcolor;
            }
            //折射与反射部分参考了smallpt代码
            else if(type == SPEC) {
                // 镜面反射
                Vector3f new_dir = rf.reflect_d(hit_d, hit_n); // 计算反射方向
                Ray reflect_ray(hit_p+new_dir*EPSILON, new_dir);
                return c * Raytrace(reflect_ray, sp, depth+1)*factor;
            }
            else if(type == RFRE){
                // 折射
                float refra_ratio = hit.getMaterial()->getRefractive();
                Vector3f refract_dir = rf.refract_d(hit_d, hit_n, refra_ratio);
                Ray refract_ray(hit_p+refract_dir*EPSILON, refract_dir);
                return c * Raytrace(refract_ray, sp, depth+1)*factor;
            }
            return Vector3f::ZERO; 
        }







        // 路径追踪实现
        Vector3f Pathtrace(const Ray &ray, SceneParser& sp, int depth) { //路径追踪
            if (depth >= MAX_DEPTH) { // 限制递归深度
                return Vector3f::ZERO; // 返回零向量
            }
        
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
            // 俄罗斯轮盘赌策略
            double factor=1.0;
            double p = max_c(c.x(), c.y(), c.z()); 
            //printf("p: %f\n", p); // 调试输出

            if (depth > 5) {
                double rn= (double)rand() / RAND_MAX;
                //printf("depth: %d, p: %f, rn: %f\n", depth, p, rn);
                if (rn < p) {
                    factor /=p;
                } else {
                    return Vector3f::ZERO; 
                }
            }
            Refl_T type = hit.getMaterial()->getType();
            if (type == DIFF) {

                Vector3f nl = Vector3f::dot(hit_n, hit_d) < 0 ? hit_n : -hit_n;

                //实现NEE采样
                Vector3f L_smps = Vector3f::ZERO; // 初始化采样光照

                for (int li = 0; li < sp.getNumLights(); li++) {
                    Light* light = sp.getLight(li);
                    Vector3f L, LightColor;
                    light->getIllumination(hit_p, L, LightColor);
                    Vector3f L_dir=L.normalized();
                    Ray shadowRay(hit_p + L_dir * EPSILON, L_dir);
                    Hit shadowHit;
                    if (!sp.getGroup()->intersect(shadowRay, shadowHit, EPSILON) ||
                        shadowHit.getT() > L.length() - EPSILON) {

                        float cos_theta = Vector3f::dot(nl, L_dir);
                        if (cos_theta > 0) {
                            L_smps += LightColor * c * cos_theta;
                        }
                    }
                }

            
                
                double r1 = 2 * PI * ((double)rand() / RAND_MAX);
                double r2 = (double)rand() / RAND_MAX;
                double r2s = sqrt(r2);

                Vector3f w = nl;
                Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
                Vector3f v = Vector3f::cross(w, u);

                Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
                Ray new_ray(hit_p, d);

                // NEE 和 直接光照加权
                return hit_e + 0.5*c * Pathtrace(new_ray, sp, depth+1)*factor+0.5*L_smps;
            }
            else if(type == SPEC) {
                // 镜面反射
                Ray reflect_ray(hit_p, rf.reflect_d(hit_d, hit_n));
                return hit_e + c * Pathtrace(reflect_ray, sp, depth+1)*factor;
            }
            else if(type == RFRE){
                // 折射
                float refra_ratio = hit.getMaterial()->getRefractive();
                Vector3f refract_dir = rf.refract_d(hit_d, hit_n, refra_ratio);
                Ray refract_ray(hit_p, refract_dir);
                return hit_e + c * Pathtrace(refract_ray, sp, depth+1)*factor;
            }
            else if (type == GLOS) {
                Vector3f nl = Vector3f::dot(hit_n, hit_d) < 0 ? hit_n : -hit_n;

                // 构造一个接近反射方向的分布，用于 glossy 散射
                Vector3f reflect_dir = rf.reflect_d(hit_d, nl);
                
                // 采样一个在镜面附近偏移的小方向（Phong Lobe）
                double r1 = 2 * PI * ((double)rand() / RAND_MAX);
                double r2 = (double)rand() / RAND_MAX;
                double r2s = pow(r2, 1.0 / (hit.getMaterial()->getShininess() + 1));  // 越大越靠近镜面方向

                // 构造局部坐标系
                Vector3f w = reflect_dir;
                Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
                Vector3f v = Vector3f::cross(w, u);

                // 构造微扰后的方向（around reflection direction）
                Vector3f sample_dir = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2s * r2s)).normalized();
                Ray glossy_ray(hit_p + EPSILON * sample_dir, sample_dir);

                return hit_e + c * Pathtrace(glossy_ray, sp, depth + 1) * factor;
            }

            return Vector3f::ZERO; // 默认返回零向量
        }

};