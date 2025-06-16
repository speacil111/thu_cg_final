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
#define EPISILON 0.001f




class Tracer{
    private:
        RayFunctions rf;
    public:
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
            
            if (hit.getMaterial()->getType() == DIFF) { // 漫反射 smallpt
                //shadowray判断
                Vector3f finalcolor=Vector3f::ZERO; // 初始化总光照
                for(int li=0;li<sp.getNumLights();li++){
                    Light* light=sp.getLight(li);
                    Vector3f L,LightColor;
                    light->getIllumination(hit_p, L, LightColor);
                    finalcolor += hit.getMaterial()->Shade(ray, hit, L, LightColor); // 累加光照   
                    Ray shadow_ray(hit_p, L.normalized());
                    Hit shadow_hit;
                    if(!sp.getGroup()->intersect(shadow_ray, shadow_hit, EPISILON)) {
                        // 如果没有交点，说明光线没有被遮挡
                        finalcolor += hit.getMaterial()->Shade(shadow_ray, hit, L, LightColor);
                    }
                }
                return finalcolor;
            }
            //折射与反射部分参考了smallpt代码

            else if(hit.getMaterial()->getType() == SPEC){ //镜面反射
            //设置光线新方向
                Ray reflect_ray(hit_p , rf.reflect_d(hit_d, hit_n)); // 计算反射方向    
                return c*Raytrace(reflect_ray, sp, depth + 1); //递归调用
            }
            else{
                float refra_ratio= hit.getMaterial()->getRefractive(); // 折射率
                Vector3f refract_dir = rf.refract_d(hit_d,hit_n,refra_ratio); // 计算折射方向
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
            Refl_T type = hit.getMaterial()->getType();
            if (type == DIFF) {
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
            else if(type == SPEC) {
                // 镜面反射
                Ray reflect_ray(hit_p, rf.reflect_d(hit_d, hit_n));
                return hit_e + c * Pathtrace(reflect_ray, sp, depth);
            }
            else if(type == RFRE){
                // 折射
                float refra_ratio = hit.getMaterial()->getRefractive();
                Vector3f refract_dir = rf.refract_d(hit_d, hit_n, refra_ratio);
                Ray refract_ray(hit_p, refract_dir);
                return hit_e + c * Pathtrace(refract_ray, sp, depth);
            }
            // else if(type == GLOS){
            //     // 光泽材质
            //     Vector3f nl = Vector3f::dot(hit_n, hit_d) < 0 ? hit_n : -hit_n;
            //     Vector3f d = rf.glossy_d(hit_d, nl);
            //     Ray new_ray(hit_p, d);
            //     return hit_e + c * Pathtrace(new_ray, sp, depth);
            // }
            return Vector3f::ZERO; // 默认返回零向量
        }

};