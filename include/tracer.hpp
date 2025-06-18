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
#define EPSILON 0.001f
#define NEE_SP 5
#define MAX_DEPTH 20
#define NEE 0 //0 关闭NEE采样，1开启NEE采样
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
                return Vector3f::ZERO;
            }
            Vector3f hit_p= ray.pointAtParameter(hit.getT());//碰撞点
            Vector3f hit_n=hit.getNormal().normalized(); // 碰撞点法线
            Vector3f hit_d=ray.getDirection().normalized();// 射线方向
            Vector3f c = hit.getMaterial()->getcolor();// 碰撞点颜色 color

            double factor=0.95;//衰减系数

            Vector3f finalcolor=Vector3f::ZERO;
            Refl_T type= hit.getMaterial()->getType(); 
            if(type==DIFF){
                for(int li=0;li<sp.getNumLights();li++){
                    Light* light=sp.getLight(li);
                    Vector3f L,LightColor;
                    light->getIllumination(hit_p, L, LightColor);  
                    //阴影 shadow ray
                    Hit shadow_hit;
                    Ray shadow_ray = Ray((hit_p + L * EPSILON), L);
                    bool shadow = sp.getGroup()->intersect(shadow_ray, shadow_hit, EPSILON);

                    if (!shadow || shadow_hit.getT() >(hit_p-light->getPosition()).length()-EPSILON) {
                        // 没有阴影，计算光照贡献
                        finalcolor += hit.getMaterial()->Shade(ray, hit, L, LightColor);
                    }
                }
                return finalcolor;
            }else if (type == SPEC){
                Vector3f new_dir = rf.reflect_d(hit_d, hit_n); // 计算反射方向
                Ray reflect_ray(hit_p+new_dir*EPSILON, new_dir);
                return c*Raytrace(reflect_ray, sp, depth+1)*factor;
            }else if(type == RFRE){
                // 折射 无问题
                float refra_ratio = hit.getMaterial()->getRefractive();
                Vector3f refract_dir = rf.refract_d(hit_d, hit_n, refra_ratio);
                Ray refract_ray(hit_p+refract_dir*EPSILON, refract_dir);
                return c*Raytrace(refract_ray, sp, depth+1)*factor;
            }

            return finalcolor ;
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

            if(hit_e != Vector3f::ZERO) {
                return hit_e; // 如果是光源，直接返回其发射光
            }
            // 俄罗斯轮盘赌策略
            double factor=0.9;
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
                Vector3f L_smps = Vector3f::ZERO; 
                double nee_w=0.0;
                if(NEE){
                    nee_w=0.5;
                    int num_objs= sp.getGroup()->getGroupSize();
                // 可以进行多次采样求平均！
                    for( int i=0;i<num_objs;i++){
                        Object3D *obj= sp.getGroup()->getObject(i);
                        Vector3f emis=obj->getMaterial()->getemission();
                        if(emis == Vector3f::ZERO) continue; 
                        for(int s=0;s<NEE_SP;s++){
                            double f =0;
                            double Area= obj->getArea(); 
                            Vector3f rn_p=obj->getRandomPoint(); 
                            Vector3f l_dir = rn_p - hit_p; 
                            double light_dist = l_dir.length();
                            Vector3f l_dir_n= l_dir.normalized(); 
                            Ray shadow_ray(hit_p + l_dir_n * EPSILON, l_dir_n); 
                            Hit shadow_hit;
                            bool occlued = sp.getGroup()->intersect(shadow_ray, shadow_hit, EPSILON);
                            if(shadow_hit.getT() >= l_dir.length() - EPSILON ||!occlued) {
                                double cos_theta=Vector3f::dot(nl, l_dir_n); 
                                double cos_theta_p=Vector3f::dot(shadow_hit.getNormal().normalized(), l_dir_n); 
                                f=fabs(cos_theta * cos_theta_p) / (light_dist * light_dist); 
                            }  
                            L_smps += emis*f *Area*c;
                        }
                        L_smps =L_smps/NEE_SP;
                    // printf("L_smps: %f, Area: %f, f: %f\n", L_smps.length(), Area, f); 
                    //
                    }
                }
                //下面算法肯定没问题！
                double r1 = 2 * PI * ((double)rand() / RAND_MAX);
                double r2 = (double)rand() / RAND_MAX;
                double r2s = sqrt(r2);

                Vector3f w = nl;
                Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
                Vector3f v = Vector3f::cross(w, u);

                Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
                Ray new_ray(hit_p, d);
                //实现MIS加权采样
                return hit_e + (1-nee_w)*c * Pathtrace(new_ray, sp, depth+1)*factor+nee_w*L_smps;
            
            }
            else if(type == SPEC) {
                Ray reflect_ray(hit_p, rf.reflect_d(hit_d, hit_n));
                return hit_e + c * Pathtrace(reflect_ray, sp, depth+1)*factor;
            }
            else if(type == RFRE){
                float refra_ratio = hit.getMaterial()->getRefractive();
                Vector3f refract_dir = rf.refract_d(hit_d, hit_n, refra_ratio);
                Ray refract_ray(hit_p, refract_dir);
                return hit_e + c * Pathtrace(refract_ray, sp, depth+1)*factor;
            }
            else if (type == GLOS) {
                Vector3f nl = Vector3f::dot(hit_n, hit_d) < 0 ? hit_n : -hit_n;
                Vector3f reflect_dir = rf.reflect_d(hit_d, nl);
                
                double r1 = 2 * PI * ((double)rand() / RAND_MAX);
                double r2 = (double)rand() / RAND_MAX;
                double r2s = pow(r2, 1.0 / (hit.getMaterial()->getShininess() + 1));  // 越大越靠近镜面方向

                Vector3f w = reflect_dir;
                Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
                Vector3f v = Vector3f::cross(w, u);
                Vector3f sample_dir = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2s * r2s)).normalized();
                Ray glossy_ray(hit_p + EPSILON * sample_dir, sample_dir);

                return hit_e + c * Pathtrace(glossy_ray, sp, depth + 1) * factor;
            }

            return Vector3f::ZERO; 
        }

};