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
#define NEE_SP 1
#define MAX_DEPTH 20
#define NEE 0 //0 关闭NEE采样，1开启NEE采样
#define FNIR 0 //0 关闭菲涅尔反射，1开启菲涅尔反射
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
            double factor=0.9;//衰减系数
            Vector3f finalcolor=Vector3f::ZERO;
            Refl_T type= hit.getMaterial()->getType(); 
            for(int li=0;li<sp.getNumLights();li++){
                Light* light=sp.getLight(li);
                Vector3f L,LightColor;
                light->getIllumination(hit_p, L, LightColor);  
                if(type==DIFF){
                    Hit shadow_hit;
                    Ray shadow_ray = Ray((hit_p + L * EPSILON), L);
                    bool shadow = sp.getGroup()->intersect(shadow_ray, shadow_hit, EPSILON);
                    if (!shadow || shadow_hit.getT() >((hit_p-light->getPosition()).length()-EPSILON)) {
                        finalcolor += hit.getMaterial()->Shade(ray, hit, L, LightColor);
                    }
                    return finalcolor;
                }else if (type == SPEC){
                    Vector3f reflect_dir = rf.reflect_d(hit_d, hit_n); // 计算反射方向
                    Ray reflect_ray(hit_p+reflect_dir*EPSILON,reflect_dir);
                    return c*Raytrace(reflect_ray, sp, depth+1)*factor;
                }else if(type == RFRE){
                    float eta = hit.getMaterial()->getRefractive();
                    Vector3f refract_dir = rf.refract_d(hit_d, hit_n, eta);
                    Ray refract_ray(hit_p+refract_dir*EPSILON, refract_dir);
                    return c*Raytrace(refract_ray, sp, depth+1)*factor;
                }
            }
            printf("Unknown material type encountered in Raytrace.\n");
            return finalcolor ;
        }







        // 路径追踪实现
        Vector3f Pathtrace(const Ray &ray, SceneParser& sp, int depth, unsigned int &seed) { //路径追踪
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
            double factor=0.9;
            double p = max_c(c.x(), c.y(), c.z()); 
            //printf("p: %f\n", p); // 调试输出

            if (depth > 5) {
                double rn= (double)rand_r(&seed) / RAND_MAX;
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
                double nee_w = 0.0;
                if(NEE){
                    int num_objs= sp.getGroup()->getGroupSize();
                // 可以进行多次采样求平均！
                    for( int i=0;i<num_objs;i++){
                        Object3D *obj= sp.getGroup()->getObject(i);
                        Vector3f emis=obj->getMaterial()->getemission();
                        if(emis == Vector3f::ZERO) continue; 
                        for(int s=0;s<NEE_SP;s++){
                            double f =0;
                            double Area= obj->getArea(); 
                            Vector3f rn_p=obj->getRandomPoint(seed); 
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
                //cos-weight
                double r1 = 2 * PI * ((double)rand_r(&seed) / RAND_MAX);
                double r2 = (double)rand_r(&seed) / RAND_MAX;
                double r2s = sqrt(r2);

                Vector3f w = nl;
                Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
                Vector3f v = Vector3f::cross(w, u);

                Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
                Ray new_ray(hit_p+d*EPSILON, d);

                return hit_e + c * Pathtrace(new_ray, sp, depth+1,seed) * factor + L_smps; 
            }
            else if(type == SPEC) {
                Vector3f reflect_dir = rf.reflect_d(hit_d, hit_n);
                Ray reflect_ray(hit_p+reflect_dir*EPSILON, reflect_dir);
                return hit_e + c * Pathtrace(reflect_ray, sp, depth+1,seed)*factor;
            }
            else if (type == RFRE) {
                float eta = hit.getMaterial()->getRefractive();
                if(!FNIR){
                    Vector3f refrac_dir= rf.refract_d(hit_d, hit_n,eta);
                    Ray refract_ray(hit_p + refrac_dir * EPSILON, refrac_dir);
                    return hit_e + c * Pathtrace(refract_ray, sp, depth + 1, seed) * factor;
                }
                Vector3f n = hit_n;
                Vector3f reflect_dir = rf.reflect_d(hit_d, hit_n).normalized();
                float cos_theta = -Vector3f::dot(hit_d, hit_n);
                if (cos_theta >= 0) {
                    eta = 1.0f / eta; // 如果入射角在法线内侧，交换折射率
                }
                else{
                    cos_theta = -cos_theta; // 入射角的余弦取正
                    n = -n; 
                }
                float sin2_t = pow(eta,2.0f) * (1.0f - pow(cos_theta,2.0f));
                if (sin2_t > 1.0f) {
                    // 全反射
                    Ray reflect_ray(hit_p+reflect_dir*EPSILON, reflect_dir);
                    return hit_e + c * Pathtrace(reflect_ray, sp, depth + 1, seed) * factor;
                }

                float cos_t = sqrt(1.0f - sin2_t);
                Vector3f refract_dir = (hit_d * eta + n * (eta * cos_theta - cos_t)).normalized();

                float R0 = pow((1.0f - eta) / (1.0f + eta), 2.0f);
                float Fr = R0 + (1 - R0) * pow(1 - cos_theta, 5.0f);
                float rn = (float)(rand_r(&seed)) / RAND_MAX;
                if (rn < Fr) {
                    Ray reflect_ray(hit_p+reflect_dir*EPSILON, reflect_dir);
                    return hit_e + c * Pathtrace(reflect_ray, sp, depth + 1, seed) * factor;
                } else {
                    Ray refract_ray(hit_p+refract_dir*EPSILON, refract_dir);
                    return hit_e + c * Pathtrace(refract_ray, sp, depth + 1, seed) * factor;
                }
            
            }
            else if (type == GLOS) {
                Vector3f nl = Vector3f::dot(hit_n, hit_d) < 0 ? hit_n : -hit_n;
                Vector3f reflect_dir = rf.reflect_d(hit_d, nl);

                Vector3f w = reflect_dir.normalized();
                Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
                Vector3f v = Vector3f::cross(w, u);

                float radius = 0.6; 
                double r1 = M_PI * ((double)rand_r(&seed) / RAND_MAX) / 2.0;  // zenith angle in [0, π/2]
                double r2 = 2.0 * M_PI * ((double)rand_r(&seed) / RAND_MAX);  // azimuth in [0, 2π]

                Vector3f l = (w * cos(r1) + u * sin(r1) * cos(r2) + v * sin(r1) * sin(r2)).normalized() * radius;
                Vector3f glossy_dir = (reflect_dir + l).normalized();  
                Ray glossy_ray(hit_p + EPSILON * glossy_dir, glossy_dir);
                float weight = fabs( Vector3f::dot(nl, glossy_dir)) * 2 * M_PI * radius * radius;

                return hit_e + c * Pathtrace(glossy_ray, sp, depth + 1,seed) * weight;
            }

            return Vector3f::ZERO; 
        }

};