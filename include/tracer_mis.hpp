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
#define NEE 1 //0 关闭NEE采样，1开启NEE采样
#define MIS 0 //是否采用MIS混合采样
inline double max_c(double a,double b,double c){
    return a > b ? (a > c ? a : c) : (b > c ? b : c);
}



class TracerMIS{
    private:
        RayFunctions rf;
    public:
        // MIS路径追踪实现
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

                Vector3f Le_direct = Vector3f::ZERO;
                if (NEE) {
                    int num_objs = sp.getGroup()->getGroupSize();
                    for (int i = 0; i < num_objs; i++) {
                        Object3D* light_obj = sp.getGroup()->getObject(i);
                        Vector3f emis = light_obj->getMaterial()->getemission();
                        if (emis == Vector3f::ZERO) continue;

                        double Area = light_obj->getArea();
                        for (int s = 0; s < NEE_SP; s++) {
                            Vector3f rn_p = light_obj->getRandomPoint();  // 光源采样点
                            Vector3f l_dir = rn_p - hit_p;
                            double dist2 = l_dir.squaredLength();
                            Vector3f l_dir_n = l_dir.normalized();

                            // shadow ray
                            Ray shadow_ray(hit_p + l_dir_n * EPSILON, l_dir_n);
                            Hit shadow_hit;
                            bool occluded = sp.getGroup()->intersect(shadow_ray, shadow_hit, EPSILON);
                            if (shadow_hit.getT() >= l_dir.length() - EPSILON || !occluded) {
                                Vector3f cos_n = shadow_hit.getNormal().normalized();
                                double cos_theta = fabs(Vector3f::dot(nl, l_dir_n));
                                double cos_theta_p = fabs(Vector3f::dot(cos_n, -l_dir_n));

                                double pdf_light = dist2 / (cos_theta_p * Area + 1e-6);  // 光源采样 pdf

                                Vector3f fr = c / M_PI; // 漫反射 BRDF
                                Vector3f contrib = emis * fr * cos_theta / pdf_light;

                                // 也可以加权计算 BSDF 的 PDF（假设用 cosine-weighted）
                                double pdf_bsdf = cos_theta / M_PI;

                                // Power Heuristic 权重
                                double w_nee = (pdf_light * pdf_light) / (pdf_light * pdf_light + pdf_bsdf * pdf_bsdf + 1e-6);

                                Le_direct += contrib * w_nee;
                            }
                        }
                    }
                    Le_direct = Le_direct/NEE_SP;
                }

                // ===== Cos-weighted Sample =====
                double r1 = 2 * PI * ((double)rand() / RAND_MAX);
                double r2 = (double)rand() / RAND_MAX;
                double r2s = sqrt(r2);

                Vector3f w = nl;
                Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
                Vector3f v = Vector3f::cross(w, u);

                Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
                Ray new_ray(hit_p + EPSILON * d, d);

                // 如果从cos方向采样命中了光源，我们也考虑其直接光照部分
                Vector3f L_indirect = Pathtrace(new_ray, sp, depth + 1);
                double pdf_bsdf = Vector3f::dot(nl, d) / M_PI;
                double pdf_light = estimate_light_pdf(d, hit_p, sp); // 你需要自己写这个函数或预估它

                double w_bsdf = (pdf_bsdf * pdf_bsdf) / (pdf_bsdf * pdf_bsdf + pdf_light * pdf_light + 1e-6);
                Vector3f fr = c / M_PI;
                Vector3f Le_bsdf = L_indirect * fr * Vector3f::dot(nl, d) * w_bsdf / (pdf_bsdf + 1e-6);

                return hit_e + Le_direct + Le_bsdf * factor;
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

                Vector3f w = reflect_dir.normalized();
                Vector3f u = Vector3f::cross((fabs(w.x()) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
                Vector3f v = Vector3f::cross(w, u);

                float radius = 0.6; 
                double r1 = M_PI * ((double)rand() / RAND_MAX) / 2.0;  // zenith angle in [0, π/2]
                double r2 = 2.0 * M_PI * ((double)rand() / RAND_MAX);  // azimuth in [0, 2π]

                Vector3f l = (w * cos(r1) + u * sin(r1) * cos(r2) + v * sin(r1) * sin(r2)).normalized() * radius;
                Vector3f glossy_dir = (reflect_dir + l).normalized();  
                Ray glossy_ray(hit_p + EPSILON * glossy_dir, glossy_dir);
                float weight = fabs( Vector3f::dot(nl, glossy_dir)) * 2 * M_PI * radius * radius;

                return hit_e + c * Pathtrace(glossy_ray, sp, depth + 1) * weight;
            }

            return Vector3f::ZERO; 
        }

};