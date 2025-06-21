// code内所有代码文件，未标注部分均为独立实现！

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
#include "tracer.hpp"
#include <omp.h>
#include <ctime>
#include <chrono>
#include <string>

#define SAMPLES 250 //采样数
#define M_FRAME 15 //快门持续帧数 运动模糊
#define RT 0 // 0 for path tracing, 1 for ray tracing
#define FXAA 0 //FXAA抗锯齿
#define DOF 0 //景深
#define M_B 0 //运动模糊

using namespace std;
//gamma校正
inline double clamp(double x ) {
    return x < 0 ? 0 : (x > 1 ? 1 : x);
}
inline double Gamma(double x){ return pow(clamp(x),1.0/2.2f); }


float five_max(float a, float b, float c, float d, float e) {
    float max = a;
    if (b > max) max = b;
    if (c > max) max = c;
    if (d > max) max = d;
    if (e > max) max = e;
    return max;
}

float five_min(float a, float b, float c, float d, float e) {
    float min = a;
    if (b < min) min = b;
    if (c < min) min = c;
    if (d < min) min = d;
    if (e < min) min = e;
    return min;
}
// FXAA实现,部分参考https://zhuanlan.zhihu.com/p/431384101等文章
float Pixel2Lum(const Vector3f &pixel) {
    return 0.213f* pixel.x() + 0.715f* pixel.y() +0.072* pixel.z();
}

void apply_fxaa(Image &Img) {
    int w = Img.getWidth();
    int h = Img.getHeight();
    Image copy = Image(w, h);
    float contrast = 0.02f;                  
    float minThreshold = 1.0f / 24.0f;       
    float maxthreshold = 1.0f / 8.0f;   //边缘检测阈值

    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            copy.SetPixel(i, j, Img.GetPixel(i, j));
        }
    }
    for (int x = 1; x < w - 1; x++) {
        for (int y = 1; y < h - 1; y++) {
            float M = Pixel2Lum(copy.GetPixel(x, y));
            float N = Pixel2Lum(copy.GetPixel(x, y - 1));
            float S = Pixel2Lum(copy.GetPixel(x, y + 1));
            float W = Pixel2Lum(copy.GetPixel(x - 1, y));
            float E = Pixel2Lum(copy.GetPixel(x + 1, y));
            float NW = Pixel2Lum(copy.GetPixel(x - 1, y - 1));
            float NE = Pixel2Lum(copy.GetPixel(x + 1, y - 1));
            float SW = Pixel2Lum(copy.GetPixel(x - 1, y + 1));
            float SE = Pixel2Lum(copy.GetPixel(x + 1, y + 1));

            float Max= five_max(N, S, W, E,M);
            float Min= five_min(N, S, W, E, M);
            float range= Max - Min;
            if (range <max(minThreshold, Max * maxthreshold)) { 
                continue;
            }

            float gx = (E - W + NE - NW + SE - SW) * 0.25f;
            float gy = (S - N + SE - NE + SW - NW) * 0.25f;
            bool isHorizontal = fabs(gy) > fabs(gx);
            int stepX = isHorizontal ? 1 : 0;
            int stepY = isHorizontal ? 0 : 1;
            float gPos = isHorizontal ? N : E;
            float gNeg = isHorizontal ? S : W;
            if (fabs(gNeg - M) < fabs(gPos - M)) { //判断正负两个方向哪个差距更大
                stepX = -stepX;
                stepY = -stepY;
            }
            int sampleX = x+ stepX;
            if(sampleX<0) sampleX=0;
            if(sampleX>=w) sampleX=w-1; //防止越界
            int sampleY = y+ stepY;
            if(sampleY<0) sampleY=0;
            if(sampleY>=h) sampleY=h-1;
            float filter = fabs((N + E + S + W) * 0.25f - M);
            float pixelBlend = clamp(filter / contrast);
            pixelBlend *= pixelBlend; //混合系数
            Vector3f sampleColor = copy.GetPixel(sampleX, sampleY);
            Vector3f originalColor = copy.GetPixel(x, y);
            Vector3f result = sampleColor * pixelBlend + originalColor * (1.0f - pixelBlend);//加权混合
            result = Vector3f(
                clamp(result.x()),
                clamp(result.y()),
                clamp(result.z())
            );
            Img.SetPixel(x,y,result);
        }
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

    Tracer tr;
    //景深功能
    double ape_rad=0.8 ; //光圈半径
    double focal_dis=80.0; //焦距
    //运动模糊功能
    float time =0.0f;
    SceneParser sp(inputFile.c_str());
    Camera *camera = sp.getCamera();
    Group *group = sp.getGroup();
    Image I(camera->getWidth(), camera->getHeight());
    // Then loop over each pixel in the image, shooting a ray
    if(RT){
        #pragma omp parallel for schedule(dynamic, 1) collapse(2)
        for(int x=0;x<camera->getWidth();x++){
            for(int y=0;y<camera->getHeight();y++){
                Ray camRay = sp.getCamera()->generateRay(Vector2f(x, y)) ;
                Vector3f color = Vector3f::ZERO;
                unsigned int seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                unsigned int each_seed = seed +omp_get_thread_num();
                if(M_B) time = ((float)rand_r(&each_seed))/RAND_MAX;
                color = tr.Raytrace(camRay, sp, 0,time); // 调用光线追踪函数
                I.SetPixel(x,y,color);
            }
        }
    }
    //Path trace 版本 加分项外的代码部分参考了small_pt
    else{
        #pragma omp parallel for schedule(dynamic, 1) collapse(2)
        for(int x = 0; x < camera->getWidth(); x++) {
            for(int y = 0; y < camera->getHeight(); y++) {
                Vector3f color = Vector3f::ZERO;
                for (int sy = 0; sy < 2; sy++) {  //SXAA抗锯齿，部分参考了smallpt
                    for (int sx = 0; sx < 2; sx++) {
                        Vector3f subColor = Vector3f::ZERO;
                        //高精度时钟
                        for (int s = 0; s < SAMPLES / 4; s++) {
                            unsigned int seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                            unsigned int each_seed = seed +omp_get_thread_num();
                            float r1 = 2 * ((float) rand_r(&each_seed) / RAND_MAX);
                            float dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                            float r2 = 2 * ((float)rand_r(&each_seed) / RAND_MAX);
                            float dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                            float u = x + (sx + 0.5f + dx) / 2.0f;
                            float v = y + (sy + 0.5f + dy) / 2.0f;
                            //运动模糊 默认实现了球体的运动轨迹
                            if(M_B) time = ((float)rand_r(&each_seed))/RAND_MAX;
                            Vector2f pixelPos(u, v);
                            if(DOF){
                                Ray camRay = camera->generateRay_Dof(pixelPos,ape_rad,focal_dis,each_seed);
                                subColor += tr.Pathtrace(camRay, sp, 0,each_seed,time);
                            }
                            else{
                                Ray camRay = camera->generateRay(pixelPos);
                                subColor += tr.Pathtrace(camRay, sp, 0,each_seed,time);
                            }
                            //还原球体的位置

                        }
                        color += subColor; // 0.25 for 2x2 subpixel average
                    }
                }
                color=color* 0.25f / (SAMPLES / 4);
                color=Vector3f(
                    Gamma(color.x()),
                    Gamma(color.y()),
                    Gamma(color.z())
                );
                I.SetPixel(x, y, color);
            }
        }
    }


    // 加分项:FXAA implementation
    if(FXAA){
        apply_fxaa(I);
    }
    I.SaveImage(argv[2]);
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

