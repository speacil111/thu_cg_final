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

#include <string>

#define SAMPLES 150
#define M_FRAME 10 //快门持续帧数 运动模糊
#define RT 0 // 0 for path tracing, 1 for ray tracing
#define FXAA 0 //FXAA抗锯齿
#define DOF 0 //景深
#define MIS 0 //是否采用MIS混合采样

using namespace std;
//gamma校正
inline double clamp(double x) {
    return x < 0 ? 0 : (x > 1 ? 1 : x);
}

inline double Gamma(double x){ return pow(clamp(x),1.0/2.2f); }

// FXAA实现,部分参考 https://zhuanlan.zhihu.com/p/431384101

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

float Pixel2Lum(const Vector3f &pixel) {
    return 0.213f * pixel.x() + 0.715f * pixel.y() + 0.072f * pixel.z();
}

inline float smoothstep(float edge0, float edge1, float x) {
    float t = clamp((x - edge0) / (edge1 - edge0));
    return t * t * (3 - 2 * t);
}

void apply_fxaa(Image &Img) {
    int w = Img.getWidth();
    int h = Img.getHeight();
    Image copy = Image(w, h);
    float contrast = 0.0312f;
    float minThreshold = 0.2f;
    float thresholdScale = 0.125f;

    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            copy.SetPixel(i, j, Img.GetPixel(i, j));
        }
    }
    for (int x = 1; x < w - 1; x++) {
        for (int y = 1; y < h - 1; y++) {

            // 将RGB转换成亮度
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
            if (Max - Min <= max(minThreshold, Max * thresholdScale)) { 
                continue;
            }

            float filter = 2 * (N + E + S + W) + NE + NW + SE + SW;
            filter /= 12.0f;
            filter = fabs(filter - M);
            filter = clamp(filter / contrast);
            float pixelBlend = smoothstep(0.0f, 1.0f, filter);
            pixelBlend *= pixelBlend;
            //计算方向
            float vertical = fabs(N + S - 2 * M) * 2 + fabs(NE + SE - 2 * E) + fabs(NW + SW - 2 * W);
            float horizontal = fabs(E + W - 2 * M) * 2 + fabs(NE + NW - 2 * N) + fabs(SE + SW - 2 * S);
            bool isHorizontal = vertical > horizontal;
            int stepX = isHorizontal ? 0 : 1;
            int stepY = isHorizontal ? 1 : 0;
            float positive = fabs((isHorizontal ? N : E) - M);
            float negative = fabs((isHorizontal ? S : W) - M);

            if (positive < negative) {
                stepX = -stepX;
                stepY = -stepY;
            }

            int sampleX = x + stepX;
            int sampleY = y + stepY;
            if (sampleX < 0) sampleX = 0;
            if (sampleY < 0) sampleY = 0;
            if (sampleX >= w) sampleX = w - 1;
            if (sampleY >= h) sampleY = h - 1;
            //截断
            Vector3f sample = copy.GetPixel(sampleX, sampleY);
            Vector3f result=sample * pixelBlend + copy.GetPixel(x,y) * (1.0f-pixelBlend);
            //加权混合
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

    // TODO: Main RayCasting Logic
    // First, parse the scene using SceneParser
    Tracer tr;


    //景深功能
    double len_rad=0.8 ; //景深半径
    double focal_dis=80.0; //焦距

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
                color= tr.Raytrace(camRay, sp, 0); // 调用光线追踪函数
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
                for (int sy = 0; sy < 2; sy++) {
                    for (int sx = 0; sx < 2; sx++) {
                        Vector3f subColor = Vector3f::ZERO;
                        for (int s = 0; s < SAMPLES / 4; s++) {
                            float r1 = 2 * ((float)rand() / RAND_MAX);
                            float dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                            float r2 = 2 * ((float)rand() / RAND_MAX);
                            float dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                            float u = x + (sx + 0.5f + dx) / 2.0f;
                            float v = y + (sy + 0.5f + dy) / 2.0f;
                            //运动模糊 默认实现了球体的运动轨迹
                            float time = ((float)rand())/RAND_MAX;
                            for (auto &sph : group->getSpheres()) {
                                sph->update_center(time * M_FRAME);
                            }
                            Vector2f pixelPos(u, v);
                            
                            if(DOF){
                                Ray camRay = camera->generateRay_Dof(pixelPos,len_rad,focal_dis);
                                subColor += tr.Pathtrace(camRay, sp, 0);
                            }
                            else{
                                Ray camRay = camera->generateRay(pixelPos);
                                subColor += tr.Pathtrace(camRay, sp, 0);
                            }
                            //还原球体的位置
                            for (auto &sph : group->getSpheres()) {
                                sph->update_center(-time * M_FRAME);
                            }
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

