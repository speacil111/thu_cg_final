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

#include <string>

#define SAMPLES 150
#define M_FRAME 10 //快门持续帧数 运动模糊
#define RT 0 // 0 for path tracing, 1 for ray tracing


using namespace std;
//gamma校正
inline double clamp(double x) {
    return x < 0 ? 0 : (x > 1 ? 1 : x);
}

inline double Gamma(double x){ return pow(clamp(x),1.0/2.2f); }



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
    double focal_dis=200.0; //焦距



    SceneParser sp(inputFile.c_str());
    Camera *camera = sp.getCamera();
    Group *group = sp.getGroup();
    Image I(camera->getWidth(), camera->getHeight());
    // Then loop over each pixel in the image, shooting a ray
    if(RT){
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
                            Ray camRay = camera->generateRay_Dof(pixelPos,len_rad,focal_dis);
                            subColor += tr.Pathtrace(camRay, sp, 0);
                            //还原球体的位置
                            for (auto &sph : group->getSpheres()) {
                                sph->update_center(-time * M_FRAME);
                            }
                        }
                        color += subColor * 0.25f / (SAMPLES / 4); // 0.25 for 2x2 subpixel average
                    }
                }

                color=Vector3f(
                    Gamma(color.x()),
                    Gamma(color.y()),
                    Gamma(color.z())
                );
                I.SetPixel(x, y, color);
            }
        }
    }
    I.SaveImage(argv[2]);
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

