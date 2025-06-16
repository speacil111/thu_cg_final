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
#include "trace.hpp"

#include <string>

//定义避免自相交的常亮0.001f
#define EPSILON 0.001f
#define SAMPLES 150
#define RT 0 // 0 for path tracing, 1 for ray tracing

using namespace std;

inline double clamp(double x) {
    return x < 0 ? 0 : (x > 1 ? 1 : x);
}

inline int toInt(double x){ return int(pow(clamp(x),1/2.2)*255+.5); }



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
    Tracer rt,pt;

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
                color= rt.Raytrace(camRay, sp, 0); // 调用光线追踪函数
                color= Vector3f(
                    clamp(color.x()),
                    clamp(color.y()),
                    clamp(color.z())
                );
                I.SetPixel(x,y,color);
            }
        }
    }
    //Path trace 版本 参考了small_pt
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
                            Vector2f pixelPos(u, v);
                            Ray camRay = camera->generateRay(pixelPos);
                            subColor += pt.Pathtrace(camRay, sp, 0);
                        }
                        color += subColor * 0.25f / (SAMPLES / 4); // 0.25 for 2x2 subpixel average
                    }
                }

                color = Vector3f(
                    clamp(color.x()),
                    clamp(color.y()),
                    clamp(color.z())
                );
                I.SetPixel(x, y, color);
            }
        }
    }
    I.SaveImage(argv[2]);
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

