#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include "sphere.hpp"
#include <iostream>
#include <vector>
using namespace std;


// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group() {

    }

    explicit Group (int num_objects) {
        objects.resize(num_objects);
    }

    ~Group() override {
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        bool flag = false;
        for (int i=0;i<objects.size();i++) {
            if (objects[i]->intersect(r, h, tmin)) {
                flag = true;
            }
        }
        return flag;
    }

    void addObject(int index, Object3D *obj) {
        objects[index] = obj;
    }


    int getGroupSize() {
        return objects.size();
    }

    Object3D * getObject(int index) {
        if (index < 0 || index >= objects.size()) {
            cerr << "Index out of bounds in Group::getObject" << endl;
            return nullptr;
        }
        return objects[index];
    }
    virtual ObjectType getType() const override {
        return GROUP;
    }
    // 获取所有球体对象
    vector<Sphere*> getSpheres() {
        vector<Sphere*> spheres;
        for (auto obj : objects) {
            if(obj->getType() == SPHERE) {
                Sphere* sphere = dynamic_cast<Sphere*>(obj);
                if (sphere) {
                    spheres.push_back(sphere);
                }
            }
        }
        return spheres;
    }

private:
    vector <Object3D *> objects;
};

#endif
	
