//
// Created by nsl on 9/16/16.
//

#include "DeepCam/include/Object.h"

void Object::setId(int id) {
    Object::id = id;
}

void Object::setBox(const obj_box &box) {
    Object::box = box;
}

void Object::setStatus(const std::string &status) {
    Object::status = status;
}

void Object::setAlive(bool alive) {
    Object::alive = alive;
}

void Object::setTracked(bool tracked) {
    Object::tracked = tracked;
}

void Object::setEvict(bool evict) {
    Object::evict = evict;
}

void Object::setDirection(double direction) {
    Object::direction = direction;
}

void Object::setDisplacement(double displacement) {
    Object::displacement = displacement;
}

void Object::setView(const std::string &view) {
    Object::view = view;
}

void Object::setEvict_count(int evict_count) {
    Object::evict_count = evict_count;
}

void Object::setMove_count(int move_count) {
    Object::move_count = move_count;
}

void Object::setLast_box(const obj_box &last_box) {
    Object::last_box = last_box;
}
