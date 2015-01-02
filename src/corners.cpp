#include "corners.h"

void calculate_intercepts(const ofPoint &v, Intercept* intercept) {
    intercept->setPos(v.y - v.x);
    intercept->setNeg(v.y + v.x);
}

Corners::Corners() {
    this->setTL(ofPoint(0, 0));
    this->setTR(ofPoint(640, 0));
    this->setBL(ofPoint(0, 480));
    this->setBR(ofPoint(640, 480));
}

Corners::Corners(const std::vector<ofPoint> &v) {
    this->autoget_corners(v);
}

void Corners::autoget_corners(const std::vector<ofPoint> &v) {
    Intercept max_intercept;
    Intercept min_intercept;

    for(int i=0, n=v.size(); i<n; ++i) { // back to C98 style, because of target system -.-
        Intercept intercept;
        calculate_intercepts(v[i], &intercept);
        if (!max_intercept.has_pos || max_intercept.getPos() < intercept.getPos()) {
            max_intercept.setPos(intercept.getPos());
            this->setBL(v[i]);
        }
        if (!min_intercept.has_pos || min_intercept.getPos() > intercept.getPos()) {
            min_intercept.setPos(intercept.getPos());
            this->setTR(v[i]);
        }
        if (!max_intercept.has_neg || max_intercept.getNeg() < intercept.getNeg()) {
            max_intercept.setNeg(intercept.getNeg());
            this->setBR(v[i]);
        }
        if (!min_intercept.has_neg || min_intercept.getNeg() > intercept.getNeg()) {
            if(!(v[i].x == 0 && v[i].y == 0)) {
                min_intercept.setNeg(intercept.getNeg());
                this->setTL(v[i]);
            }
        }
    }
}

void Corners::setTL(ofPoint p) {
    tl = p;
}

void Corners::setTR(ofPoint p) {
    tr = p;
}

void Corners::setBL(ofPoint p) {
    bl = p;
}

void Corners::setBR(ofPoint p) {
    br = p;
}