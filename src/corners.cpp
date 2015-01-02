#include "corners.h"

void calculate_intercepts(const ofPoint &v, Intercept* intercept) {
    intercept->setPos(v.y - v.x);
    intercept->setNeg(v.y + v.x);
}

Corners::Corners(const std::vector<ofPoint> &v) {
    Intercept max_intercept;
    Intercept min_intercept;

    for(int i=0, n=v.size(); i<n; ++i) { // back to C98 style, because of target system -.-
        Intercept intercept;
        calculate_intercepts(v[i], &intercept);
        if (!max_intercept.has_pos || max_intercept.getPos() < intercept.getPos()) {
            max_intercept.setPos(intercept.getPos());
            this->bl = v[i];
        }
        if (!min_intercept.has_pos || min_intercept.getPos() > intercept.getPos()) {
            min_intercept.setPos(intercept.getPos());
            this->tr = v[i];
        }
        if (!max_intercept.has_neg || max_intercept.getNeg() < intercept.getNeg()) {
            max_intercept.setNeg(intercept.getNeg());
            this->br = v[i];
        }
        if (!min_intercept.has_neg || min_intercept.getNeg() > intercept.getNeg()) {
            if(!(v[i].x == 0 && v[i].y == 0)) {
                min_intercept.setNeg(intercept.getNeg());
                this->tl = v[i];
            }
        }
    }
}
