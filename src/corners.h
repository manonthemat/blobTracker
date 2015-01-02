#pragma once

#include <vector>
#include "ofMain.h"
#include "intercept.h"

class Corners {
    public:
        Corners();
        Corners(const std::vector<ofPoint> &v);
        void autoget_corners(const std::vector<ofPoint> &v);
        ofPoint getTL() { return tl; };
        ofPoint getTR() { return tr; };
        ofPoint getBL() { return bl; };
        ofPoint getBR() { return br; };
        void setTL(ofPoint p);
        void setTR(ofPoint p);
        void setBL(ofPoint p);
        void setBR(ofPoint p);
    private:
        ofPoint tl;
        ofPoint tr;
        ofPoint bl;
        ofPoint br;
};

void calculate_intercepts(const ofPoint &v, Intercept* intercept);
