#pragma once

#include <vector>
#include "ofMain.h"
#include "intercept.h"

class Corners {
    public:
        Corners(const std::vector<ofPoint> &v);
        ofPoint tl;
        ofPoint tr;
        ofPoint bl;
        ofPoint br;
};

void calculate_intercepts(const ofPoint &v, Intercept* intercept);
