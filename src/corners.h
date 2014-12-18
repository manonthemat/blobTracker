#pragma once

#include <vector>
#include "ofMain.h"

struct INTERCEPT {
    double pos;
    double neg;
};

struct CORNERS {
    ofPoint tl;
    ofPoint tr;
    ofPoint bl;
    ofPoint br;
};

INTERCEPT calculate_intercepts(const ofPoint &v);
CORNERS get_corners(const std::vector<ofPoint> &v);