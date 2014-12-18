#include "corners.h"

INTERCEPT calculate_intercepts(const ofPoint &v) {
    INTERCEPT intercept;
    intercept.pos = v.y - v.x;
    intercept.neg = v.y + v.x;
    return intercept;
}

CORNERS get_corners(const std::vector<ofPoint> &v) {
    CORNERS corners;
    const int init = -1; // damn you Xcode... I want it C++11 style!
    INTERCEPT max_intercept;
    INTERCEPT min_intercept;
    max_intercept.pos = init; // magic constant
    max_intercept.neg = init;
    min_intercept.pos = init;
    min_intercept.neg = init;
    for(int i=0, n=v.size(); i<n; ++i) {
        INTERCEPT intercept = calculate_intercepts(v[i]);
        if ((max_intercept.pos == init) || (max_intercept.pos < intercept.pos)) {
            max_intercept.pos = intercept.pos;
            corners.tl = v[i];
        }
        if ((min_intercept.pos == init) || (min_intercept.pos > intercept.pos)) {
            min_intercept.pos = intercept.pos;
            corners.br = v[i];
        }
        if ((max_intercept.neg == init) || (max_intercept.neg < intercept.neg)) {
            max_intercept.neg = intercept.neg;
            corners.tr = v[i];
        }
        if ((min_intercept.neg == init) || (min_intercept.neg > intercept.neg)) {
            min_intercept.neg = intercept.neg;
            corners.bl = v[i];
        }
    }
    return corners;
}
