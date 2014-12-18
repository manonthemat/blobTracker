#include "corners.h"

INTERCEPT calculate_intercepts(const ofPoint &v) {
    INTERCEPT intercept;
    intercept.pos = v.y - v.x;
    intercept.neg = v.y + v.x;
    return intercept;
}

CORNERS get_corners(const std::vector<ofPoint> &v) {
    CORNERS corners;
    const double init = -1.0; // damn you Xcode... I want it C++11 style!
    INTERCEPT max_intercept;
    INTERCEPT min_intercept;
    max_intercept.pos = init; // use of magic constant
    max_intercept.neg = init;
    min_intercept.pos = init;
    min_intercept.neg = init;
    for(int i=0, n=v.size(); i<n; ++i) {
        INTERCEPT intercept = calculate_intercepts(v[i]);
        if ((max_intercept.pos == init) || (max_intercept.pos < intercept.pos)) {
            max_intercept.pos = intercept.pos;
            corners.bl = v[i];
        }
        if ((min_intercept.pos == init) || (min_intercept.pos > intercept.pos)) {
            min_intercept.pos = intercept.pos;
            corners.tr = v[i];
        }
        if ((max_intercept.neg == init) || (max_intercept.neg < intercept.neg)) {
            max_intercept.neg = intercept.neg;
            corners.br = v[i];
        }
        if ((min_intercept.neg == init) || (min_intercept.neg > intercept.neg)) {
            // yeah, you can't have the contour in the very origin of the viewport...
            ofLog() << "looking to set TL";
            if(!(v[i].x == 0 && v[i].y == 0)) {
                min_intercept.neg = intercept.neg;
                corners.tl = v[i];
                ofLog() << "setting TL";
            }
        }
    }
    return corners;
}
