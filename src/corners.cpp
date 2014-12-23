#include "corners.h"

void calculate_intercepts(const ofPoint &v, Intercept* intercept) {
    intercept->setPos(v.y - v.x);
    intercept->setNeg(v.y + v.x);
}

void get_corners(const std::vector<ofPoint> &v, Corners* corners) {
    Intercept max_intercept;
    Intercept min_intercept;

    for(const auto& vertex : v) {
        Intercept intercept;
        calculate_intercepts(vertex, &intercept);
        if ((!max_intercept.has_pos) || (max_intercept.getPos() < intercept.getPos())) {
            max_intercept.setPos(intercept.getPos());
            corners->bl = vertex;
        }
        if ((!min_intercept.has_pos) || (min_intercept.getPos() > intercept.getPos())) {
            min_intercept.setPos(intercept.getPos());
            corners->tr = vertex;
        }
        if ((!max_intercept.has_neg) || (max_intercept.getNeg() < intercept.getNeg())) {
            max_intercept.setNeg(intercept.getNeg());
            corners->br = vertex;
        }
        if ((!min_intercept.has_neg) || (min_intercept.getNeg() > intercept.getNeg())) {
            ofLog() << "looking to set TL";
            if(!(vertex.x == 0 && vertex.y == 0)) {
                min_intercept.setNeg(intercept.getNeg());
                corners->tl = vertex;
                ofLog() << "setting TL";
            }
        }
    }
}
