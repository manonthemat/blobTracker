#ifndef blobTracker_blobObject_h
#define blobTracker_blobObject_h

enum Direction_X {
    left_to_right, right_to_left
};

enum Direction_Y {
    top_to_down, down_to_top
};

struct blobObject {
    int id;
    ofVec2f lastPos;
    ofVec2f pos;
    Direction_X direction_x;
    Direction_Y direction_y;
};

#endif
