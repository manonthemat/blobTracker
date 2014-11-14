#ifndef blobTracker_blobObject_h
#define blobTracker_blobObject_h

struct blobObject {
    int id;
    ofVec2f lastPos;
    ofVec2f pos;
    float size;
    bool falling;
};

#endif
