#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"

class blobTracker : public ofBaseApp{

    public:
        void setup();
        void update();
        void draw();
        void exit();

        ofxKinect kinect;

        ofxCvColorImage colorImage;
        ofxCvGrayscaleImage depthImage; // kinect grayscale depth image
        ofxCvContourFinder contourFinder;
        bool bBlackWhite;

        int nearThreshold, farThreshold; // to be used for kinect's depth clipping
        int totalBlobCounter; // blob counter to keep track of blobs

        // TODO: build a Ball structure incl. size, position, state (falling / rising)...

        void keyPressed(int key);
        void mousePressed(int x, int y, int button);
};
