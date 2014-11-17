#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "blobObject.h"

class blobTracker : public ofBaseApp{

    public:
        void setup();
        void update();
        void draw();
        void exit();

        ofxKinect kinect;

        ofxCvColorImage colorImage;
        ofxCvColorImage ballImage[4];
        ofxCvColorImage outImage[4];

        ofxCvGrayscaleImage depthImage; // kinect grayscale depth image
        ofxCvContourFinder contourFinder;

        int nearThreshold, farThreshold; // to be used for kinect's depth clipping
        //int totalBlobCounter; // blob counter to keep track of blobs

        blobObject balls[4];
        ofPoint dest[4];
        ofPoint src[4];

        unsigned int timer;

        void manipulateBlobs(ofxCvContourFinder* contourFinder, ofxCvColorImage* origImg, ofxCvGrayscaleImage* depthImg);
        float getInitialDistance(ofxKinect* kinect);

        void keyPressed(int key);
};
