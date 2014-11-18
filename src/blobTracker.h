#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"
#include "blobObject.h"

class blobTracker : public ofBaseApp{

    public:
        void setup();
        void update();
        void draw();
        void exit();

        ofxKinect kinect;
        ofxOscSender sender;
        ofxOscReceiver receiver;

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

        //void sendMessage(ofxOscSender* sender, string message);
        void sendHitMessage(ofxOscSender* sender, ofPoint pos, int id, bool flipped);
        void manipulateBlobs(ofxCvContourFinder* contourFinder, ofxCvColorImage* origImg, ofxCvGrayscaleImage* depthImg);
        float getInitialDistance(ofxKinect* kinect);
        int getColorId(ofxCvColorImage* ballImage);

        void keyPressed(int key);
};
