#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxOsc.h"
#include "ofxNetwork.h"
#include "blobObject.h"
#include "corners.h"


class blobTracker : public ofBaseApp{

    public:
        void setup();
        void update();
        void draw();
        void exit();

        ofxKinect kinect;
        int origNearClipping, origFarClipping;

        // Networking
        ofxOscSender sender;
        ofxOscReceiver receiver;
        ofxTCPServer tcp_server;

        ofxCvColorImage colorImage, tmp;
        ofxCvColorImage ballImage[4];
        ofxCvColorImage outImage[4];

        ofxCvGrayscaleImage depthImage; // kinect grayscale depth image
        ofxCvColorImage configImage;
        ofxCv::ContourFinder configFinder;
        ofxCvContourFinder contourFinder;

        int nearThreshold, farThreshold; // to be used for kinect's depth clipping
        int cthresh;
        int carea;

        bool drawCams, configured;

        blobObject balls[4];
        ofPoint dest[4];
        ofPoint src[4];
        bool flip;

        unsigned int timer;

        void presetPoints();
        void sendConfigStatus(ofxOscSender* sender, int config_completed);
        bool autoConfigureViewport(ofxKinect* kinect);
        bool autoConfigureClipping(ofxKinect* kinect);
        void sendHitMessage(ofxOscSender* sender, ofPoint pos, int id, bool flipped);
        void manipulateBlobs(ofxCvContourFinder* contourFinder, ofxCvColorImage* origImg, ofxCvGrayscaleImage* depthImg);
        int getColorId(ofxCvColorImage* ballImage);
        void getNetworkMessages(ofxTCPServer* server);

        void keyPressed(int key);
};
