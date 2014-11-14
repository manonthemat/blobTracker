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


		void keyPressed(int key);
		void mousePressed(int x, int y, int button);
};
