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
        ofxCvGrayscaleImage dImg; // kinect grayscale depth image
        ofxCvGrayscaleImage grayBg, grayDiff;
        ofxCvContourFinder contourFinder, cvFinder2;
        bool bLearnBackground;
        ofxCvGrayscaleImage grayImage;

        int nearThreshold, farThreshold; // to be used for kinect's depth clipping


		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

};
