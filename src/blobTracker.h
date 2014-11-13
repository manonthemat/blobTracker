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
        ofxCvGrayscaleImage grayBg, grayDiff;
        ofxCvGrayscaleImage nearImage, farImage; // the near and far thresholded images
        int nearThreshold, farThreshold;
        ofxCvContourFinder contourFinder;
        bool bLearnBackground;

        ofxCvGrayscaleImage grayImage;


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
