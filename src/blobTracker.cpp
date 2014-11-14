#include "blobTracker.h"

//--------------------------------------------------------------
void blobTracker::setup(){
    ofLogToFile("log.txt", true); // append to logfile log.txt
    kinect.setRegistration(true); // to enabe depth->video image calibration
    kinect.init(false, true, false);
    kinect.open();

    nearThreshold = 500;
    farThreshold = 800;
    kinect.setDepthClipping(nearThreshold, farThreshold);

    colorImage.allocate(kinect.width, kinect.height);
    depthImage.allocate(kinect.width, kinect.height);

    ofSetFrameRate(60);
}

//--------------------------------------------------------------
void blobTracker::update(){
    kinect.update();

    // will only be executed if the connection to the kinect is established and there's a new frame
    if(kinect.isFrameNew()) {
        colorImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
        depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        depthImage.blur(3);

        if (bBlackWhite) {
            // turn shades of gray into white -> turn the kinect depth image into a black/white image
            unsigned char* pixels = depthImage.getPixels();
            for (int i = 0, n = depthImage.getHeight() * depthImage.getWidth(); i < n; i++) {
                if (pixels[i] != 0) {
                    pixels[i] = 255;
                }
            }
        }
        depthImage.flagImageChanged(); // mark the depthImage as being changed
        contourFinder.findContours(depthImage, 100, (kinect.width * kinect.height)/3, 10, false); // find contours
    }

}

//--------------------------------------------------------------
void blobTracker::draw(){
    depthImage.draw(0, 0, kinect.width/3, kinect.height/3);
    contourFinder.draw(0, 0, kinect.width/3, kinect.height/3);
    colorImage.draw(kinect.width/3, 0, kinect.width/3, kinect.height/3);

    stringstream reportStr;
    reportStr << "contourFinder has " << contourFinder.nBlobs << " blobs" << endl
              << "clipping distance for kinect depth: " << nearThreshold << "/" << farThreshold << endl
              << "fps is: " << ofGetFrameRate();
    ofDrawBitmapString(reportStr.str(), 0, kinect.height/3*2+40);
}

//--------------------------------------------------------------
void blobTracker::exit() {
    kinect.close();
}

//--------------------------------------------------------------
void blobTracker::keyPressed(int key){
    switch (key) {
        case 'n':
            if (nearThreshold > 500 && nearThreshold < farThreshold) nearThreshold--;
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
        case 'N':
            if (nearThreshold < 4000 && nearThreshold < farThreshold-1) nearThreshold++;
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
        case 'f':
            if (farThreshold > 500 && farThreshold > nearThreshold+1) farThreshold--;
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
        case 'F':
            if (farThreshold < 4000 && farThreshold > nearThreshold) farThreshold++;
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
        case 'b':
            bBlackWhite = !bBlackWhite;
            break;
    }

}

//--------------------------------------------------------------
void blobTracker::mousePressed(int x, int y, int button){

}
