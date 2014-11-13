#include "blobTracker.h"

//--------------------------------------------------------------
void blobTracker::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE); // cause I want to know what's going on
    kinect.setRegistration(true); // to enabe depth->video image calibration
    kinect.init(false, true, false);
    kinect.open();

    colorImage.allocate(kinect.width, kinect.height);
    depthImage.allocate(kinect.width, kinect.height);
    nearImage.allocate(kinect.width, kinect.height);
    farImage.allocate(kinect.width, kinect.height);
    grayBg.allocate(kinect.width, kinect.height);
    grayDiff.allocate(kinect.width, kinect.height);

    nearThreshold = 250;
    farThreshold = 240;
    ofSetFrameRate(60);
}

//--------------------------------------------------------------
void blobTracker::update(){
    kinect.update();

    // will only be executed if the connection to the kinect is established and there's a new frame
    if(kinect.isFrameNew()) {
        colorImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
        depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        nearImage = depthImage;
        farImage = depthImage;
        nearImage.threshold(nearThreshold, true); // increases the contrast and inverts the image
        farImage.threshold(farThreshold); // increases the contrast
        // cvAnd gets the union of the two threshold images and puts them into depthImage
        cvAnd(nearImage.getCvImage(), farImage.getCvImage(), depthImage.getCvImage(), NULL);
        depthImage.flagImageChanged(); // mark the depthImage as being changed
        contourFinder.findContours(depthImage, 10, (kinect.width * kinect.height)/3, 10, true); // find contours

        grayImage = colorImage;
        if (bLearnBackground) {
            grayBg = grayImage;
            bLearnBackground = false;
        }
        grayDiff.absDiff(grayBg, grayImage);
        grayDiff.threshold(50);
    
    }

}

//--------------------------------------------------------------
void blobTracker::draw(){
    nearImage.draw(0, 0, kinect.width/3, kinect.height/3);
    farImage.draw(kinect.width/3, 0, kinect.width/3, kinect.height/3);
    depthImage.draw(0, kinect.height/3, kinect.width/3, kinect.height/3);
    colorImage.draw(kinect.width/3, kinect.height/3, kinect.width/3, kinect.height/3);

    grayImage.draw(0, kinect.height, kinect.width/3, kinect.height/3);
    grayBg.draw(kinect.width/3, kinect.height, kinect.width/3, kinect.height/3);
    grayDiff.draw(kinect.width/3*2, kinect.height, kinect.width/3, kinect.height/3);
}

//--------------------------------------------------------------
void blobTracker::exit() {
    kinect.close();
}

//--------------------------------------------------------------
void blobTracker::keyPressed(int key){
    switch (key) {
        case 'n':
            if (nearThreshold > 0) nearThreshold--;
            break;
        case 'N':
            if (nearThreshold < 255) nearThreshold++;
            break;
        case 'f':
            if (farThreshold > 0) farThreshold--;
            break;
        case 'F':
            if (farThreshold < 255) farThreshold++;
            break;
        case ' ':
            bLearnBackground = true;
            break;
        
    }

}

//--------------------------------------------------------------
void blobTracker::keyReleased(int key){

}

//--------------------------------------------------------------
void blobTracker::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void blobTracker::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void blobTracker::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void blobTracker::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void blobTracker::windowResized(int w, int h){

}

//--------------------------------------------------------------
void blobTracker::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void blobTracker::dragEvent(ofDragInfo dragInfo){ 

}
