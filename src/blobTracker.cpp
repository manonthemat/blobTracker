#include "blobTracker.h"

//--------------------------------------------------------------
void blobTracker::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE); // cause I want to know what's going on
    kinect.setRegistration(true); // to enabe depth->video image calibration
    kinect.init(false, true, false);
    kinect.open();

    nearThreshold = 500;
    farThreshold = 800;
    kinect.setDepthClipping(nearThreshold, farThreshold);

    colorImage.allocate(kinect.width, kinect.height);
    grayBg.allocate(kinect.width, kinect.height);
    grayDiff.allocate(kinect.width, kinect.height);
    dImg.allocate(kinect.width, kinect.height);

    ofSetFrameRate(60);
}

//--------------------------------------------------------------
void blobTracker::update(){
    kinect.update();

    // will only be executed if the connection to the kinect is established and there's a new frame
    if(kinect.isFrameNew()) {
        colorImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
        dImg.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        dImg.blur(3);

        if (bBlackWhite) {
            // turn shades of gray into white -> turn the kinect depth image into a black/white image
            unsigned char* pixels = dImg.getPixels();
            for (int i = 0, n = dImg.getHeight() * dImg.getWidth(); i < n; i++) {
                if (pixels[i] != 0) {
                    pixels[i] = 255;
                }
            }
        }
        dImg.flagImageChanged(); // mark the dImg as being changed
        contourFinder.findContours(dImg, 10, (kinect.width * kinect.height)/3, 10, true); // find contours

        grayImage = colorImage;
        if (bLearnBackground) {
            grayBg = grayImage;
            bLearnBackground = false;
        }
        grayDiff.absDiff(grayBg, grayImage);
        grayDiff.threshold(50);

        cvFinder2.findContours(grayDiff, 10, (kinect.width*kinect.height)/3, 10, true);
    }

}

//--------------------------------------------------------------
void blobTracker::draw(){
    dImg.draw(0, 0, kinect.width/3, kinect.height/3);
    contourFinder.draw(0, 0, kinect.width/3, kinect.height/3);
    colorImage.draw(kinect.width/3, 0, kinect.width/3, kinect.height/3);

    grayImage.draw(0, kinect.height/3, kinect.width/3, kinect.height/3);
    grayBg.draw(kinect.width/3, kinect.height/3, kinect.width/3, kinect.height/3);
    grayDiff.draw(kinect.width/3*2, kinect.height/3, kinect.width/3, kinect.height/3);
    cvFinder2.draw(kinect.width/3*2, kinect.height/3, kinect.width/3, kinect.height/3);

    stringstream reportStr;
    reportStr << "first contourFinder has " << contourFinder.nBlobs << " blobs" << endl
              << "second contourFinder has " << cvFinder2.nBlobs << " blobs" << endl
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
        case ' ':
            bLearnBackground = true;
            break;
        case 'b':
            bBlackWhite = !bBlackWhite;
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
