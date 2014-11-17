#include "blobTracker.h"

//--------------------------------------------------------------
void blobTracker::setup(){
    ofLogToFile("log.txt", true); // append to logfile log.txt
    kinect.setRegistration(true); // to enabe depth->video image calibration
    kinect.init(false, true, false); // no infrared, yes to video, no to texture
    kinect.open();

//    float t = getInitialDistance(&kinect);
//    ofLog() << "distance at 0 0 is " << t;

    nearThreshold = 500;
    farThreshold = 1000;
    kinect.setDepthClipping(nearThreshold, farThreshold);

    colorImage.allocate(kinect.width, kinect.height);
    depthImage.allocate(kinect.width, kinect.height);

    ballImage[0].allocate(kinect.width, kinect.height);
    ballImage[1].allocate(kinect.width, kinect.height);
    ballImage[2].allocate(kinect.width, kinect.height);
    ballImage[3].allocate(kinect.width, kinect.height);
    outImage[0].allocate(kinect.width, kinect.height);
    outImage[1].allocate(kinect.width, kinect.height);
    outImage[2].allocate(kinect.width, kinect.height);
    outImage[3].allocate(kinect.width, kinect.height);

    ofSetFrameRate(60);

    //totalBlobCounter = 0;
    dest[0] = ofPoint(0, 0);
    dest[1] = ofPoint(640, 0);
    dest[2] = ofPoint(640, 480);
    dest[3] = ofPoint(0, 480);

    timer = 0;
}

//--------------------------------------------------------------
void blobTracker::manipulateBlobs(ofxCvContourFinder* contourFinder, ofxCvColorImage* origImg, ofxCvGrayscaleImage* depthImg){
    // we're expecting up to 4 blobs really (4 balls max)
    for (int i = 0, n = contourFinder->nBlobs; i < n; i++) {
        ofxCvBlob blob = contourFinder->blobs.at(i);
        // balls is an array of 4
        balls[i].id = i;
        balls[i].pos.set(blob.centroid.x, blob.centroid.y);
        bool matches = balls[i].lastPos.match(balls[i].pos, 100);
        if (matches) {
            if (!balls[i].processed || (ofGetUnixTime() - timer) > 1) {
                /*
                if (balls[i].lastPos.x < balls[i].pos.x) {
                    balls[i].direction_x = left_to_right;
                } else {
                    balls[i].direction_x = right_to_left;
                }
                if (balls[i].lastPos.y < balls[i].pos.y) {
                    balls[i].direction_y = top_to_down;
                } else {
                    balls[i].direction_y = down_to_top;
                }
                */

                // src should be declared in the header-file as: ofPoint src[4]
                src[0] = blob.boundingRect.getTopLeft();
                src[1] = blob.boundingRect.getTopRight();
                src[2] = blob.boundingRect.getBottomRight();
                src[3] = blob.boundingRect.getBottomLeft();
                // ballImage is a ofxCvColorImage, which has been defined in the header and allocated in our setup function
                ballImage[i].setFromPixels(origImg->getPixels(), origImg->getWidth(), origImg->getHeight());
                unsigned char* ballPixels = ballImage[i].getPixels();
                unsigned char* depthPixels = depthImg->getPixels();
                for (int j = 0, w = depthImg->getWidth(), h = depthImg->getHeight(); j < w * h; j++) {
                    if (depthPixels[j] == 0) {
                        ballPixels[j*3] = 0;
                        ballPixels[j*3+1] = 0;
                        ballPixels[j*3+2] = 0;
                    }
                }
                ballImage[i].setFromPixels(ballPixels, ballImage[i].getWidth(), ballImage[i].getHeight());
                // outImage is a ofxCvColorImage, which has been defined in the header and allocated in our setup function
                outImage[i].warpIntoMe(ballImage[i], src, dest);
                balls[i].processed = true;
                timer = ofGetUnixTime();
            }
        } else {
            balls[i].processed = false;
        }
        balls[i].lastPos = balls[i].pos;
    }
}

//--------------------------------------------------------------
void blobTracker::update(){
    kinect.update();

    // will only be executed if the connection to the kinect is established and there's a new frame
    if(kinect.isFrameNew()) {
        colorImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
        depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        depthImage.blur(3);

        depthImage.flagImageChanged(); // mark the depthImage as being changed
        contourFinder.findContours(depthImage, 100, 50000, 4, false); // find contours

        manipulateBlobs(&contourFinder, &colorImage, &depthImage); // calling the work-horse
    }
}

//--------------------------------------------------------------
void blobTracker::draw(){
    outImage[0].draw(0, 0, outImage[0].width, outImage[0].height);
    outImage[1].draw(outImage[0].width, 0, 320, 240);
    outImage[2].draw(outImage[0].width + 320, 0, 320, 240);
    colorImage.draw(0, outImage[0].height, 320, 240);
    depthImage.draw(320, outImage[0].height, 320, 240);
    contourFinder.draw(320, outImage[0].height, 320, 240);
    //outImage[3].draw(960, kinect.height, 320, 240);

    stringstream reportStr;
    reportStr << "contourFinder has " << contourFinder.nBlobs << " blobs" << endl
              << "clipping distance for kinect depth: " << nearThreshold << "/" << farThreshold << endl
              << "fps is: " << ofGetFrameRate() << endl;
              //<< "total blobs: " << totalBlobCounter;
    ofDrawBitmapString(reportStr.str(), 0, 700);
}

//--------------------------------------------------------------
void blobTracker::exit() {
    kinect.close();
}

//--------------------------------------------------------------
void blobTracker::keyPressed(int key){
    switch (key) {
        case 'n':
            if (nearThreshold > 510 && nearThreshold < farThreshold) nearThreshold-=10;
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
        case 'N':
            if (nearThreshold < 3990 && nearThreshold < farThreshold-10) nearThreshold+=10;
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
        case 'f':
            if (farThreshold > 510 && farThreshold > nearThreshold+10) farThreshold-=10;
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
        case 'F':
            if (farThreshold < 3990 && farThreshold > nearThreshold) farThreshold+=10;
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
    }
}

//--------------------------------------------------------------
float blobTracker::getInitialDistance(ofxKinect* kinect) {
    float min = kinect->getDistanceAt(0,0);
    return min;
}
