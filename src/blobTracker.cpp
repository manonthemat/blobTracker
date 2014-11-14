#include "blobTracker.h"

//--------------------------------------------------------------
void blobTracker::setup(){
    ofLogToFile("log.txt", true); // append to logfile log.txt
    kinect.setRegistration(true); // to enabe depth->video image calibration
    kinect.init(false, true, false); // no infrared, yes to video, no to texture
    kinect.open();

    nearThreshold = 500;
    farThreshold = 1200;
    kinect.setDepthClipping(nearThreshold, farThreshold);

    colorImage.allocate(kinect.width, kinect.height);
    depthImage.allocate(kinect.width, kinect.height);

    ofSetFrameRate(60);

    totalBlobCounter = 0;
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
        contourFinder.findContours(depthImage, 100, (kinect.width * kinect.height), 10, false); // find contours

        for (int i = 0, n = contourFinder.nBlobs; i < n; i++) {
            ofxCvBlob blob = contourFinder.blobs.at(i);
            ofLog() << "I found a blob with #" << i << endl
                    << "at X: " << blob.centroid.x << " Y: " << blob.centroid.y << endl
                    << "with " << blob.nPts << " points";
            balls[i].id = i;
            balls[i].pos.set(blob.centroid.x, blob.centroid.y);
            float distance = balls[i].lastPos.distance(balls[i].pos);
            bool matches = balls[i].lastPos.match(balls[i].pos, 100);
            if (matches != true) {
                ofLog() << "total blobs: " << ++totalBlobCounter;
            }
            ofLog() << "Distance: " << distance << endl << "Matches: " << matches << endl;
            balls[i].size = blob.nPts;
            balls[i].lastPos = balls[i].pos;
        }
    }
}

//--------------------------------------------------------------
void blobTracker::draw(){
    depthImage.draw(0, 0, kinect.width, kinect.height);
    contourFinder.draw(0, 0, kinect.width, kinect.height);
    colorImage.draw(kinect.width, 0, kinect.width, kinect.height);

    stringstream reportStr;
    reportStr << "contourFinder has " << contourFinder.nBlobs << " blobs" << endl
              << "clipping distance for kinect depth: " << nearThreshold << "/" << farThreshold << endl
              << "fps is: " << ofGetFrameRate() << endl
              << "total blobs: " << totalBlobCounter;
    ofDrawBitmapString(reportStr.str(), 0, kinect.height+20);
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
