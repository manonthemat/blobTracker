#include "blobTracker.h"

//--------------------------------------------------------------
void blobTracker::setup(){
    ofLogToFile("log.txt", true); // append to logfile log.txt
    kinect.setRegistration(true); // to enabe depth->video image calibration
    kinect.init(false, true, false); // no infrared, yes to video, no to texture
    kinect.open();

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

    totalBlobCounter = 0;
    dest[0] = ofPoint(0, 0);
    dest[1] = ofPoint(640, 0);
    dest[2] = ofPoint(640, 480);
    dest[3] = ofPoint(0, 480);
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
        contourFinder.findContours(depthImage, 100, (kinect.width * kinect.height), 4, false); // find contours

        // blob manipulation
        for (int i = 0, n = contourFinder.nBlobs; i < n; i++) {
            ofxCvBlob blob = contourFinder.blobs.at(i);
            ofLog() << "I found a blob with #" << i << endl
                    << "at X: " << blob.centroid.x << " Y: " << blob.centroid.y << endl
                    << "with " << blob.nPts << " points";
            balls[i].id = i;
            balls[i].pos.set(blob.centroid.x, blob.centroid.y);
            float distance = balls[i].lastPos.distance(balls[i].pos);
            bool matches = balls[i].lastPos.match(balls[i].pos, 100);
            if (matches) {
                if (balls[i].lastPos.x < balls[i].pos.x) {
                    balls[i].direction_x = left_to_right;
                    ofLog() << "left to right";
                } else {
                    balls[i].direction_x = right_to_left;
                    ofLog() << "right to left";
                }
                if (balls[i].lastPos.y < balls[i].pos.y) {
                    balls[i].direction_y = top_to_down;
                    ofLog() << "top to down";
                } else {
                    balls[i].direction_y = down_to_top;
                    ofLog() << "down to top";
                }
            } else {
                ofLog() << "total blobs: " << ++totalBlobCounter;
            }
            ofLog() << "Distance: " << distance << endl << "Matches: " << matches << endl;
            balls[i].size = blob.nPts;
            balls[i].lastPos = balls[i].pos;

            ofPoint src[4];
            src[0] = blob.boundingRect.getTopLeft();
            src[1] = blob.boundingRect.getTopRight();
            src[2] = blob.boundingRect.getBottomRight();
            src[3] = blob.boundingRect.getBottomLeft();
//            ballImage[i].setFromPixels(colorImage.getPixelsRef());
            ballImage[i].setFromPixels(colorImage.getPixels(), colorImage.getWidth(), colorImage.getHeight());
            unsigned char* ballPixels = ballImage[i].getPixels();
            unsigned char* depthPixels = depthImage.getPixels();
            for (int j = 0, w = depthImage.getWidth(), h = depthImage.getHeight(); j < w * h; j++) {
                if (depthPixels[j] == 0) {
                    ballPixels[j*3] = 0;
                    ballPixels[j*3+1] = 0;
                    ballPixels[j*3+2] = 0;
                }
            }
            //ballImage[i].setFromPixels(ballPixels, 320, 240);
            ballImage[i].setFromPixels(ballPixels, ballImage[i].getWidth(), ballImage[i].getHeight());
            outImage[i].warpIntoMe(ballImage[i], src, dest);

        }
    }
}

//--------------------------------------------------------------
void blobTracker::draw(){
    depthImage.draw(0, 0, kinect.width, kinect.height);
    contourFinder.draw(0, 0, kinect.width, kinect.height);
    colorImage.draw(kinect.width, 0, kinect.width, kinect.height);
    outImage[0].draw(0, kinect.height, 320, 240);
    outImage[1].draw(320, kinect.height, 320, 240);
    outImage[2].draw(640, kinect.height, 320, 240);
    outImage[3].draw(960, kinect.height, 320, 240);

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
