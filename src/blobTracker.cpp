#include "blobTracker.h"

//--------------------------------------------------------------
void blobTracker::setup(){
    ofLogToFile("log.txt", true); // append to logfile log.txt
    drawCams = false;
    configured = false;
    kinect.setRegistration(true); // to enabe depth->video image calibration
    kinect.init(false, true, false); // no infrared, yes to video, no to texture
    kinect.open();
    origNearClipping = kinect.getNearClipping();
    origFarClipping = kinect.getFarClipping();

    nearThreshold = 500;
    farThreshold = 4000;

    colorImage.allocate(kinect.width, kinect.height);
    tmp.allocate(kinect.width, kinect.height);
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

    presetPoints();

    timer = 0;

    // setting up the networking
    sender.setup("localhost", 9999);
    receiver.setup(7600);
}

//--------------------------------------------------------------
void blobTracker::presetPoints() {
    dest[0] = ofPoint(0, 0);
    dest[1] = ofPoint(640, 0);
    dest[2] = ofPoint(640, 480);
    dest[3] = ofPoint(0, 480);
}

//--------------------------------------------------------------
bool blobTracker::autoConfigureViewport(ofxKinect* kinect) {
    // setting the dest(ination) points through this function
    // on successful execution should return true (which should be assigned to the boolean variable "configured" in this example
    kinect->setDepthClipping(origNearClipping, origFarClipping);
    configImage.setFromPixels(kinect->getPixelsRef());
    configImage.blur();
    configFinder.setAutoThreshold(true);
    configFinder.setMinArea(1000);
    configFinder.setTargetColor(ofColor::white);
    configFinder.findContours(configImage);
    if (configFinder.size() != 0) {
        cv::Rect rect = configFinder.getBoundingRect(0);
        dest[0] = ofPoint(rect.x, rect.y);
        dest[1] = ofPoint(rect.x + rect.width, rect.y);
        dest[2] = ofPoint(rect.x + rect.width, rect.y + rect.height);
        dest[3] = ofPoint(rect.x, rect.y + rect.height);
        sendConfigStatus(&sender, 1); // send OSC message to hide auto-configure screen in unity3d
        kinect->setDepthClipping(nearThreshold, farThreshold);
        return true;
    } else {
        return false;
    }
}

//--------------------------------------------------------------
bool blobTracker::autoConfigureClipping(ofxKinect* kinect) {
    bool success = false;
    bool all_black = false;

    kinect->setDepthClipping(500, 4000); // setting depth clipping to standard thresholds 500/4000;
    for (int i = 4000; i > 500; i-=10) {
        //ofLog() << "i is " << i;
        kinect->setDepthClipping(i-10, i);
        kinect->update();
        unsigned char* pixels = kinect->getDepthPixels();
        // iterate over pixels, when there's a pixel that's not black, break out of the loop
        for (int p = 0, n = kinect->width * kinect->height; p < n; p++) {
            if (p == n-1 && pixels[p] == 0) {
                //ofLog() << "n is " << n;
                all_black = true;
            }
            else if (pixels[p] != 0) {
                break;;
            }
            // if we're at the end of the iteration (all pixels are black), set all_black to true
        }
        // if the pixels are all black (all_black is true), save i as the new farThreshold minus a security buffer and set success to true
        if (all_black == true) {
            ofLog() << "setting the nearThreshold to " << i-10;
            ofLog() << "setting the farThreshold to " << i;
            nearThreshold = i - 10;
            farThreshold = i;
            kinect->setDepthClipping(nearThreshold, farThreshold); // set depth clipping to the new values
            success = true;
            break; // exiting the for-loop
        }
    }
    return success;
}

//--------------------------------------------------------------
void blobTracker::sendConfigStatus(ofxOscSender* sender, int config_completed) {
    ofxOscMessage m;
    m.setAddress("/config");
    m.addIntArg(config_completed);
    sender->sendMessage(m);
}

//--------------------------------------------------------------
void blobTracker::sendHitMessage(ofxOscSender* sender, ofPoint pos, int id, bool flipped=false) {
    if (id == -1) {
        return;
    }

    ofxOscMessage m;
    m.setAddress("/shoot");
    // if pos is out of dest bounds, return (and don't send a hit message)
    if ((pos.x < dest[0].x) || (pos.x > dest[1].x) || (pos.y < dest[0].y) || (pos.y > dest[2].y)) {
        return;
    }
    float x = (pos.x - dest[0].x) / (dest[1].x - dest[0].x);
    float y = (pos.y - dest[0].y) / (dest[2].y - dest[0].y);
    //float x = pos.x / kinect.width;
    //float y = pos.y / kinect.height;
    if (flipped) {
        x = 1-x;
        y = 1-y;
    }
    m.addFloatArg(x);
    m.addFloatArg(y);
    m.addIntArg(id);
    //ofLog() << "sending hit message " << x << " " << y << " " << id;
    sender->sendMessage(m);
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
                int c = getColorId(&outImage[i]);
                //ofLog() << "color id for ball " << i << " is " << c;
                sendHitMessage(&sender, blob.centroid, c, flip);
                outImage[i].set(0, 0, 0); // clear the outImage -> all black
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
        //colorImage.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
        tmp.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
        colorImage.warpIntoMe(tmp, src, dest);
        depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        depthImage.blur(3);

        depthImage.flagImageChanged(); // mark the depthImage as being changed
        if (configured) {
            contourFinder.findContours(depthImage, 100, 50000, 4, false); // find contours

            manipulateBlobs(&contourFinder, &colorImage, &depthImage); // calling the work-horse
        } else {
            sendConfigStatus(&sender, 0); // send OSC message to show auto-configure screen in unity3d
            configured = autoConfigureViewport(&kinect) && autoConfigureClipping(&kinect);
        }
    }
}

//--------------------------------------------------------------
int blobTracker::getColorId(ofxCvColorImage* ballImage) {
    ofPixels pixels;
    pixels.setFromPixels(ballImage->getPixels(), ballImage->width, ballImage->height, 3);
    unsigned int numPixels = 0;
    unsigned int r = 0.0;
    unsigned int g = 0.0;
    unsigned int b = 0.0;
    for (unsigned int i = 0, n = pixels.size(); i < n; i+=3) {
        if (pixels[i] == 0 && pixels[i+1] == 0 && pixels[i+2] == 0) {
            // if the pixel is black (r,g,b all zero) -> continue
            continue;
        } else {
            numPixels++;
            r += pixels[i];
            g += pixels[i+1];
            b += pixels[i+2];
        }
    }
    if (numPixels > 0) {
        float hue = ofColor((int)r/numPixels, (int)g/numPixels, (int)b/numPixels).getHueAngle();

        hue = (int) hue;
        //ofLog() << "hue: " << hue;

        if (hue >= 330 || hue <= 29)
            return 0; // red
/*
        else if (hue >= 30 && hue <= 75)
            return 1; // yellow

        else if (hue > 190 && hue <= 280)
            return 2; // blue

        else if (hue >= 76 && hue <= 170)
            return 3; // green

        return -1;
 */
        else
            return 3;
    } else {
        return -1;
    }
}

//--------------------------------------------------------------
void blobTracker::draw(){
    if (!configured) {
        // if not configured, draw red screen
        ofBackground(255, 0, 0);
    }
    else {
        // draw white screen when configured
        ofBackground(255, 255, 255);
    }
    if (drawCams) {
        //colorImage.draw(0, 0, colorImage.width, colorImage.height);
        tmp.draw(0, 0, colorImage.width, colorImage.height);
        depthImage.draw(colorImage.width, 0, 320, 240);
        contourFinder.draw(colorImage.width, 0, 320, 240);

        // draw area of interest
        ofCircle(dest[0].x, dest[0].y, 3);
        ofCircle(dest[1].x, dest[1].y, 3);
        ofCircle(dest[2].x, dest[2].y, 3);
        ofCircle(dest[3].x, dest[3].y, 3);

        stringstream reportStr;
        reportStr << "contourFinder has " << contourFinder.nBlobs << " blobs" << endl
                  << "clipping distance for kinect depth: " << nearThreshold << "/" << farThreshold << endl
                  << "fps is: " << ofGetFrameRate() << endl
                  << "flipped: " << flip << endl;
        ofDrawBitmapString(reportStr.str(), 0, 0);
    }
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
        case ' ':
            drawCams = !drawCams;
            break;
        case 'r':
            kinect.setDepthClipping(origNearClipping, origFarClipping);
            break;
        case 'R':
            kinect.setDepthClipping(nearThreshold, farThreshold);
            break;
        case 'x':
            flip = !flip;
            break;
        case 'p':
            presetPoints();
            break;
    }
}

//--------------------------------------------------------------
