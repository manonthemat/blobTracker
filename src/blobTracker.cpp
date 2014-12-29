#include "blobTracker.h"

//--------------------------------------------------------------
void blobTracker::setup(){
    drawCams = false;
    configured = false;
    kinect.setRegistration(true); // to enabe depth->video image calibration
    kinect.init(false, true, false); // no infrared, yes to video, no to texture
    kinect.open();
    origNearClipping = kinect.getNearClipping();
    origFarClipping = kinect.getFarClipping();
    
    flip = true;

    nearThreshold = 1480;
    farThreshold = 2220;

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

    //presetPoints();

    timer = 0;

    // setting up the networking
    sender.setup("localhost", 9999);
    receiver.setup(7600);
    sendConfigStatus(&sender, 0); // send OSC message to show auto-configure screen in unity3d
    tcp_server.setup(8888);
    tcp_server.setMessageDelimiter(";");

    
    kinect.setDepthClipping(nearThreshold, farThreshold);
    configured = autoConfigureViewport(&kinect);
}

//--------------------------------------------------------------
void blobTracker::presetPoints() {
    dest[0] = ofPoint(30, 85);
    dest[1] = ofPoint(625, 75);
    dest[2] = ofPoint(545, 420);
    dest[3] = ofPoint(105, 422);
}

//--------------------------------------------------------------
bool blobTracker::autoConfigureViewport(ofxKinect* kinect) {
    // setting the dest(ination) points through this function
    // on successful execution should return true (which should be assigned to the boolean variable "configured" in this example
    kinect->setDepthClipping(origNearClipping, origFarClipping);
    configImage.setFromPixels(kinect->getPixelsRef());
    configImage.blur(5);
    //configFinder.setAutoThreshold(true);
    configFinder.setThreshold(128);
    configFinder.setMinArea(100000);
    configFinder.setUseTargetColor(true);
    configFinder.setSimplify(true);
    configFinder.setTargetColor(ofColor(0, 0, 255));
    configFinder.findContours(configImage);
    if (configFinder.size() != 0) {
        vector<cv::Point> contours = configFinder.getContour(0);

        int n = contours.size();
        vector<ofPoint> points;
        points.resize(n);
        for(int i=0; i<n; ++i) {
            ofPoint pt;
            pt.x = contours[i].x;
            pt.y = contours[i].y;
            points.push_back(pt);
        }
        get_corners(points, &corners);
        dest[0] = corners.tl;
        dest[1] = corners.tr;
        dest[2] = corners.br;
        dest[3] = corners.bl;

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

    if (kinect->isConnected()) {
        kinect->setDepthClipping(500, 4000); // setting depth clipping to standard thresholds 500/4000;
        for (int i = 4000; i > 500; i-=10) {
            //ofLog() << "i is " << i;
            kinect->setDepthClipping(i-10, i);
            kinect->update();
            unsigned char* pixels = kinect->getDepthPixels();
            // iterate over pixels, when there's a pixel that's not black, break out of the loop
            for (int p = 0, n = kinect->width * kinect->height; p < n; p++) {
                if ((p > 1000) && (p == n-1) && (pixels[p] == 0)) {
                    // if we're at the end of the iteration (all pixels are black), set all_black to true
                    //ofLog() << "n is " << n;
                    all_black = true;
                }
                else if (pixels[p] != 0) {
                    break;
                }
            }
            // if the pixels are all black (all_black is true), save i as the new farThreshold minus a security buffer and set success to true
            if (all_black) {
                nearThreshold = i - 10;
                farThreshold = i;
                ofLog() << "setting the nearThreshold to " << nearThreshold;
                ofLog() << "setting the farThreshold to " << farThreshold;
                kinect->setDepthClipping(nearThreshold, farThreshold); // set depth clipping to the new values
                success = true;
                break; // exiting the for-loop
            }
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
    if (flipped) {
        x = 1-x;
        y = 1-y;
    }
    // skyzone ladder coding:
    // add some height, because the environment is quite different to the studio
    y += 0.15;
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
        //balls[i].id = i;
        balls[i].pos.set(blob.centroid.x, blob.centroid.y);
        bool matches = balls[i].lastPos.match(balls[i].pos, 100);
        //bool matches = true;
        if (matches) {
            if (!balls[i].processed || (ofGetUnixTime() - timer) > 1) {
                src[0] = blob.boundingRect.getTopLeft();
                src[1] = blob.boundingRect.getTopRight();
                src[2] = blob.boundingRect.getBottomRight();
                src[3] = blob.boundingRect.getBottomLeft();
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
        if(!configured)
            configured = autoConfigureViewport(&kinect);
        tmp.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
        colorImage.warpIntoMe(tmp, src, dest);
        depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        depthImage.blur(3);

        depthImage.flagImageChanged(); // mark the depthImage as being changed
        contourFinder.findContours(depthImage, 1000, 2000, 4, false); // find contours
        manipulateBlobs(&contourFinder, &colorImage, &depthImage); // calling the work-horse
    }
    
    int id = tcp_server.getLastID();
    if(id > 0) { // if there's at least one client connected
        string received = tcp_server.receive(0); // in this case we're only interested in the first client
        if(received != "") ofLog() << received;
        if(received == "drawCams") drawCams = true;
        else if(received == "hideCams") drawCams = false;
        else if(received == "screenshot") ofSaveScreen("screenshot.jpg");
        else if(received == "unconfigure") configured = false;
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

        if (hue >=250 || hue <=50)
            return 0;
/*
        if (hue >= 330 || hue <= 29)
            return 0; // red
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
        // if not configured, draw blue screen
        ofBackground(0, 0, 255);
    }
    else {
        // draw white screen when configured
        ofBackground(255, 255, 255);
    }
    if (drawCams) {
        //colorImage.draw(0, 0, colorImage.width, colorImage.height);
        tmp.draw(0, 0, tmp.width, tmp.height);
        depthImage.draw(tmp.width, 0, 320, 240);
        configFinder.draw();
        contourFinder.draw(tmp.width, 0, 320, 240);

        // draw markers
        ofCircle(dest[0].x, dest[0].y, 3);
        ofDrawBitmapString("TL", dest[0].x, dest[0].y);
        ofCircle(dest[1].x, dest[1].y, 3);
        ofDrawBitmapString("TR", dest[1].x, dest[1].y);
        ofCircle(dest[2].x, dest[2].y, 3);
        ofDrawBitmapString("BR", dest[2].x, dest[2].y);
        ofCircle(dest[3].x, dest[3].y, 3);
        ofDrawBitmapString("BL", dest[3].x, dest[3].y);

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
    tcp_server.close(); // disconnecting all clients and shutting down the TCP server
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
        case 'c':
            configured = false;
            break;
    }
}

//--------------------------------------------------------------
