#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    aravis.listDevices();
    bInited = aravis.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
    aravis.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    glm::vec2 center = glm::vec2( ofGetWidth()/2, ofGetHeight()/2 );
    
    std::string fps = ofToString(int(ofGetFrameRate()));
    std::string temp = ofToString(aravis.getTemperature());
    std::string sensorWidth = ofToString(aravis.getSensorWidth());
    std::string sensorHeight = ofToString(aravis.getSensorHeight());
    std::string expValue = ofToString(aravis.getExposureValue());
    ArvAuto expAuto = aravis.getExposureAuto();
    std::string expAutoMode = expAuto == 0 ? "ARV_AUTO_OFF" : expAuto == 1 ? "ARV_AUTO_ONCE" : "ARV_AUTO_CONTINUOUS";
    
    if (bInited) {
        aravis.draw(0,0,ofGetWidth(), ofGetHeight());
//        ofDrawBitmapStringHighlight("SENSOR: " + sensorWidth + " x " + sensorHeight, glm::vec2(10,40));
        ofDrawBitmapStringHighlight("SENSOR: " + sensorWidth + " x " + sensorHeight, glm::vec2(10,40));
        ofDrawBitmapStringHighlight("CELCIUS: " + temp, glm::vec2(10,60));
        ofDrawBitmapStringHighlight("FPS: " + fps, glm::vec2(10,80));
        ofDrawBitmapStringHighlight("EXPOSURE: " + expValue, glm::vec2(10,100));
        ofDrawBitmapStringHighlight("MODE: " + expAutoMode, glm::vec2(10,120));
    } else {
        ofDrawBitmapStringHighlight("No camera connected.", glm::vec2(ofGetWidth()/2 - 40,ofGetHeight()/2 - 40));
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
