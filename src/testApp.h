#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxSimpleGuiToo.h"
#include "ofxBox2d.h"
#include "ofxShader.h"
#include "ofxFingerDetector.h"
#include "contourSimplify.h"


class ImageBox : public ofxBox2dRect{
	
public:
	ImageBox() {
	}
	ofImage myImage;
	void draw() {
		
		ofxBox2dRect::draw(false);
		ofSetColor(0xFFFFFF);
		ofEnableAlphaBlending();
		glPushMatrix();
		ofTranslate(getPosition().x, getPosition().y, 0);
		ofRotate(getRotation());
		myImage.setAnchorPercent(0.5, 0.5);
		myImage.draw(0, 0); 
		//myImage.draw(getPosition().x, getPosition().y);
		glPopMatrix();
		ofDisableAlphaBlending();
	}
	void setTexture(ofImage image)
	{
		myImage = image;
		
	}
};

class testApp : public ofBaseApp
{

	public:

		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void createGUI();
		unsigned char *			rgbaPixels;
		
		unsigned char * getRGBAPixels();
		unsigned char * getKinectRGBPixels();
		ofxKinect kinect;
    
		ofxShader shader;

		ofImage grayOfImage;
		ofImage kinectRawImage;
		ofImage kinectRGBA;
		
		ofxCvGrayscaleImage 	grayImage;
		ofxCvGrayscaleImage 	grayBg;
		ofxCvGrayscaleImage 	grayDiff;
		ofxCvColorImage cvColorImage;
	
		ofxSimpleGuiToo	gui;

		ofImage			calibratedTex;
  
		ofxCvContourFinder 	contourFinder;
		ofxFingerDetector fingerFinder;

	
		bool							bDrawLines;
		bool							bMouseForce;
		
		ofxBox2d						box2d;			  //	the box2d world
		vector		<ofxBox2dCircle>	circles;		  //	default box2d circles
		vector		<ofxBox2dPolygon>	polygons;		  //	defalut box2d polgons
		vector		<ImageBox>		boxes;			  //	defalut box2d rects
		vector      <ofxBox2dLine>		lines;			  //	default box2d lines (hacked)
		
		ofxBox2dCircle					ballJoints[5];	  //	ball joints
		ofxBox2dJoint					joints[5];		  //	box2d joints
		
		b2World *myWorld;
		vector<ofxBox2dRect> worldBlobs;
		vector<ofxBox2dLine> lineStrips;
		contourSimplify contourSimp;
	

		vector<vector <ofxPoint2f> > simpleContours;
		vector <ofxPoint2f> contourReg;
		vector <ofxPoint2f> contourSmooth;
		vector <ofxPoint2f> contourSimple;
		void createBox2D();
		void doBoxGrid();
		void clearBoxes();
};

#endif
