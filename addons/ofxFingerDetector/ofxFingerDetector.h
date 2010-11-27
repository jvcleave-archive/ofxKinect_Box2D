#ifndef _VIDEO_HAND_FINGER_FINDER_
#define _VIDEO_HAND_FINGER_FINDER_

#include "ofMain.h"
#include "ofxCvMain.h"
#include "ofxVectorMath.h"



//-----------------------------------------------|
class ofxFingerDetector{
	
	public:
	
	ofxFingerDetector();
	
	bool findFingers(ofxCvBlob blob);
	bool findHands(ofxCvBlob smblob);
	void draw(float x, float y);
	void drawhands(float x, float y);
	
	
	void findFarthestPoint(vector<ofxPoint2f> hand, ofPoint centroid, int position, int i);
	
	float dlh,max;
	
	int handPositions[2];
	
	vector  <ofxPoint2f>		fingerPoints;
	vector  <ofxPoint2f>		handPoints;
	
	vector	<float>				fingersPointCurve;
	vector	<float>				handPointCurve;
	
	vector	<bool>				bfingerRuns;
	
	vector  <ofxPoint2f>		leftHand;
	vector  <ofxPoint2f>		rightHand;
	
	ofxVec2f	v1, v2, aux1;
	 
	ofxVec3f	v1D, vxv;
	ofxVec3f	v2D;
	 
	int k,smk;
	
	ofPoint handCentroid;
	
	 float teta,lhd;

};
//-----------------------------------------------|


#endif	
