/*
 *  ofxFingerDetector.cpp
 *  openFrameworks
 *
 *  Created by Dani Quilez on 3/30/09.
 *  Copyright 2009 Mechanics_of_destruction. All rights reserved.
 *
 */

#include "ofxFingerDetector.h"

ofxFingerDetector::ofxFingerDetector()
{
	//k is used for fingers and smk is used for hand detection
	k=35;
	smk=200;
	teta=0.f;
	handPositions[0]=0;
	handPositions[1]=0;
}
bool ofxFingerDetector::findFingers (ofxCvBlob blob)
{
	fingerPoints.clear();
	fingersPointCurve.clear();
	bfingerRuns.clear();
	
	for(int i=k; i<blob.nPts-k; i++)
	{
		
		//calculating angle between vectors
		v1.set(blob.pts[i].x-blob.pts[i-k].x, blob.pts[i].y-blob.pts[i-k].y);
		v2.set(blob.pts[i].x-blob.pts[i+k].x, blob.pts[i].y-blob.pts[i+k].y);
		
		v1D.set(blob.pts[i].x-blob.pts[i-k].x ,blob.pts[i].y-blob.pts[i-k].y, 0);
		v2D.set(blob.pts[i].x-blob.pts[i+k].x ,blob.pts[i].y-blob.pts[i+k].y, 0);
		
		vxv = v1D.cross(v2D);
		
		v1.normalize();
		v2.normalize();
		teta=v1.angle(v2);
		
		//control conditions 
		if(fabs(teta) < 40)
		{	//pik?
			if(vxv.z > 0)
			{
				bfingerRuns.push_back(true);
				//we put the select points into fingerPoints vector
				fingerPoints.push_back(blob.pts[i]);
				fingersPointCurve.push_back(teta);
			}
		}
	}
	if(fingerPoints.size()>0)
	{
		return true;
	}
	else 
	{
		return false;
	}
	
}
bool ofxFingerDetector::findHands(ofxCvBlob blob)
{
	handPoints.clear();
	handPointCurve.clear();
	leftHand.clear();
	rightHand.clear();
	
	handCentroid=blob.centroid;

	for(int i=smk; i<blob.nPts-smk; i++)
	{
		
		v1.set(blob.pts[i].x-blob.pts[i-smk].x,blob.pts[i].y-blob.pts[i-smk].y);
		v2.set(blob.pts[i].x-blob.pts[i+smk].x,blob.pts[i].y-blob.pts[i+smk].y);
		
		v1D.set(blob.pts[i].x-blob.pts[i-smk].x,blob.pts[i].y-blob.pts[i-smk].y,0);
		v2D.set(blob.pts[i].x-blob.pts[i+smk].x,blob.pts[i].y-blob.pts[i+smk].y,0);
		
		vxv = v1D.cross(v2D);
		
		v1.normalize();
		v2.normalize();
		
		teta=v1.angle(v2);
		
		if(fabs(teta) < 30)
		{	//pik?
			if(vxv.z > 0)
			{
				handPoints.push_back(blob.pts[i]);
				handPointCurve.push_back(teta);
			}
		}
	}
	for(int i=0; i<handPoints.size();i++)
	{
		if(i==0)
		{
			leftHand.push_back(handPoints[i]);
		}
		else
		{
			aux1.set(handPoints[i].x-handPoints[0].x, handPoints[i].y-handPoints[0].y);
			dlh=aux1.length();
		
			//we detect left and right hand and 
		
			if(dlh<100)
			{
				leftHand.push_back(handPoints[i]);
			}
			if(dlh>100)
			{
				rightHand.push_back(handPoints[i]);
			}
		}
	}
	//for each hand, try to find the point which is farther to the centroid of the Blob
	if(leftHand.size()>0)
	{
		findFarthestPoint(leftHand, handCentroid, 0, 0);
	}
	if(rightHand.size()>0)
	{
		findFarthestPoint(rightHand, handCentroid, 1, 0);
	}
	if(rightHand.size()>0 || leftHand.size()>0) return true;
	return false;
	//Positions of hands are in (leftHand[handPositions[0]].x, y+leftHand[handPositions[0]].y) for left hand and (rightHand[handPositions[1]].x, y+rightHand[handPositions[1]].y) for right hand
}
void ofxFingerDetector::findFarthestPoint(vector<ofxPoint2f> hand, ofPoint centroid, int position, int i)
{
	aux1.set(hand[0].x-centroid.x,hand[0].y-centroid.y);
	lhd=aux1.length();
	max=lhd;
	handPositions[position]=i;
	for(int j=1; j<hand.size(); j++)
	{
		aux1.set(hand[j].x-centroid.x, hand[j].y-centroid.y);
		lhd=aux1.length();
		if(lhd>max)
		{
			max=lhd;
			handPositions[position]=j;
		}
	}
}
void ofxFingerDetector::draw(float x, float y)
{
	for(int i=0; i<fingerPoints.size(); i++)
	{
		ofEnableAlphaBlending(); 
		ofPushStyle();
			ofFill();
			ofSetColor(255,0,0,20);
			ofCircle(x+fingerPoints[i].x, y+fingerPoints[i].y, 10);
			ofNoFill();
			ofBeginShape();
			for (int i=0; i<fingerPoints.size(); i++) {
				ofVertex(fingerPoints[i].x, fingerPoints[i].y);
			}
			ofEndShape(true);
		ofPopStyle();
		/*ofPushStyle();
			for (int i=0; i<fingerPoints.size(); i++) 
			{
				ofDrawBitmapString( ofToString(i),
								   fingerPoints[i].x, fingerPoints[i].y );
			}
		ofPopStyle();*/
		ofDisableAlphaBlending();
	}
}
void ofxFingerDetector::drawhands(float x, float y)
{
	ofFill();
	//control condition
	if(leftHand.size()>0)
	{
		ofSetColor(255,255,0);
		//ofCircle(x+leftHand[handPositions[0]].x, y+leftHand[handPositions[0]].y, 50);
		ofNoFill();
		ofBeginShape();
		for (int i=0; i<leftHand.size(); i++) {
			ofCurveVertex(leftHand[i].x, leftHand[i].y);
		}
		ofEndShape(true);
	}
	if(rightHand.size()>0)
	{
		ofSetColor(0,0,255);
		//ofCircle(x+rightHand[handPositions[1]].x, y+rightHand[handPositions[1]].y, 50);
		ofNoFill();
		ofBeginShape();
		for (int i=0; i<rightHand.size(); i++) {
			ofCurveVertex(rightHand[i].x, rightHand[i].y);
		}
		ofEndShape(true);
	}
}
