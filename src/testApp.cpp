#include "testApp.h"

bool forceGUIDraw = true;

bool doDrawKinectRGB = true;
bool doDrawKinectRGBA = false;
bool doDrawKinectDepth= false;
bool doDrawGrayImage =false;
bool doDrawCountours = true;
bool doDrawBox2DCountours = true;
bool doDrawSimpleContours = false;
bool doFingerTracking = false;

bool useCalibrated = true;
int imageWidth = 640;
int imageHeight = 480;
int backgroundColor = 0;
//
int minArea = 0;
int maxArea = 300000;
int numConsidered = 10;
int margin = 20;
bool findHoles = false;
//
int threshold =80;
int blur =3;
bool bLearnBakground = true;
bool fingersFound=false;
bool handFound=false;
int numBlobs = 0;
//

bool bDrawLines = false;
bool bMouseForce = false;

float smoothPct = 0.75f;
int numSmoothContours =0;
int tolerance = 4;
//3.0, 0.75, 0.1
int boxGridStep = 25;
float boxMass =3.0f;
float boxBounce =0.75f;
float boxFriction = 0.1f;

float strength = 8.0f;
float damping  = 0.7f;
float minDis   = 100;

bool doFingerFollow = false;
int numFingerPoints = 0;
float lineFriction = 0.2f;
float lineRestitution =0.96f;
float lineDensity = 1.0f;
//--------------------------------------------------------------
void testApp::setup()
{
	ofSetLogLevel(OF_LOG_ERROR);
	ofBackground(255, 255, 255);
	//ofSetVerticalSync(false);
	ofSetFrameRate(30);
	//ofDisableArbTex();
	ofSetMinMagFilters(GL_NEAREST, GL_NEAREST);
	glEnable(GL_DEPTH_TEST);
	ofEnableSmoothing();
	
	threshold = 80;
	blur =3;
	maxArea = (imageWidth*imageHeight)/2;
	
	kinect.init();
	kinect.enableDepthNearValueWhite(true);
	kinect.setVerbose(false);
	kinect.open();
	int length = imageWidth*imageHeight;
	rgbaPixels		  		= NULL;
	rgbaPixels = new unsigned char[length*4];
	memset(rgbaPixels, 0, length*4*sizeof(unsigned char));
	
	grayOfImage.allocate(imageWidth, imageHeight, OF_IMAGE_GRAYSCALE);
	grayImage.allocate(imageWidth, imageHeight);
	grayBg.allocate(imageWidth, imageHeight);
	grayDiff.allocate(imageWidth, imageHeight);
	
	kinectRawImage.allocate(imageWidth, imageHeight, OF_IMAGE_COLOR);
	kinectRGBA.allocate(imageWidth, imageHeight, OF_IMAGE_COLOR_ALPHA);

	
	cvColorImage.allocate(imageWidth, imageHeight);
	
	createGUI();
	createBox2D();
  
}
void testApp::createBox2D()
{
	box2d.init();
	box2d.setGravity(0, 10);
	box2d.createFloor();
	box2d.checkBounds(true);
	box2d.setFPS(30);
	myWorld = box2d.getWorld();
	
}
//--------------------------------------------------------------
void testApp::update()
{
	ofBackground(backgroundColor, backgroundColor, backgroundColor);
	kinect.update();
	//box2d.update();
	kinectRawImage.setFromPixels(getKinectRGBPixels(), imageWidth, imageHeight, OF_IMAGE_COLOR, true);

	
	grayImage.setFromPixels(kinect.getDepthPixels(), imageWidth, imageHeight);
	if (bLearnBakground == true)
	{
		grayBg = grayImage;		// the = sign copys the pixels from grayImage into grayBg (operator overloading)
		bLearnBakground = false;
	}
	
	// take the abs value of the difference between background and incoming and then threshold:
	grayDiff.absDiff(grayBg, grayImage);
	grayDiff.blur( blur );
	grayDiff.threshold(threshold);
	
	contourFinder.blobs.clear();
	fingersFound=false;
	handFound=false;
	numBlobs = contourFinder.findContours(grayDiff, minArea, maxArea, numConsidered, findHoles);
	//blobTracker.trackBlobs( contourFinder.blobs );
	for(int i=0; i<lineStrips.size(); i++)
	{
		lineStrips[i].clear();
	}
	lineStrips.clear();
	simpleContours.clear();
	for(int i=0; i<contourFinder.blobs.size(); i++)
	{
		
		if (contourFinder.blobs[i].nPts != -1) 
		{
			int numPoints = contourFinder.blobs[i].nPts;
			contourReg.clear();
			contourSmooth.clear();
			contourSimple.clear();
			contourReg.assign(numPoints, ofxPoint2f());
			contourSmooth.assign(numPoints, ofxPoint2f());
			
			for(int j = 0; j < numPoints; j++){
				contourReg[j] = contourFinder.blobs[0].pts[j];
			}
			
			contourSimp.smooth(contourReg, contourSmooth, smoothPct);
			contourSimp.simplify(contourSmooth, contourSimple, tolerance);
			simpleContours.push_back(contourSimple);
	
			ofxBox2dLine lineStrip;
			

			for (float f = contourSimple.size()-1; f >=0; f--) {
				
				lineStrip.addPoint(contourSimple[f].x, imageHeight+contourSimple[f].y);
			}
			lineStrip.setWorld(myWorld);
			lineStrip.bIsLoop = true;
			lineStrip.createShapeWithOptions(lineFriction, lineRestitution, lineDensity);
			
			lineStrips.push_back(lineStrip);	
			
		}
	}
	
	if (doFingerTracking) {
		if(numBlobs > 0)
		{
			
			
			fingersFound=fingerFinder.findFingers(contourFinder.blobs[0]);
			handFound=fingerFinder.findHands(contourFinder.blobs[0]);
			if(doFingerFollow) {
				
				 numFingerPoints = fingerFinder.fingerPoints.size();

				
				
				for(int i=0; i<boxes.size(); i++) {
					int randomPoint = ofRandom(0, numFingerPoints);
					ofxPoint2f point = fingerFinder.fingerPoints[numFingerPoints];
					boxes[i].addAttractionPoint(point.x, imageHeight+point.y, strength, minDis);
					boxes[i].addDamping(damping, damping);
				}
				
			}
		}
		
	}
	
	
	
	
	box2d.update();
	if (forceGUIDraw) 
	{
		gui.show();
	}else 
	{
		gui.hide();
	}
	
}

//--------------------------------------------------------------
void testApp::draw()
{
	  
	if (doDrawKinectRGB) 
	{
		kinectRawImage.draw(0, 0, imageWidth, imageHeight);
		kinectRawImage.draw(0, imageHeight, imageWidth, imageHeight);
	}
	if (doDrawKinectRGBA) 
	{
		ofEnableAlphaBlending();
		kinectRGBA.draw(imageWidth, 0, imageWidth, imageHeight);
		ofDisableAlphaBlending();
	}
	if (doDrawKinectDepth) 
	{
		kinect.drawDepth(imageWidth, imageHeight, imageWidth, imageHeight);
		//kinect.drawDepth(0, imageHeight, imageWidth, imageHeight);
	}


	if (doDrawBox2DCountours)
	{
		for(int i=0; i<lineStrips.size(); i++)
		{
			lineStrips[i].draw();
		}
	}
		
	for(int i=0; i<circles.size(); i++)
	{
		circles[i].draw();
	}
	for(int i=0; i<boxes.size(); i++)
	{
		boxes[i].draw();
	}
	if (doDrawSimpleContours)
	{
		ofPushStyle();
		
		for(int i=0; i<simpleContours.size(); i++)
		{
			ofSetColor(0, 255, 0);
			ofNoFill();
			ofBeginShape();
			for(int j = 0; j < simpleContours[i].size(); j++)
			{
				ofVertex(simpleContours[i][j].x, simpleContours[i][j].y);
			}
			ofEndShape(true);
		}
		ofPopStyle();
	}

	//box2d.draw();
	string info = "";
	info += "Press [f] to toggle Mouse Force ["+ofToString(bMouseForce)+"]\n"; 
	info += "Press [c] for circles\n";
	info += "Press [b] for blocks\n";
	info += "Press [z] for custom particle\n";
	info += "Total Bodies: "+ofToString(box2d.getBodyCount())+"\n";
	info += "Total Joints: "+ofToString(box2d.getJointCount())+"\n\n";
	info += "numFingerPoints: "+ofToString(numFingerPoints)+"\n\n";
	ofPushStyle();
		ofSetColor(255, 255, 255);
		ofDrawBitmapString(info, imageWidth, 30);
	ofPopStyle();

	if (doDrawCountours)
	{
		contourFinder.draw(imageWidth, 0);
	}
	if (doFingerTracking) 
	{
		ofPushMatrix();
		ofPushStyle();
		if(fingersFound) fingerFinder.draw(0, 0);
		if(fingersFound) fingerFinder.draw(imageWidth, imageHeight);
		if(handFound)fingerFinder.drawhands(0, 0);
		ofPopStyle();
		ofPopMatrix();
	}
	
	gui.draw();
}
	

void testApp::exit()
{
	kinect.close();
}

void testApp::keyPressed (int key)
{
	switch (key)
	{
			
		case 'f':
		{
			doFingerFollow = !doFingerFollow;
			break;
		}
		case 'g':
		{
			forceGUIDraw = !forceGUIDraw;
			glClear(GL_COLOR_BUFFER_BIT);
			break;
		}
		case 'c':
		{
			//float r = ofRandom(4, 20);		// a random radius 4px - 20px
			ofxBox2dCircle circle;
			circle.setPhysics(3.0, 0.53, 0.1);
			int circleSize = kinect.getDistanceAt(mouseX, mouseY);
			circleSize = ofMap(circleSize, 0, 600, 1, 8, true);
			//kinect.getDistanceAt(mouseX, mouseY);
			cout << "circleSize" << circleSize <<endl;
			circle.setup(myWorld, mouseX, mouseY, circleSize);
			circles.push_back(circle);
			break;
		}
		case 'b':
		{
			doBoxGrid();
			break;
		}
		case 'x':
		{
			clearBoxes();
			break;
		}
		
	}
}
void testApp::clearBoxes()
{
	for(int i=0; i<boxes.size(); i++)
	{
		boxes[i].destroyShape();
	}
	boxes.clear();
}
void testApp::doBoxGrid()
{

	clearBoxes();
	int w = imageWidth;
	int h = imageHeight;

	cvColorImage.setFromPixels(getKinectRGBPixels(), w, h);
	
	for(int i=0; i<w; i+=boxGridStep)
	{
		for(int j=0; j<h; j+=boxGridStep)
		{
			ImageBox rect;
			float boxSize = kinect.getDistanceAt(i, j);
			boxSize = ofMap(boxSize, 0, 600, 30, 2, true);
			//rect.setPhysics(3.0, 0.53, 0.1);
			rect.setPhysics(boxMass, boxBounce, boxFriction);
			cvColorImage.setROI(ofRectangle(i, j, boxSize, boxSize));
			ofImage boxImage;
			boxImage.setFromPixels(cvColorImage.getRoiPixels(), boxSize, boxSize, OF_IMAGE_COLOR, true);
			rect.setTexture(boxImage);
			cvColorImage.resetROI();
			rect.setup(myWorld, i, j, boxSize, boxSize);
			boxes.push_back(rect);
			

		}
	}
}
void testApp::createGUI()
{
	gui.setAutoSave(false);
	gui.addToggle("useCalibrated", useCalibrated);
	gui.addToggle("doDrawKinectRGB", doDrawKinectRGB);
	gui.addToggle("doDrawKinectRGBA", doDrawKinectRGBA);
	gui.addToggle("doDrawKinectDepth", doDrawKinectDepth);
	gui.addToggle("doDrawGrayImage", doDrawGrayImage);
	gui.addToggle("doDrawBox2DCountours", doDrawBox2DCountours);
	gui.addToggle("doDrawSimpleContours", doDrawSimpleContours);

	gui.addPage("BOX2D");
	gui.addTitle("CONTOUR SIMPLIFICATION", 22);
		gui.addSlider("smoothPct", smoothPct, 0.01f, 1.0f);
		gui.addSlider("tolerance", tolerance, 1, 40);
		gui.addToggle("doDrawBox2DCountours", doDrawBox2DCountours);
		gui.addToggle("doDrawSimpleContours", doDrawSimpleContours);
		
		gui.addTitle("CONTOURS", 22);
		gui.addSlider("lineFriction", lineFriction, 0.00f, 0.5f);
		gui.addSlider("lineRestitution", lineRestitution, 0.00f, 0.20f);
		gui.addSlider("lineDensity", lineDensity, 0.00f, 2.00f);
	
		gui.addTitle("BOXES", 22);
		gui.addSlider("boxGridStep", boxGridStep, 10, 100);
		gui.addSlider("boxMass", boxMass, 0.00f, 10.00f);
		gui.addSlider("boxBounce", boxBounce, 0.00f, 3.00f);
		gui.addSlider("boxFriction", boxFriction, 0.00f, 0.50f);
	
	gui.addTitle("FOLLOWING", 22);
		gui.addSlider("strength", strength, 0.00f, 16.00f);
		gui.addSlider("damping", damping, 0.00f, 0.70f);
		gui.addSlider("minDis", minDis, 1, 100);

	gui.addPage("BLOBS");
	gui.addTitle("countourFinder", 22);
	gui.addSlider("minArea", minArea, 0.0, 1000);
	gui.addSlider("maxArea", maxArea, 0.0, 400000);
	gui.addSlider("numConsidered", numConsidered, 0.0, 100);
	gui.addToggle("findHoles", findHoles);
	gui.addSlider("threshold", threshold, 0.0, 200);
	gui.addSlider("blur", blur, 0.0, 20);
	
	gui.addToggle("doDrawCountours", doDrawCountours);
	gui.addToggle("doFingerTracking", doFingerTracking);
	gui.addToggle("doFingerFollow", doFingerFollow);


//	gui.addSlider("fingerFinder.k", fingerFinder.k, 10, 60);

	
}
unsigned char * testApp::getRGBAPixels(){
	ofxVec3f texcoord3d;
	unsigned char * rgbaP = rgbaPixels;
	unsigned char * rgbPixels = getKinectRGBPixels();

	unsigned char * depthPixels = kinect.getDepthPixels();
	ofxMatrix4x4 rgbDepthMatrix = kinect.getRGBDepthMatrix().getPtr();
	for ( int y = 0; y < imageHeight; y++) {
		for ( int x = 0; x <imageWidth; x++) {
			texcoord3d.set(x,y,0);
			texcoord3d = rgbDepthMatrix * texcoord3d ;
			texcoord3d.x = ofClamp(texcoord3d.x,0,640);
			texcoord3d.y = ofClamp(texcoord3d.y,0,480);
			int pos = int(texcoord3d.y)*640+int(texcoord3d.x);
			*rgbaP++ = rgbPixels[pos*3];
			*rgbaP++ = rgbPixels[pos*3+1];
			*rgbaP++ = rgbPixels[pos*3+2];
			*rgbaP++ = depthPixels[y*640+x];
		}
	}
	return rgbaPixels;
}
unsigned char * testApp::getKinectRGBPixels(){
	if (useCalibrated) {
		return kinect.getCalibratedRGBPixels();
	}else {
		return kinect.getPixels();
	}
}
//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

