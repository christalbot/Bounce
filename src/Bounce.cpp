#include <stdlib.h>
#include <time.h>
#include <Box2D/Box2D.h>
#include "CinderOpenCV.h"
#include "cinder/app/AppNative.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/Capture.h"
#include "cinder/gl/Texture.h"
#include "cinder/Surface.h"
#include "cinder/params/Params.h"
#include "debugDraw.h"

// Build Options
//#define HALFWINDOWS		//Uncomment to display CV windows at half size
#define CONSOLE			//Uncomment to have output displayed to console
#define VERBOSE			//Uncomment to print more information to console
#define OPENCVCAP		//Uncomment to use openCV for video capture, Comment out to use Cinder for video Capture

using namespace ci;			//Cinder
using namespace cv;			//OpenCV
using namespace ci::app;	//Cinder - App
using namespace std;		//Standard
using namespace gl;			//OpenGL


class Bounce : public AppNative {
public:

	// Option Variables   //##################################################################################################
	float							ballSize;
	Color							colourBG;
	Color							colourBalls;
	Color							colourCV;
	Scalar							mColorMin;
	Scalar							mColorMax;
	int								frameCntValue, frameCnt;
	int								ballCreationCntValue, ballCreationCnt, ballsPerIteration;
	int								hueRangeBX, hueRangeBG, hueValueBX, hueValueBG, satValueMin, valValueMin, satValueMax, valValueMax;
	int								GravValueX, GravValueY, bounceValue, densityValue, frictionValue;
	//########################################################################################################################

#ifdef OPENCVCAP
	VideoCapture					mCap;
#else
	Capture							mCap;
#endif

	params::InterfaceGl				mParams;
	debugDraw						debug;
	b2World							*mWorld;
	vector<b2Body*>					mBoxes, balls;
	vector<int>						ballsToDelete;
	b2Body*							walls[3];
	b2BodyDef						wallBodyDef;
	b2EdgeShape						wallEdgeShape;
	b2FixtureDef					wallFixtureDef;
	Surface8u						mImage;
	vector<vector<Point>>			contours;
	vector<Rectf>					foundRectanglesBX;
	Mat								inputMat, matSmall, inputHSVMat, frameTresh, frameTresh2;
	Rectf							boundary, box;
	Rect							rectCV, biggest;
	Rectf							recCIN;
	Color							col;
	ci::Vec3f						colourBoxHSV;
	float							w, h; // Width, Height
	float							xScale, yScale, topDiff, leftDiff;
	double							mApproxEps;
	int								mCannyThresh;
	bool							debugFlag, animFlag, colPickerFlag,
									pauseFlag, opencvFlag, generateBallsFlag, ballGeneratorMode;


	//Functions
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();
	void reset();
	void resize();
	void keyDown( KeyEvent event );
	void shutdown();
	void prepareSettings(Settings *settings);
	void addBox( const ci::Vec2f &pos, const float32 w, const float32 h );
	void addBall( const ci::Vec2f &pos );
	bool isBallOffScreen(const b2Body &ball);
	void deleteBalls();
	void deleteBoxes();
	void resetAll();
	void createCvWindows();
	void updateColours();

	//Callback functions (buttons, etc.)
	void buttonUpBalls();
	void buttonUpCol();
	void buttonUpPhys();
	void buttonReset();
	static void bcb(int state, void* userdata);
	static void bcbGenerator(int state, void* userdata);
	static void bcbMode(int state, void* userdata);
	static void bcbDelBalls(int state, void* userdata);

	//Toggle Functions
	void toggleMode();
	void toggleBallGen();
	void toggleDebugDraw();
	void toggleColPicker();
};



void Bounce::prepareSettings(Settings *settings){
	settings->setWindowSize(800,600);
	settings->setFrameRate(60.0f);
	settings->setAlwaysOnTop(true);

#ifdef CONSOLE
	settings->enableConsoleWindow();
#endif

}


/****************************************************************************************************************************
*
*	Setup Function
*
****************************************************************************************************************************/
void Bounce::setup()
{
	resetAll();		//Initialise Option Variables
	srand((int)time(NULL));
	enableVerticalSync();
	getWindow()->setTitle("Bounce");

	debugFlag				= false;
	animFlag				= true;
	colPickerFlag			= false;
	pauseFlag				= true;
	opencvFlag				= true;
	generateBallsFlag		= false;
	ballGeneratorMode		= false; // False = Rain; True = Waterfall
	w						= (float)getWindowWidth();
	h						= (float)getWindowHeight();
	hueRangeBX				= 15;
	hueRangeBG				= 15;
	xScale					= 1;
	yScale					= 1;
	topDiff					= 0;
	leftDiff				= 0;
	mApproxEps				= 1.0;
	mCannyThresh			= 1;
	recCIN					= Rectf();
	rectCV					= Rect();

	updateColours();
	createCvWindows();

#ifdef VERBOSE
	console() << "Box2D Version: " << b2_version.major << "." << b2_version.minor << "." << b2_version.revision << endl;
	console() << "OpenCV Version:  " << CV_VERSION << std::endl;
#endif


	// ---------------------------------------------------------------------------------------------
	// Setup options dialog of Cinder app
	// ---------------------------------------------------------------------------------------------
	mParams = params::InterfaceGl( getWindow(), "Project Options", toPixels( ci::Vec2i( 400, 600 ) ) );

	mParams.addText( "textColours", "label='OpenCV Options:'" );
	mParams.addParam("Frame Delay", &frameCntValue, "min=0, max=300");
	mParams.addParam("Background Colour", &colourBG);
	mParams.addParam("OpenCV Colour", &colourCV);
	mParams.addButton( ">>Update OpenCV Colour", std::bind( &Bounce::buttonUpCol, this ) );	

	mParams.addSeparator();
	mParams.addText( "ballOptions", "label='Ball Options:'" );
	mParams.addParam("Ball Colour", &colourBalls);
	mParams.addParam("Ball Size (Radius)", &ballSize, "min=4, max=30");
	mParams.addButton( ">>Update Current Balls", std::bind( &Bounce::buttonUpBalls, this ) );
	mParams.addText( "ballOptions2", "label='New Ball Properties:'" );
	mParams.addParam("Density", &densityValue, "min=1, max=100");
	mParams.addParam("Friction", &frictionValue, "min=1, max=100");
	mParams.addParam("Bounce", &bounceValue, "min=1, max=100");

	mParams.addSeparator();
	mParams.addText( "ballGenerator", "label='Ball Generator:'" );
	mParams.addParam("Iteration Delay (Rain)", &ballCreationCntValue, "min=0, max=50");
	mParams.addParam("Balls per Iteration (Rain)", &ballsPerIteration, "min=1, max=50");
	mParams.addButton( "Mode", std::bind( &Bounce::toggleMode, this ) );
	mParams.addButton( "Generator On/Off", std::bind( &Bounce::toggleBallGen, this ) );

	mParams.addSeparator();
	mParams.addText( "physicsOptions", "label='Physics Options:'" );
	mParams.addParam("Gravity (Y)", &GravValueY, "min=1, max=100");
	mParams.addParam("Gravity (X)", &GravValueX, "min=-5, max=5");
	mParams.addButton( ">>Update Physics", std::bind( &Bounce::buttonUpPhys, this ) );

	mParams.addSeparator();
	mParams.addText( "resetOptions", "label='Reset Options:'" );
	mParams.addButton( "Reset Everything", std::bind( &Bounce::buttonReset, this ) );
	mParams.addButton( "Delete All Balls", std::bind( &Bounce::deleteBalls, this ) );

	mParams.addSeparator();
	mParams.addText( "generalOptions", "label='Other:'" );
	mParams.addButton( "Draw Debug Data", std::bind( &Bounce::toggleDebugDraw, this ) );
	mParams.addButton( "Colour Picker", std::bind( &Bounce::toggleColPicker, this ) );

	mParams.minimize();

	//Create Physics world
	b2Vec2 gravity( 0.0f, 8.0f );
	mWorld = new b2World( gravity );

	//Setup debug draw
	mWorld->SetDebugDraw(&debug);
	debug.SetFlags( b2Draw::e_shapeBit && b2Draw::e_aabbBit && b2Draw::e_centerOfMassBit && b2Draw::e_jointBit && b2Draw::e_pairBit);


	//Static body definition for walls
	wallBodyDef.type = b2_staticBody;
	wallFixtureDef.shape = &wallEdgeShape;

	//Ground
	wallBodyDef.position.Set( 0, h - 6 );
	wallEdgeShape.Set( b2Vec2( 0,0), b2Vec2( w, 0 ) );
	walls[0] = mWorld->CreateBody(&wallBodyDef);
	walls[0]->CreateFixture(&wallFixtureDef);

	//Left Wall
	wallBodyDef.position.Set( 0, 0 );
	wallEdgeShape.Set( b2Vec2( 0,0), b2Vec2( 0, h - (ballSize * 8) - ballSize ));
	walls[1] = mWorld->CreateBody(&wallBodyDef);
	walls[1]->CreateFixture(&wallFixtureDef);

	//Right Wall
	wallBodyDef.position.Set( w, 0 );
	wallEdgeShape.Set( b2Vec2( 0,0), b2Vec2( 0, h - (ballSize * 8) - ballSize ));
	walls[2] = mWorld->CreateBody(&wallBodyDef);
	walls[2]->CreateFixture(&wallFixtureDef);


	//Setup video capture
	try {
#ifdef OPENCVCAP
		mCap = VideoCapture(0);
		if(!mCap.isOpened()) throw 1;
#else
		mCap = Capture( 640, 480 );
		mCap.start();
#endif
	}
	catch( ... ) {
		console() << "Failed to initialize capture" << std::endl;
	}
}


/****************************************************************************************************************************
*
*	Update Function
*
****************************************************************************************************************************/
void Bounce::update()
{
	if(!pauseFlag){
		if(colPickerFlag){

#ifdef OPENCVCAP
			mCap >> inputMat;
			if(!mCap.isOpened()){
#else
			if( mCap && mCap.checkNewFrame() ) {
				mImage = mCap.getSurface();
				inputMat = Mat( toOcv( mImage) );
#endif
			}
		} else {
			if(balls.size() > 1000) deleteBalls(); // Prevent to many balls to build up
			if(frameCnt >= frameCntValue){
				frameCnt = 0;

				if(opencvFlag){
					updateColours();

#ifdef OPENCVCAP
					if(mCap.isOpened()){
						mCap >> inputMat;
#else
					if( mCap && mCap.checkNewFrame() ) {
						mImage = mCap.getSurface();
						inputMat = Mat( toOcv( mImage) );
#endif


#ifdef HALFWINDOWS
						cv::resize(inputMat, matSmall, Size(inputMat.cols / 2, inputMat.rows / 2));
						imshow("Original", matSmall);
#else
						imshow("Original", inputMat);
#endif // !HALFWINDOWS


						cvtColor(inputMat, inputHSVMat, CV_BGR2HSV);




						// ---------------------------------------------------------------------------------------------
						// Track animation widow
						// ---------------------------------------------------------------------------------------------

						mColorMin = Scalar(hueValueBG - hueRangeBG, satValueMin, valValueMin);
						mColorMax = Scalar(hueValueBG + hueRangeBG, satValueMax, valValueMax);
						inRange(inputHSVMat, mColorMin, mColorMax, frameTresh);
						medianBlur(frameTresh, frameTresh, 7);
						Mat cannyMat2;
						Canny(frameTresh, cannyMat2, mCannyThresh, mCannyThresh*2.f, 3 );
						findContours(cannyMat2, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
						int ar;
						biggest = Rect(0,0,0,0);
						for( int i = 0; i < (int)contours.size(); i++ ) {
							rectCV = boundingRect(contours[i]);
							ar = rectCV.area();
							if(ar > biggest.area()){
								biggest = rectCV;
							}
						}
						recCIN = Rectf( fromOcv(biggest) );




						// ---------------------------------------------------------------------------------------------
						// Track Shapes
						// ---------------------------------------------------------------------------------------------

						mColorMin = Scalar(hueValueBX - hueRangeBX, satValueMin, valValueMin);
						mColorMax = Scalar(hueValueBX + hueRangeBX, satValueMax, valValueMax);
						inRange(inputHSVMat, mColorMin, mColorMax, frameTresh2);
						medianBlur(frameTresh2, frameTresh2, 7);
						Mat cannyMat,cc,vv;
						Canny(frameTresh2, cannyMat, mCannyThresh, mCannyThresh*2.f, 3 );
						Canny(frameTresh, cc, mCannyThresh, mCannyThresh*2.f, 3 );
						vv = cannyMat + cc;

#ifdef HALFWINDOWS
						cv::resize(vv, matSmall, Size(inputMat.cols / 2, inputMat.rows / 2));
						imshow("CV", matSmall);
#else
						imshow("CV", vv);
#endif


						//OpenCV colour range preview window
						Mat colPrevMin(50, 200, CV_8UC3, mColorMin);
						Mat colPrevMax(50, 100, CV_8UC3, mColorMax);
						cvtColor(colPrevMin, colPrevMin, CV_HSV2BGR);
						cvtColor(colPrevMax, colPrevMax, CV_HSV2BGR);
						Mat colPrevCombined(colPrevMin, Range(0,50), Range(100,200));
						colPrevMax.copyTo(colPrevCombined);
						imshow("HSV", colPrevMin);
						displayOverlay("HSV", "Min            Max");



						// ---------------------------------------------------------------------------------------------
						// Find contours of boxes
						// ---------------------------------------------------------------------------------------------
						findContours(cannyMat, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
						foundRectanglesBX.clear();
						for( int i = 0; i < (int)contours.size(); i++ ) {
							Rect rect2( boundingRect(contours[i]));
							Rectf rec( fromOcv(rect2) );
							if(recCIN.contains(rec.getCenter()))
								foundRectanglesBX.push_back(rec);
						}
					}
				} // End of OpencvFlag
			} // End of Frame count check



			//-----------------------------------------------
			// Calculate scale/placement from background box
			//-----------------------------------------------
			if(opencvFlag){
				drawStrokedRect(recCIN);
					
				topDiff = recCIN.getX1();
				leftDiff = recCIN.getY1();

				xScale = w / recCIN.getWidth();
				yScale = h / recCIN.getHeight();

				// Delete current dynamic bodies ready for new ones from boxes recognised by openCV
				deleteBoxes();

				// Create dynamic bodies from boxes recognised by openCV using scaling/placement factors calculated from animation background
				for(int i = 0; i < (int)foundRectanglesBX.size(); i++){
					box = Rectf((foundRectanglesBX.at(i).x1 * xScale) - (leftDiff * xScale), (foundRectanglesBX.at(i).y1 * yScale) - (topDiff * yScale), (foundRectanglesBX.at(i).x2 * xScale) - (leftDiff * xScale), (foundRectanglesBX.at(i).y2 * yScale) - (topDiff * yScale));
					addBox(box.getCenter(), box.getWidth() / 2, box.getHeight() / 2);
				}
			}



			//------------------------------------------
			// Delete balls when go off-screen
			//------------------------------------------
			for (int i = 0; i < (int)balls.size(); i++){
				if(isBallOffScreen(*balls.at(i))){
					ballsToDelete.push_back(i);
				}
			}
			for (int i = (int)ballsToDelete.size() - 1; i >= 0; i--){
				mWorld->DestroyBody(balls.at(ballsToDelete.at(i)));
				balls.erase(balls.begin() + ballsToDelete.at(i));
			}
			ballsToDelete.clear();
			//-------------------------------------------



			// Step - Physics engine
			for( int i = 0; i < 10; ++i ){
				mWorld->Step( 1 / 30.0f, 10, 10 );
			}


			// Increment counters
			frameCnt++;
			ballCreationCnt++;



			//Create balls
			if(generateBallsFlag){
				int y = 0;
				int x = 0;
				if(ballGeneratorMode){ // False = Rain; True = Waterfall
					//Waterfall
					y = (rand() % 100 + 1) * -1;
					x = (int)(w / 3);
					addBall(ci::Vec2f( (float)x, (float)y ));
				} else {
					//Rain
					if(ballCreationCnt >= ballCreationCntValue){
						for(int i = 0; i < ballsPerIteration; i++){
							x = rand() % (int)w + 1;
							addBall(ci::Vec2f((float)x, (float)y));
						}
						ballCreationCnt = 0;
					}
				}
			}
		}
	}
}



/****************************************************************************************************************************
*
*	Draw Function
*
****************************************************************************************************************************/
void Bounce::draw()
{
	if(pauseFlag){
		clear( Color(0,0,0) );
		drawStringCentered("Paused",ci::Vec2f(w/2,h/2),Color(0.5f,0.5f,0.5f),Font("Lucida Console",80.0f));
	} else {
		if(colPickerFlag){
			clear( Color(0,0,0) );
			gl::draw(fromOcv(inputMat));
		} else {
			color( colourBG );
			drawSolidRect(Rectf(0, 0, w, h));

			if(debugFlag){
				mWorld->DrawDebugData();
				
				color(0,0,0);
				drawStrokedRect(recCIN);

				color(1,1,1);
				for(int i = 0; i < (int)foundRectanglesBX.size(); i++){
					Rectf box((foundRectanglesBX.at(i).x1 * xScale) - (leftDiff * xScale), (foundRectanglesBX.at(i).y1 * yScale) - (topDiff * yScale), (foundRectanglesBX.at(i).x2 * xScale) - (leftDiff * xScale), (foundRectanglesBX.at(i).y2 * yScale) - (topDiff * yScale));
					drawStrokedRect(foundRectanglesBX.at(i));
					drawStrokedRect(box);
				}
			}

			if(animFlag){
				color( colourBalls );
				for( vector<b2Body*>::const_iterator ballIt = balls.begin(); ballIt != balls.end(); ++ballIt ) {
					ci::Vec2f pos( (*ballIt)->GetPosition().x, (*ballIt)->GetPosition().y );
					drawSolidCircle( ci::Vec2f( pos.x, pos.y), (*ballIt)->GetFixtureList()->GetShape()->m_radius);
				}
			}
		}
	}
	mParams.draw();
}








/****************************************************************************************************************************
*
*	Additional Functions
*
****************************************************************************************************************************/


//-------------------------------------------------
// Shutdown
//-------------------------------------------------
void Bounce::shutdown()
{
	delete mWorld;
}


//-------------------------------------------------
// Resize
//-------------------------------------------------
void Bounce::resize()
{
	w = (float32)getWindowWidth();
	h = (float32)getWindowHeight();
	//Ground
	mWorld->DestroyBody(walls[0]);
	wallBodyDef.position.Set( 0, h - 6 );
	wallEdgeShape.Set( b2Vec2( 0,0), b2Vec2( w, 0 ) );
	walls[0] = mWorld->CreateBody(&wallBodyDef);
	walls[0]->CreateFixture(&wallFixtureDef);

	//Left Wall
	mWorld->DestroyBody(walls[1]);
	wallBodyDef.position.Set( 0, 0 );
	wallEdgeShape.Set( b2Vec2( 0,0), b2Vec2( 0, h - (ballSize * 8) - ballSize ));
	walls[1] = mWorld->CreateBody(&wallBodyDef);
	walls[1]->CreateFixture(&wallFixtureDef);

	////Right Wall
	mWorld->DestroyBody(walls[2]);
	wallBodyDef.position.Set( w, 0 );
	wallEdgeShape.Set( b2Vec2( 0,0), b2Vec2( 0, h - (ballSize * 8) - ballSize ));
	walls[2] = mWorld->CreateBody(&wallBodyDef);
	walls[2]->CreateFixture(&wallFixtureDef);

#ifdef VERBOSE
	console() << "Window resized to: " << getWindowWidth() << "x" << getWindowHeight() << endl;
#endif
}




//-------------------------------------------------
// Add Box (Static Body)
//-------------------------------------------------
void Bounce::addBox( const ci::Vec2f &pos, const float32 w, const float32 h )
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set( pos.x, pos.y );

	b2Body *body = mWorld->CreateBody( &bodyDef );

	b2PolygonShape dynamicBox;
	dynamicBox.SetAsBox( w, h );

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.3f;
	fixtureDef.restitution = 0.0f; // bounce

	body->CreateFixture( &fixtureDef );
	mBoxes.push_back( body );
}

//-------------------------------------------------
// Add Ball (Dynamic Body)
//-------------------------------------------------
void Bounce::addBall( const ci::Vec2f &pos )
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set( pos.x, pos.y );
	b2Body *ball = mWorld->CreateBody( &bodyDef );
	b2CircleShape dynamicBall;
	dynamicBall.m_p.Set(0, 0);
	dynamicBall.m_radius = ballSize;


	b2FixtureDef ballFixtureDef;
	ballFixtureDef.shape = &dynamicBall;

	ballFixtureDef.density = (float)densityValue / 100;
	ballFixtureDef.friction = (float)frictionValue / 100;
	ballFixtureDef.restitution = (float)bounceValue / 100; // bounce

	ball->CreateFixture( &ballFixtureDef );

	if(ballGeneratorMode){ // False = Rain; True = Waterfall
		//Waterfall
		ball->SetAngularVelocity(0.01f);
		ball->ApplyAngularImpulse(0.01f);
		ball->ApplyLinearImpulse(b2Vec2(1000,1000), ball->GetWorldCenter());
	} else {
		//Rain
		ball->SetAngularVelocity(0.01f);
		ball->ApplyAngularImpulse(0.01f);
	}
	balls.push_back( ball );
}


//-------------------------------------------------
// Mouse Down
//-------------------------------------------------
void Bounce::mouseDown( MouseEvent event )
{
	if(!pauseFlag){
		if(colPickerFlag){

#ifdef OPENCVCAP
			mImage = fromOcv(inputMat);
#endif

			if( mImage&&mImage.getBounds().contains( event.getPos() ) ) {
				int rr, bb, gg;
				rr = mImage.getPixel( event.getPos() ).r;
				bb = mImage.getPixel( event.getPos() ).b;
				gg = mImage.getPixel( event.getPos() ).g;
				colourCV = Color8u(rr,gg,bb);
				updateColours();

#ifdef VERBOSE
				console() << "Hue: " << hueValueBX << endl << "R: " << rr << endl << "B: " << bb << endl << "G: " << gg << endl;
#endif

			}
			colPickerFlag = !colPickerFlag;
		} else {
			if(event.isLeft()){
				addBall( event.getPos() );
			} else {

				if(event.isShiftDown()){
					addBox( event.getPos(), 30, 30 );
				} else if(event.isAltDown()){
					addBox( event.getPos(), 80, 80 );
				} else if(event.isControlDown()){
					addBox( event.getPos(), 100, 100 );
				} else {
					addBox( event.getPos(), 40, 40 );
				}
			}
		}
	}
}




//-------------------------------------------------
// Key Down
//-------------------------------------------------
void Bounce::keyDown( KeyEvent event )
{
	switch( event.getChar() ){

	case 'a':
	case 'A':
		animFlag =! animFlag;
		break;

	case 'b':
	case 'B':
		toggleBallGen();
		break;

	case 'c':
	case 'C':
		console() << "Ball Count = " << balls.size() << endl;
		break;

	case 'd':
	case 'D':
		toggleDebugDraw();
		break;

	case 'f':
	case 'F':
		if(app::isFullScreen()){
			app::setFullScreen(false);
		} else {
			app::setFullScreen(true);
		}
		break;

	case 'm':
	case 'M':
		toggleMode();
		break;

	case 'p':
	case 'P':
		toggleColPicker();
		break;

	case 'q':
	case 'Q':
		deleteBalls();
		break;

	case 'r':
	case 'R':
		resetAll();
		break;

	case 's':
	case 'S':
		if(frameCntValue != 0)
			frameCntValue = 0;
		else
			frameCntValue = 60;
		break;

#ifdef VERBOSE
		if(colPickerFlag) console() << "\nColour pick mode ON" << endl;
		else console() << "\nColour pick mode OFF" << endl;
#endif

		break;

	case 'o':
	case 'O':
		opencvFlag =! opencvFlag;
		if(opencvFlag){

#ifdef VERBOSE
			console() << "Computer Vision turned on" << endl;
#endif

			createCvWindows();
		}
		else{

#ifdef VERBOSE
			console() << "Computer Vision turned off" << endl;
#endif

			destroyAllWindows();
		}
		break;

	default: break;
	}

	if(event.getCode() == KeyEvent::KEY_SPACE){
		pauseFlag =! pauseFlag;
	}
	if(event.getCode() == KeyEvent::KEY_LCTRL){
	}
	if(event.getCode() == KeyEvent::KEY_ESCAPE){
		quit();
	}
}


//-------------------------------------------------
// Is ball off screen
//-------------------------------------------------
bool Bounce::isBallOffScreen(const b2Body &ball)
{
	float32 r = ball.GetFixtureList()->GetShape()->m_radius;
	b2Vec2 point = ball.GetPosition();
	if(   point.x < (0 - r) || point.x > (w + r)  || point.y > (h + r)  ) return true;
	return false;
}


//-------------------------------------------------
// Update Colours
//-------------------------------------------------
void Bounce::updateColours(){
	colourBoxHSV = colourCV.get(CM_HSV);
	colourBoxHSV.x *= 179;
	colourBoxHSV.y *= 255;
	colourBoxHSV.z *= 255;
	hueValueBX = (int)colourBoxHSV.x;

	colourBoxHSV = colourBG.get(CM_HSV);
	colourBoxHSV.x *= 179;
	colourBoxHSV.y *= 255;
	colourBoxHSV.z *= 255;
	hueValueBG = (int)colourBoxHSV.x;
}


//-------------------------------------------------
// Reset All
//-------------------------------------------------
void Bounce::resetAll(){
	ballSize				= 10;
	colourBG				= Color8u(255, 0, 255); //Pink
	colourBalls				= Color8u(255, 255, 255);
	//colourCV				= Color8u(163, 246, 255); Light Blue
	colourCV				= Color8u(215, 125, 0); //Orange
	GravValueY				= 8;
	GravValueX				= 0;
	densityValue			= 40;
	frictionValue			= 10;
	bounceValue				= 90;
	frameCntValue			= 60;
	frameCnt				= 0;
	ballCreationCntValue	= 10;
	ballCreationCnt			= 0;
	ballsPerIteration		= 5;
	hueValueBX				= 28;
	satValueMin				= 100;
	valValueMin				= 100;
	satValueMax				= 255;
	valValueMax				= 255;
	//###############################
	deleteBalls();
	deleteBoxes();
	updateColours();

#ifdef VERBOSE
	console() << "" << endl;
#endif
}


//-------------------------------------------------
// Create CV Windows
//-------------------------------------------------
void Bounce::createCvWindows(){
	namedWindow("Original", CV_WINDOW_AUTOSIZE);
	namedWindow("CV", CV_WINDOW_AUTOSIZE);
	namedWindow("HSV", CV_GUI_NORMAL | CV_WINDOW_AUTOSIZE);
	createButton("Draw Debug Data", bcb, this);
	createTrackbar("H.Range BX", "", &hueRangeBX, 30);
	createTrackbar("H.Range BG", "", &hueRangeBG, 30);
	createTrackbar("Sat-Min", "", &satValueMin, 255);
	createTrackbar("Sat-Max", "", &satValueMax, 255);
	createTrackbar("Light-Min", "", &valValueMin, 255);
	createTrackbar("Light-Max", "", &valValueMax, 255);
	createButton("Ball Generator", bcbGenerator, this);
	createButton("Mode", bcbMode, this);
	createButton("Del Balls", bcbDelBalls, this);
	waitKey(5);
}



/****************************************************************************************************************************
*
*	Callback Functions (Buttons, etc.)
*
****************************************************************************************************************************/


void Bounce::buttonUpBalls(){
	for(int i = 0; i < (int)balls.size(); i++){
		balls.at(i)->GetFixtureList()->GetShape()->m_radius = ballSize;
	}
	Bounce::resize();
}

void Bounce::buttonUpCol(){
	updateColours();
}

void Bounce::buttonUpPhys(){
	mWorld->SetGravity(b2Vec2( (float)GravValueX, (float)GravValueY ));
}

void Bounce::buttonReset(){
	resetAll();
	resize();
	updateColours();
}

void Bounce::deleteBalls(){
	for (int i = 0; i < (int)balls.size(); i++){
		mWorld->DestroyBody(balls.at(i));
	}
	balls.clear();
#ifdef VERBOSE
	console() << "All Balls deleted" << endl;
#endif
}

void Bounce::deleteBoxes(){
	for(int i = (int)mBoxes.size() - 1; i >= 0; i--){
		mWorld->DestroyBody(mBoxes.at(i));
		mBoxes.erase(mBoxes.begin() + i);
	}
	mBoxes.clear();
}

void Bounce::bcb(int state, void* userdata){
	Bounce *self = static_cast<Bounce*>(userdata);
	self->toggleDebugDraw();
}

void Bounce::bcbGenerator(int state, void* userdata){
	Bounce *self = static_cast<Bounce*>(userdata);
	self->toggleBallGen();
}

void Bounce::bcbMode(int state, void* userdata){
	Bounce *self = static_cast<Bounce*>(userdata);
	self->toggleMode();
}

void Bounce::bcbDelBalls(int state, void* userdata){
	Bounce *self = static_cast<Bounce*>(userdata);
	self->deleteBalls();
}



/****************************************************************************************************************************
*
*	Toggle Functions
*
****************************************************************************************************************************/

void Bounce::toggleMode(){
	Bounce::ballGeneratorMode =! Bounce::ballGeneratorMode;
#ifdef VERBOSE
	console() << "Ball Mode: " << (Bounce::ballGeneratorMode ? "Waterfall" : "Rain") << endl;
#endif
}

void Bounce::toggleBallGen(){
	Bounce::generateBallsFlag =! Bounce::generateBallsFlag;
#ifdef VERBOSE
	console() << "Ball Generator: " << (Bounce::generateBallsFlag ? "On" : "Off") << endl;
#endif
}

void Bounce::toggleDebugDraw(){
	Bounce::debugFlag =! Bounce::debugFlag;
#ifdef VERBOSE
	console() << "Debug Draw: " << (Bounce::debugFlag ? "On" : "Off") << endl;
#endif
}

void Bounce::toggleColPicker(){
	Bounce::colPickerFlag =! Bounce::colPickerFlag;
#ifdef VERBOSE
	console() << "Colour Picker: " << (Bounce::colPickerFlag ? "On - Click image to select OpenCV Colour" : "Off") << endl;
#endif
}



CINDER_APP_NATIVE( Bounce, RendererGl )