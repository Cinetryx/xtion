/***********************************************************/
/****	ShakaijissouProject PROGRAM			            ****/
/****	2014 OyNCT KawamuraKen Kinect Term		        ****/
/****	Designed On 2014.jul.15 By Tokunn		        ****/
/****	Update On 2014.Oct.14 By Tokunn edit source	    ****/
/***********************************************************/


// Standard
#include <iostream>
#include <stdexcept>
// Math and String
#include <math.h>
#include <string.h>
#include <cstdlib>
#include <sstream>
#include <iomanip>
// Xtion
#include <NiTE.h>
#include <OpenNI.h>
// Network
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



enum Pose{ NONE=0, MAJOKO=1, OBAKE=2, KAIDAN=3, NEKO=4, KING=5, BRUNA=6 };


/******* Xtion Class *******/
class Xtion
{
    public:
        Xtion();
        void update();
        bool poseCheckFlag;
    private:
        void convColorStream( openni::VideoFrameRef& colorFrame );
        void makeDebugStream( nite::UserTrackerFrameRef& userFrame );
        void showUsersStream( nite::UserTrackerFrameRef& userFrame );    // #=# DEBUG #=#
        nite::UserId checkFrontUser( const nite::Array<nite::UserData>& users );
        void drawBox( const nite::UserData& user, int flag );
        void changeResolution( openni::VideoStream& stream );
        void trackingUser( const nite::UserData& user );
        void showSkeleton( nite::UserTracker& userTracker, const nite::UserData& user );
        void putDebugText( const nite::Array<nite::UserData>& users );
        Pose checkPose( const nite::Skeleton& skeelton );
        void printWindow();
    private:
        openni::Device device;  // Using device
        openni::VideoStream colorStream;
        nite::UserTracker userTracker;
        cv::Mat colorImage;     // ColorStream image ( colorImage )
        cv::Mat cameraImage;     // Camera Print image ( cameraImage )
        cv::Mat debugImage;     // Debug Print image ( debugImage )
        cv::Mat debugBrack;     // Debug print image Brack ( debugBrack )
        cv::Mat direcImage;     // Direction Print image ( direcImage )
        cv::Mat timeImage;      // Time Print image ( timeImage )
        cv::Mat depthImage;     // Depth Print image ( depthImage )     #=# DEBUG #=#
        Pose beforePose;
        Pose afterimagePose;
        Pose pose;
        int countPose;
        int beforeRecoNum;
};


/*---- Initialize ----*/
Xtion::Xtion()
{
    openni::Status ret = device.open( openni::ANY_DEVICE );
    colorStream.create( device, openni::SENSOR_COLOR );
    changeResolution( colorStream );
    colorStream.start();

    userTracker.create();

    debugImage = cv::Mat( cv::Size( 300, 100 ), CV_8UC3 );
    debugBrack = cv::imread( "Images/DEBUG.png" );
    timeImage = cv::Mat( cv::Size( 230, 30 ), CV_8UC3 );

    beforeRecoNum = 20;
    countPose = 0;
    pose = NONE;
    beforePose = NONE;
    afterimagePose = NONE;
    poseCheckFlag = true;
}


/*---- Update frame ----*/
void Xtion::update()
{
    openni::VideoFrameRef colorFrame;               // will in a ColorStream ( colorFrame )
    colorStream.readFrame( &colorFrame );           // Read Frame
    convColorStream( colorFrame );     // Convert colorStream

    nite::UserTrackerFrameRef userFrame;            // Will in a DebugStream ( userFrame )
    userTracker.readFrame( &userFrame );            // Read Frame
    makeDebugStream( userFrame );      // Make debugStream

    //showUsersStream( userFrame );      // #=# DEBUG #=#

    printWindow();
}


/*---- Print Window ----*/
void Xtion::printWindow()
{
    cv::Mat baseImage = cv::Mat( cv::Size( 1366, 768 ), CV_8UC3 );
    cv::Mat backImage;
    if ( countPose >= 60 ) {
        backImage = cv::imread( "Images/Background3.png" );
    }
    else {
        backImage = cv::imread( "Images/Background.png" );
    }

    std::ostringstream timeName;    // Make file name
    timeName << "Images/Time/" << countPose << ".png";
    std::string timeImageName = timeName.str();

    int len = timeImageName.length();   // Convert string to char&
    char* fname = new char[len+1];
    memcpy( fname, timeImageName.c_str(), len+1 );

    timeImage = cv::imread( fname );

    cv::Mat RoiBack( baseImage, cv::Rect( 0, 0, backImage.cols, backImage.rows ) );
    cv::Mat RoiTime( baseImage, cv::Rect( 1065, 295, timeImage.cols, timeImage.rows ) );
    cv::Mat RoiDebug( baseImage, cv::Rect( 980, 110, debugImage.cols, debugImage.rows ) );
    cv::Mat RoiDirec( baseImage, cv::Rect( 70, 65, direcImage.cols, direcImage.rows ) );
    cv::Mat RoiCamera( baseImage, cv::Rect( 980, 460, cameraImage.cols, cameraImage.rows ) );

    backImage.copyTo( RoiBack );
    timeImage.copyTo( RoiTime );
    debugImage.copyTo( RoiDebug );
    direcImage.copyTo( RoiDirec );
    cameraImage.copyTo( RoiCamera );

    const char windowName[] = "Etoshan  -NITOyC-  by Tokunn";
    cv::namedWindow( windowName, CV_WINDOW_AUTOSIZE );
    cv::imshow( windowName, baseImage );
    cvMoveWindow( windowName, 80, 50 );

    //cv::imshow( "Depth Frame", depthImage );        // #=# DEBUG #=#
}


/*---- Convert OpenNI format to OpenCV format ----*/
void Xtion::convColorStream( openni::VideoFrameRef& colorFrame )
{
    colorImage = cv::Mat( colorFrame.getHeight(),
                                  colorFrame.getWidth(),
                                  CV_8UC3,
                                  (unsigned char*)colorFrame.getData() );
    cv::cvtColor( colorImage, colorImage, CV_RGB2BGR );
}


/*---- Make DebugStream ----*/
void Xtion::makeDebugStream( nite::UserTrackerFrameRef& userFrame )
{
    const nite::Array<nite::UserData>& users = userFrame.getUsers();

    nite::UserId frontUserId = checkFrontUser( users );     // Get front user
    cameraImage = colorImage;

    direcImage = cv::imread( "Images/Default.png" );

    for ( int i = 0; i < users.getSize(); ++i ) {
        const nite::UserData& user = users[i];
        nite::UserId userId = user.getId();
        if ( userId == frontUserId ) {
            drawBox( user, 1 );     // Draw box of front user
            trackingUser( user );   // Tracking front user
        }
        else {
            drawBox( user, 0 );     // Draw box of other user
        }
    }

    if ( users.getSize() == 0 ) {
    }
    putDebugText( users );
}


/*---- Tracking Front User ----*/
void Xtion::trackingUser( const nite::UserData& user )
{
    if ( user.isNew() ) {
        userTracker.startSkeletonTracking( user.getId() );
    }
    else if ( !user.isLost() ) {
        showSkeleton( userTracker, user );
    }
}


/*---- Check Front User ----*/
nite::UserId Xtion::checkFrontUser( const nite::Array<nite::UserData>& users )
{
    float distanceUser[ 20 ];
    nite::UserId distanceUserId[ 20 ];
    distanceUserId[0] = 0;  // Initialize

    for ( int i = 0; i < users.getSize(); ++i ) {
        const nite::UserData& user = users[i];
        distanceUser[i] = user.getCenterOfMass().z;
        distanceUserId[i] = user.getId();
    }
    for ( int i = 0; i < users.getSize(); ++i ) {
        for ( int j = i + 1; j < users.getSize(); ++j ) {
            if ( distanceUser[i] > distanceUser[j] ) {
                float temp = distanceUser[i];
                distanceUser[i] = distanceUser[j];
                distanceUser[j] = temp;

                nite::UserId temp_Id = distanceUserId[i];
                distanceUserId[i] = distanceUserId[j];
                distanceUserId[j] = temp_Id;
            }
        }
    }
    /*
    for ( int i = 0; i < users.getSize(); ++i ) {       // #=# DEBUGSTART #=#
        std::cout << distanceUserId[i] << '\t';
    }
    std::cout << "\t\t\tFrontUser: " << distanceUserId[0] << '\n';

    static int debug = 0;
    if ( debug == 20 ) {
        std::cout << "Check\n";
        debug = 0;
    }
    debug++;                    // #=# DEBUGEND #=#
    */

    return distanceUserId[0];
}


/*---- Put Debug Text ----*/
void Xtion::putDebugText( const nite::Array<nite::UserData>& users )
{
    char poseName[7][7] = { "NONE", "MAJOKO", "OBAKE", "KAIDAN", "NEKO", "KING", "BRUNA" };

    if ( beforeRecoNum != users.getSize() || pose != afterimagePose ) {
        //debugImage = cv::Mat( cv::Size( 300, 100 ), CV_8UC3 );      // Clean image
        debugImage = debugBrack;
    }
    else {
        std::stringstream recoNum, posename;
        recoNum << "RecoNum: " << users.getSize();
        posename << "Pose: " << poseName[ pose ];
        cv::putText( debugImage, recoNum.str(), cv::Point( 5, 30 ), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 255, 0 ), 2 );
        cv::putText( debugImage, posename.str(), cv::Point( 5, 85 ), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 255, 0 ), 2 );
        std::cout << recoNum.str() << '\t' << posename.str() << '\n';   // #=# DEBUG #=#
    }
    beforeRecoNum = users.getSize();
}


/*---- Draw Bounding Box ----*/
void Xtion::drawBox( const nite::UserData& user, int flag )
{
    const nite::BoundingBox& box = user.getBoundingBox();

    float RHx = box.max.x, RHy = box.max.y;     // Right High
    cv::Point RIGT_HIGH = cvPoint( (int)RHx, (int)RHy );
    float RLx = box.max.x, RLy = box.min.y;     // Right Low
    cv::Point RIGT_LOW = cvPoint( (int)RLx, (int)RLy );
    float LHx = box.min.x, LHy = box.max.y;     // Left High
    cv::Point LEFT_HIGH = cvPoint( (int)LHx, (int)LHy );
    float LLx = box.min.x, LLy = box.min.y;     // Left Low
    cv::Point LEFT_LOW = cvPoint( (int)LLx, (int)LLy );

    cv::Scalar color( 0, 255, 0 );
    if ( flag ) {
        color = cv::Scalar( 255, 255, 0 );
    }

    cv::line( cameraImage, LEFT_HIGH, RIGT_HIGH, color, 3 );  // LOW
    cv::line( cameraImage, LEFT_LOW , RIGT_LOW , color, 3 );  // HIGH
    cv::line( cameraImage, LEFT_HIGH, LEFT_LOW , color, 3 );  // LEFT
    cv::line( cameraImage, RIGT_HIGH, RIGT_LOW , color, 3 );  // RIGHT
}


/*---- Change Resolution ----*/
void Xtion::changeResolution( openni::VideoStream& stream )     // Don't have to
{
    openni::VideoMode mode = stream.getVideoMode();
    mode.setResolution( 320, 240 );
    mode.setFps( 30 );
    stream.setVideoMode( mode );
}


/*---- Show User Skeleton ----*/
void Xtion::showSkeleton( nite::UserTracker& userTracker, const nite::UserData& user )
{
    const nite::Skeleton& skeelton = user.getSkeleton();
    if ( skeelton.getState() != nite::SKELETON_TRACKED ) {
        return;
    }

    for ( int j = 0; j <= 14; ++j ) {
        const nite::SkeletonJoint& joint = skeelton.getJoint( (nite::JointType)j );
        if ( joint.getPositionConfidence() < 0.7f ) {
            continue;
        }
        const nite::Point3f& position = joint.getPosition();
        float x = 0, y = 0;
        userTracker.convertJointCoordinatesToDepth( position.x, position.y, position.z, &x, &y );
        cv::circle( cameraImage, cvPoint( (int)x, (int)y ), 5, cv::Scalar( 0, 128, 255 ), -1 );
    }

    pose = checkPose( skeelton );   // Check Pose
    afterimagePose = beforePose;
    if ( poseCheckFlag ) {
        if ( pose != NONE && pose == beforePose ) {
            countPose++;
        }
        else {
            countPose = 0;
            beforePose = pose;
        }
        if ( countPose > 60 ) {
            countPose = 0;
            beforePose = NONE;
        }
        else if ( countPose == 60 ) {
            poseCheckFlag = false;
        }
    }
}


/*---- Check Pose and show Book image ----*/
Pose Xtion::checkPose( const nite::Skeleton& skeelton )
{
    Pose checkPose = NONE;

    const nite::Point3f& joint_head = ( skeelton.getJoint( nite::JOINT_HEAD ) ).getPosition();
    const nite::Point3f& joint_neck = ( skeelton.getJoint( nite::JOINT_NECK ) ).getPosition();
    const nite::Point3f& joint_torso = ( skeelton.getJoint( nite::JOINT_TORSO ) ).getPosition();
    const nite::Point3f& joint_left_hand = ( skeelton.getJoint( nite::JOINT_LEFT_HAND ) ).getPosition();
    const nite::Point3f& joint_rigt_hand = ( skeelton.getJoint( nite::JOINT_RIGHT_HAND ) ).getPosition();

    /*'' Check NEKO ''*/
    if ( joint_head.y < joint_left_hand.y && joint_rigt_hand.y < joint_torso.y ) {
        checkPose = NEKO;
        direcImage = cv::imread( "Images/NEKO.png" );
    }
    /*'' Check MAJOKO ''*/
    if ( joint_head.y < joint_rigt_hand.y && joint_left_hand.y < joint_torso.y ) {
        checkPose = MAJOKO;
        direcImage = cv::imread( "Images/MAJOKO.png" );
    }
    /*'' Check KAIDAN ''*/
    if ( ( ( joint_left_hand.y < joint_neck.y ) && ( joint_rigt_hand.y < joint_neck.y ) ) && ( ( joint_torso.y < joint_left_hand.y ) && ( joint_torso.y < joint_rigt_hand.y ) ) ) {
        checkPose = KAIDAN;
        direcImage = cv::imread( "Images/KAIDAN.png" );
    }
    /*'' check BRUNA ''*/
    if ( joint_left_hand.y > joint_head.y && joint_rigt_hand.y > joint_head.y ) {
        checkPose = BRUNA;
        direcImage = cv::imread( "Images/BRUNA.png" );
    }
    if ( direcImage.empty() ) {     // Check Error
        std::cout << "Nofile\n";
    }
    return checkPose;
}


/*---- Show Users on Depth ----*/
void Xtion::showUsersStream( nite::UserTrackerFrameRef& userFrame )      // #=# DEBUG #=#
{
    static const cv::Scalar colors[] = {
        cv::Scalar( 1, 0, 0 ),
        cv::Scalar( 0, 1, 0 ),
        cv::Scalar( 0, 0, 1 ),
        cv::Scalar( 0, 1, 1 ),
        cv::Scalar( 1, 1, 0 ),
        cv::Scalar( 1, 0, 1 ),
    };

    openni::VideoFrameRef depthFrame = userFrame.getDepthFrame();
    if ( depthFrame.isValid() ) {
        depthImage = cv::Mat( depthFrame.getHeight(),
                              depthFrame.getWidth(),
                              CV_8UC4 );
        openni::DepthPixel* depth = (openni::DepthPixel*)depthFrame.getData();
        const nite::UserId* pLabels = userFrame.getUserMap().getPixels();

        for ( int i = 0; i < (depthFrame.getDataSize() / sizeof( openni::DepthPixel ) ); ++i ) {
            int index = i * 4;
            uchar* data = &depthImage.data[index];
            if ( pLabels[i] != 0 ) {
                data[0] *= colors[pLabels[i]][0];
                data[1] *= colors[pLabels[i]][1];
                data[2] *= colors[pLabels[i]][2];
            }
            else {
                int gray = ~( ( depth[i] * 255 ) / 10000 );
                data[0] = gray;
                data[1] = gray;
                data[2] = gray;
            }
        }
    }
}


/*===== MAIN LOOP =====*/
int main()
{
    try {
        std::cout << "ChePo 1\n";   // #=# DEBUG #=#
        openni::OpenNI::initialize();
        nite::NiTE::initialize();
        std::cout << "ChePo 2\n";   // #=# DEUBG #=#
        Xtion app;
        std::cout << "ChePo 3\n";   // #=# DEBUG #=#
        
        while (true) {
            app.update();
            int Key = cv::waitKey( 10 );
            if ( Key == 32 ) {      // Space Key
                app.poseCheckFlag = true;
            }
            else if ( Key == 27 ) { // Escape Key
                break;
            }
        }

    }
    catch ( std::exception& ) {
        std::cout << openni::OpenNI::getExtendedError() << std::endl;
    }
    return 0;
}
