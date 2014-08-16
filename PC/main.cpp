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



/******* Xtion Class *******/
class Xtion
{
    public:
        Xtion();
        void update();
    private:
        cv::Mat convColorStream( openni::VideoFrameRef& colorFrame );
        cv::Mat makeDebugStream( nite::UserTrackerFrameRef& userFrame );
        cv::Mat showUsersStream( nite::UserTrackerFrameRef& userFrame );    // #=# DEBUG #=#
        nite::UserId checkFrontUser( const nite::Array<nite::UserData>& users );
        void drawBox( const nite::UserData& user, int flag );
        void changeResolution( openni::VideoStream& stream );
        void trackingUser( const nite::UserData& user );
        void printWindow();
        void showSkeleton( cv::Mat& depthImage, nite::UserTracker& userTracker, const nite::UserData& user );


    private:
        openni::Device device;  // Using device
        openni::VideoStream colorStream;
        nite::UserTracker userTracker;
        cv::Mat colorImage;     // ColorStream image ( colorImage )
        cv::Mat debugImage;     // Debug Print image ( debugImage )
        cv::Mat depthImage;     // Depth Print image ( depthImage )     #=# DEBUG #=#
};


/*---- Initialize ----*/
Xtion::Xtion()
{
    openni::Status ret = device.open( openni::ANY_DEVICE );
    colorStream.create( device, openni::SENSOR_COLOR );
    changeResolution( colorStream );
    colorStream.start();

    userTracker.create();
}


/*---- Update frame ----*/
void Xtion::update()
{
    openni::VideoFrameRef colorFrame;               // will in a ColorStream ( colorFrame )
    colorStream.readFrame( &colorFrame );           // Read Frame
    colorImage = convColorStream( colorFrame );     // Convert colorStream

    nite::UserTrackerFrameRef userFrame;            // Will in a DebugStream ( userFrame )
    userTracker.readFrame( &userFrame );            // Read Frame
    debugImage = makeDebugStream( userFrame );      // Make debugStream

    depthImage = showUsersStream( userFrame );      // #=# DEBUG #=#

    printWindow();

    //cv::imshow( "Depth Frame", depthImage );        // #=# DEBUG #=#

}


/*---- Print Window ----*/
void Xtion::printWindow()
{
    cv::Mat backImage = cv::imread( "Images/Background.png" );
    cv::Mat baseimage( cv::Size( 1366, 768 ), CV_8UC3 );

    cv::Mat RoiBack( baseimage, cv::Rect( 0, 0, backImage.cols, backImage.rows ) );
    cv::Mat RoiDebug( baseimage, cv::Rect( 985, 470, debugImage.cols, debugImage.rows ) );

    backImage.copyTo( RoiBack );
    debugImage.copyTo( RoiDebug );

    const char windowName[] = "Etoshan  -NITOyC-  by Tokunn";
    cv::namedWindow( windowName, CV_WINDOW_AUTOSIZE );
    cv::imshow( windowName, baseimage );
    cvMoveWindow( windowName, 80, 50 );
}


/*---- Convert OpenNI format to OpenCV format ----*/
cv::Mat Xtion::convColorStream( openni::VideoFrameRef& colorFrame )
{
    colorImage = cv::Mat( colorFrame.getHeight(),
                                  colorFrame.getWidth(),
                                  CV_8UC3,
                                  (unsigned char*)colorFrame.getData() );
    cv::cvtColor( colorImage, colorImage, CV_RGB2BGR );
    return colorImage;
}


/*---- Make DebugStream ----*/
cv::Mat Xtion::makeDebugStream( nite::UserTrackerFrameRef& userFrame )
{
    const nite::Array<nite::UserData>& users = userFrame.getUsers();

    nite::UserId frontUserId = checkFrontUser( users );     // Get front user
    debugImage = colorImage;

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
    return debugImage;
}


/*---- Tracking Front User ----*/
void Xtion::trackingUser( const nite::UserData& user )
{
    if ( user.isNew() ) {
        userTracker.startSkeletonTracking( user.getId() );
    }
    else if ( !user.isLost() ) {
        showSkeleton( depthImage, userTracker, user );
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
        //color = cv::Scalar( 0, 0, 255 );
    }

    cv::line( debugImage, LEFT_HIGH, RIGT_HIGH, color, 3 );  // LOW
    cv::line( debugImage, LEFT_LOW , RIGT_LOW , color, 3 );  // HIGH
    cv::line( debugImage, LEFT_HIGH, LEFT_LOW , color, 3 );  // LEFT
    cv::line( debugImage, RIGT_HIGH, RIGT_LOW , color, 3 );  // RIGHT
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
void Xtion::showSkeleton( cv::Mat& depthImage, nite::UserTracker& userTracker, const nite::UserData& user )
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
        cv::circle( debugImage, cvPoint( (int)x, (int)y ), 5, cv::Scalar( 0, 0, 255 ), -1 );
    }
}


/*---- Show Users on Depth ----*/
cv::Mat Xtion::showUsersStream( nite::UserTrackerFrameRef& userFrame )      // #=# DEBUG #=#
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
    return depthImage;
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
            int key = cv::waitKey( 10 );
            if ( key == 27 ) {
                break;
            }
        }
    }
    catch ( std::exception& ) {
        std::cout << openni::OpenNI::getExtendedError() << std::endl;
    }

    return 0;
}
