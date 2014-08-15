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
        void drawBox( const nite::UserData& user );


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
    cv::imshow( "Depth Frame", depthImage );        // #=# DEBUG #=#

    cv::imshow( "Debug Frame", debugImage );
}


/*---- Convert OpenNI format to OpenCv format ----*/
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
    debugImage = colorImage;
    for ( int i = 0; i < users.getSize(); ++i ) {
        const nite::UserData& user = users[i];
        drawBox( user );
    }
    nite::UserId frontUserId = checkFrontUser( users );
    return debugImage;
}


/*---- Check Front User ----*/
nite::UserId Xtion::checkFrontUser( const nite::Array<nite::UserData>& users )
{
    float distanceUser[ users.getSize() ];
    nite::UserId distanceUserId[ users.getSize() ];

    for ( int i = 0; i < users.getSize(); ++i ) {
        const nite::UserData& user = users[i];
        distanceUser[i] = user.getCenterOfMass().z;
        distanceUserId[i] = user.getId();
    }
    for ( int i = 1; i < users.getSize(); ++i ) {
        if ( distanceUser[0] > distanceUser[i] ) {
            float temp = distanceUser[0];
            distanceUser[0] = distanceUser[i];
            distanceUser[i] = temp;

            nite::UserId temp_Id = distanceUserId[0];
            distanceUserId[0] = distanceUserId[i];
            distanceUserId[i] = temp_Id;
        }
    }
    return distanceUserId[0];
}


/*---- Draw Bounding Box ----*/
void Xtion::drawBox( const nite::UserData& user )
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

    cv::line( debugImage, LEFT_HIGH, RIGT_HIGH, cv::Scalar(0,255,0) );  // LOW
    cv::line( debugImage, LEFT_LOW , RIGT_LOW , cv::Scalar(0,255,0) );  // HIGH
    cv::line( debugImage, LEFT_HIGH, LEFT_LOW , cv::Scalar(0,255,0) );  // LEFT
    cv::line( debugImage, RIGT_HIGH, RIGT_LOW , cv::Scalar(0,255,0) );  // RIGHT
}


/*---- Show Users on Depth ----*/
cv::Mat Xtion::showUsersStream( nite::UserTrackerFrameRef& userFrame )      // #=# DEBUG #=#
{
    static const cv::Scalar colors[] = {
        cv::Scalar( 1, 0, 0 ),
        cv::Scalar( 0, 1, 0 ),
        cv::Scalar( 0, 0, 1 ),
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
                data[0] *= colors[0][0];
                data[1] *= colors[0][1];
                data[2] *= colors[0][2];
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
