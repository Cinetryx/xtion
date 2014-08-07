/***********************************************************/
/****	ShakaijissouProject PROGRAM			            ****/
/****	2014 OyNCT KawamuraKen Kinect Term		        ****/
/****	Designed On 2014.jul.15 By Tokunn		        ****/
/****	Update On 2014.Jul.21 By Tokunn add comment	    ****/
/***********************************************************/


#include <math.h>
#include <iostream>
#include <string.h>
#include <stdexcept>
#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


enum Pose {NONE, MAJOKO, KAIDAN, BRUNA, NEKO, KING};


class NiteApp
{

    public:


        void initialize()
        {
            userTracker.create();

	        openni::Status ret = device.open(openni::ANY_DEVICE);
	        if (ret != openni::STATUS_OK) {
                throw std::runtime_error( "openni::Device::open() failed." );
	        }

	        colorStream.create(device, openni::SENSOR_COLOR);
	        changeResolution(colorStream);

	        colorStream.start();
        }


        void update()
        {
            openni::VideoFrameRef colorFrame;
            nite::UserTrackerFrameRef userFrame;

            userTracker.readFrame(&userFrame);
            colorStream.readFrame(&colorFrame);

            depthImage = showUser(userFrame);
            colorImage = showColorStream(colorFrame);

            const nite::Array<nite::UserData>& users = userFrame.getUsers();

            for (int i = 0; i < users.getSize(); ++i) {
                const nite::UserData& user = users[i];

                if (user.isNew()) {
                    userTracker.startSkeletonTracking(user.getId());
                }
                else if (!user.isLost()) {
                    showSkeleton(depthImage, userTracker, user);
                }
            }

            cv::imshow("Skeleton", depthImage);
            cv::imshow("ColorStream", colorImage);
        }



    private:
        double to_deg( double r ) {
            return r * 180.0 / (atan(1.0) * 4.0);
        }

        void changeResolution(openni::VideoStream& stream)
        {
            openni::VideoMode mode = stream.getVideoMode();
            mode.setResolution(640, 480);
            mode.setFps(30);
            stream.setVideoMode(mode);
        }


        cv::Mat showUser(nite::UserTrackerFrameRef& userFrame)
        {
            static const cv::Scalar colors[] = {
                cv::Scalar(0, 0, 1),
                cv::Scalar(1, 0, 0),
                cv::Scalar(0, 1, 0),
                cv::Scalar(1, 1, 0),
                cv::Scalar(1, 0, 1),
                cv::Scalar(0, 1, 1),
                cv::Scalar(0.5, 0, 0),
                cv::Scalar(0, 0.5, 0),
                cv::Scalar(0, 0, 0.5),
                cv::Scalar(0.5, 0.5, 0),
            };

            cv::Mat depthImage;

            openni::VideoFrameRef depthFrame = userFrame.getDepthFrame();
            if (depthFrame.isValid()) {
                depthImage = cv::Mat(depthFrame.getHeight(),
                                     depthFrame.getWidth(),
                                     CV_8UC4);

                openni::DepthPixel* depth = (openni::DepthPixel*)depthFrame.getData();
                const nite::UserId* pLabels = userFrame.getUserMap().getPixels();

                for (int i = 0; i < (depthFrame.getDataSize() / sizeof(openni::DepthPixel)); ++i) {
                    int index = i * 4;

                    uchar* data = &depthImage.data[index];
                    if (pLabels[i] != 0) {
                        data[0] *= colors[pLabels[i]][0];
                        data[1] *= colors[pLabels[i]][1];
                        data[2] *= colors[pLabels[i]][2];
                    }
                    else {
                        int gray = ~((depth[i] * 255) / 10000);
                        data[0] = gray;
                        data[1] = gray;
                        data[2] = gray;
                    }
                }
            }
            return depthImage;
        }


        cv::Mat showColorStream(const openni::VideoFrameRef& colorFrame)
        {
            cv::Mat colorImage = cv::Mat(colorFrame.getHeight(),
                                         colorFrame.getWidth(),
                                         CV_8UC3, (unsigned char*)colorFrame.getData());

            cv::cvtColor(colorImage, colorImage, CV_RGB2BGR);

            return colorImage;
        }


        void showSkeleton(cv::Mat& depthImage, nite::UserTracker& userTracker, const nite::UserData& user)
        {
            const nite::Skeleton& skeelton = user.getSkeleton();
            if (skeelton.getState() != nite::SKELETON_TRACKED) {
                return;
            }

            float x, y;
            float x3[15], y3[15], z3[15];
            char joint_name[15][15] = {"HEAD_HEAD", "NECK_NECK", "LEFT_SHOULDER", "RIGT_SHOULDER", "LEFT_ELBOW", "RIGT_ELBOW", "LEFT_HAND", "RIGT_HAND",
                                       "TORSO_TORSO", "LEFT_HIP", "RIGT_HIP", "LEFT_KNEE", "RIGT_KNEE", "LEFT_FOOT", "RIGT_FOOT"};

            for (int j = 0; j <= 14; ++j) {
                const nite::SkeletonJoint& joint = skeelton.getJoint((nite::JointType)j);
                if (joint.getPositionConfidence() < 0.7f) {
                    continue;
                }

                const nite::Point3f& position = joint.getPosition();
                userTracker.convertJointCoordinatesToDepth(position.x, position.y, position.z, &x, &y);

                cv::circle(depthImage, cvPoint((int)x, (int)y), 5, cv::Scalar(0, 0, 255), -1);

                x3[j] = position.x;
                y3[j] = position.y;
                z3[j] = position.z;

                // std::cout << joint_name[j] << "\t\tX:" << (int)x3[j] << "\t\tY:" << (int)y3[j] << "\t\tZ:" << (int)z3[j]<< '\n';
            }

            const Pose checkedPose_buff = checkPose(x3, y3, z3, depthImage);
            const int checkedPose_buff_count = 0;

            Pose checkedPose = checkPose(x3, y3, z3, depthImage);

            if (checkedPose != NONE && checkedPose == checkedPose_buff) {      // checedPose continue
                checkedPose_buff_count += 1;
            }
            else {
                checkedPose_buff = checkedPose;
                checkedPose_buff_count = 0;
            }

        }


        Pose checkPose(float *x3, float *y3, float *z3, cv::Mat& depthImage)
        {
            char checkedPose_print[15] = "NONE";
            int accidental_deg = 20;
            Pose checkedPose = NONE;
            std::stringstream ss;

            int HEAD_HEAD = 0;
            int NECK_NECK = 1;
            int LEFT_SHOULDER = 2;
            int RIGT_SHOULDER = 3;
            int LEFT_ELBOW = 4;
            int RIGT_ELBOW = 5;
            int LEFT_HAND = 6;
            int RIGT_HAND = 7;
            int TORSO_TORSO = 8;
            int LEFT_HIP = 9;
            int RIGT_HIP = 10;
            int LEFT_KNEE = 11;
            int RIGT_KNEE = 12;
            int LEFT_FOOT = 13;
            int RIGT_FOOT = 14;

            /** Check Majoko **/
            if (y3[ HEAD_HEAD ] > y3[ LEFT_HAND ] && y3[ HEAD_HEAD ] < y3[ RIGT_HAND ]) {

                double theta_shordar_elbow = to_deg( atan2( (y3[ RIGT_ELBOW ] - y3[ RIGT_SHOULDER ]), (x3[ RIGT_ELBOW ] - x3[ RIGT_SHOULDER ]) ) );    // atan( shor-el ) to degree
                double theta_elbow_hand = to_deg( atan2( y3[ RIGT_HAND ] - y3[ RIGT_ELBOW ], x3[ RIGT_HAND ] - x3[ RIGT_ELBOW ] ) );       // atan( el-han ) to degree
                double theta_diff = fabs( theta_shordar_elbow - theta_elbow_hand );             // difference
                //std::cout << theta_shordar_elbow << "\t\t" << theta_elbow_hand << "\t\t" << theta_diff << '\n';

                if ( theta_diff < accidental_deg ) {
                    strcpy(checkedPose_print, "MAJOKO");
                    checkedPose = MAJOKO;
                    ehonnImage = cv::imread("./majoko.jpg");
                    imshow("Ehon", ehonnImage);
                }
            }

            /** Check Kaidan Restaurant **/
            if ((y3[ TORSO_TORSO ] < y3[ LEFT_HAND ]) && (y3[ TORSO_TORSO ] < y3[ RIGT_HAND ]) && (y3[ LEFT_SHOULDER ] > y3[ LEFT_HAND ]) && (y3[ RIGT_SHOULDER ] > y3[ RIGT_HAND ])) {
                strcpy(checkedPose_print, "KAIDAN");
                checkedPose = KAIDAN;
                ehonnImage = cv::imread("./kaidan.jpg");
                imshow("Ehon", ehonnImage);
            }

            /** Check Bruna **/
            if ((y3[ HEAD_HEAD ] < y3[ LEFT_HAND ]) && (y3[ HEAD_HEAD ] < y3[ RIGT_HAND ])) {
                strcpy(checkedPose_print, "BRUNA");
                checkedPose = BRUNA;
                ehonnImage = cv::imread("./bruna.jpg");
                imshow("Ehon", ehonnImage);
            }

            /** Check Neko Nezumi **/
            if ((y3[ LEFT_HAND ] > y3[ LEFT_SHOULDER ]) && (y3[ RIGT_HAND ] < y3[ RIGT_SHOULDER ])) {
                strcpy(checkedPose_print, "NEKO");
                checkedPose = NEKO;
                ehonnImage = cv::imread("./neko.jpg");
                imshow("Ehon", ehonnImage);
            }

            /** Check King **/
            /*if ((y3[ LEFT_SHOULDER ] > y3[ LEFT_HAND ]) && (y3[ RIGT_SHOULDER ] > y3[ RIGT_HAND ])) {

                double theta_left_elbow1 = to_deg( atan2( (y3[ LEFT_SHOULDER - LEFT_ELBOW ]), (x3[ LEFT_SHOULDER - LEFT_ELBOW ]) ) );    //atan( shor-el ) to degree
                double theta_left_elbow2 = to_deg( atan2( (y3[ LEFT_ELBOW - LEFT_HAND ]), (x3[ LEFT_HAND - LEFT_ELBOW ]) ) );   //atan( el-han ), atan( han-el ) to degree
                double theta_left = fabs(theta_left_elbow1) + fabs(theta_left_elbow2);

                double theta_rigt_elbow1 = to_deg( atan2( (y3[ RIGT_SHOULDER - RIGT_ELBOW ]), (x3[ RIGT_SHOULDER - RIGT_ELBOW ]) ) );   //atan( shor-el ) to degree
                double theta_rigt_elbow2 = to_deg( atan2( (y3[ RIGT_ELBOW - RIGT_HAND ]), (x3[ RIGT_HAND - RIGT_ELBOW ]) ) );   //atan( el-han ), atan( han-el ) to degree
                double theta_rigt = fabs(theta_rigt_elbow1) + fabs(theta_rigt_elbow2);

                std::cout << theta_left << "\t\t" << theta_rigt << '\n';
            }*/

            /** Check NONE **/
            if( strcmp(checkedPose_print, "NONE") == 0 ) {
                cv::destroyWindow("Ehon");
            }
            
            ss << "Pose:" << checkedPose_print;
            cv::putText(depthImage, ss.str(), cv::Point(0, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
            return checkedPose;
        }




    private:
        
        nite::UserTracker userTracker;
        openni::VideoStream colorStream;

        cv::Mat depthImage;
        cv::Mat colorImage;
        cv::Mat ehonnImage;

        openni::Device device;
};




int main(int argc, const char * argv[])
{
    try {
        openni::OpenNI::initialize();
        nite::NiTE::initialize();

        NiteApp app;
        app.initialize();

        while (true) {
            app.update();

            int key = cv::waitKey(10);
            if (key == 'q') {
                break;
            }
        }
    }

    catch (std::exception&) {
        std::cout << openni::OpenNI::getExtendedError() << std::endl;
    }

    return 0;
}
