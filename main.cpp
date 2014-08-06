/***********************************************************/
/****	ShakaijissouProject PROGRAM			            ****/
/****	2014 OyNCT KawamuraKen Kinect Term		        ****/
/****	Designed On 2014.jul.15 By Tokunn		        ****/
/****	Update On 2014.Jul.21 By Tokunn add comment	    ****/
/***********************************************************/


#include <iostream>
#include <stdexcept>
#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


enum Pose {NONE, OUT, MOUNTAIN, RIVER, OBAKE};


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

                std::cout << joint_name[j] << "\t\tX:" << (int)x3[j] << "\t\tY:" << (int)y3[j] << "\t\tZ:" << (int)z3[j]<< '\n';
            }

            Pose checkedPose = checkPose(x3, y3, z3, depthImage);

        }


        Pose checkPose(float *x3, float *y3, float *z3, cv::Mat& depthImage)
        {
            char checkedPose_print[15] = "NONE";
            Pose checkedPose = NONE;
            std::stringstream ss;

            if (y3[0] < y3[6] && y3[0] < y3[7]) {
                strcpy(checkedPose_print, "MOUNTAIN");
                checkedPose = MOUNTAIN;
                ehonnImage = cv::imread("./DaredaOre.jpg");
                imshow("Ehon", ehonnImage);
            }
            else if ((y3[8] < y3[6]) && (y3[8] < y3[7]) && (y3[2] > y3[6]) && (y3[3] > y3[7])) {
                strcpy(checkedPose_print, "OBAKE");
                checkedPose = OBAKE;
                ehonnImage = cv::imread("./Dareda.jpg");
                imshow("Ehon", ehonnImage);
            }
            else {
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
