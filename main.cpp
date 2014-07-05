#include <iostream>

#include <NiTE.h>
#include <opencv2/opencv.hpp>


class NiteApp
{


    public:

        void initialize()
        {
            userTracker.create();
        }

        void update()
        {
            nite::UserTrackerFrameRef userFrame;
            userTracker.readFrame(&userFrame);

            depthImage = showUser(userFrame);

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
        }



    private:

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


        void showSkeleton(cv::Mat& depthImage, nite::UserTracker& userTracker, const nite::UserData& user)
        {
            const nite::Skeleton& skeelton = user.getSkeleton();
            if (skeelton.getState() != nite::SKELETON_TRACKED) {
                return;
            }

            for (int j = 0; j <= 14; ++j) {
                const nite::SkeletonJoint& joint = skeelton.getJoint((nite::JointType)j);
                if (joint.getPositionConfidence() < 0.7f) {
                    continue;
                }

                const nite::Point3f& position = joint.getPosition();
                float x = 0, y = 0;
                userTracker.convertJointCoordinatesToDepth(position.x, position.y, position.z, &x, &y);

                cv::circle(depthImage, cvPoint((int)x, (int)y), 5, cv::Scalar(0, 0, 255), -1);
            }
        }



    private:
        
        nite::UserTracker userTracker;

        cv::Mat depthImage;
};




int main(int argc, const char * argv[])
{
    try {
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
