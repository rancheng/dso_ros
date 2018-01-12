/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
//#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>

#include "FullSystem/ImmaturePoint.h"

#include "IOWrapper/ImageRW.h"

using namespace dso;




class dso_wrapper{
public:

    FullSystem* fullSystem = 0;
    Undistort* undistorter = 0;
    int frameID = 0;
    std::string calib = "";
    std::string vignetteFile = "";
    std::string gammaFile = "";

    bool useSampleOutput=false;


    int mode=0;








    // frame and images, dso data to publish

    FrameHessian* frameHessian;
    std::vector<MinimalImageB3*> imgB3s = {};


    //ros interface for publish and subscribe
    ros::NodeHandle nh;
    image_transport::Publisher debug_img_pub;
    ros::Publisher transform_pub;
    ros::Subscriber imgSub;
    void parseArgument(char* arg)
    {
        int option;
        char buf[1000];

        if(1==sscanf(arg,"sampleoutput=%d",&option))
        {
            if(option==1)
            {
                useSampleOutput = true;
                printf("USING SAMPLE OUTPUT WRAPPER!\n");
            }
            return;
        }

        if(1==sscanf(arg,"quiet=%d",&option))
        {
            if(option==1)
            {
                setting_debugout_runquiet = true;
                printf("QUIET MODE, I'll shut up!\n");
            }
            return;
        }


        if(1==sscanf(arg,"nolog=%d",&option))
        {
            if(option==1)
            {
                setting_logStuff = false;
                printf("DISABLE LOGGING!\n");
            }
            return;
        }

        if(1==sscanf(arg,"nogui=%d",&option))
        {
            if(option==1)
            {
                disableAllDisplay = true;
                printf("NO GUI!\n");
            }
            return;
        }
        if(1==sscanf(arg,"nomt=%d",&option))
        {
            if(option==1)
            {
                multiThreading = false;
                printf("NO MultiThreading!\n");
            }
            return;
        }
        if(1==sscanf(arg,"calib=%s",buf))
        {
            calib = buf;
            printf("loading calibration from %s!\n", calib.c_str());
            return;
        }
        if(1==sscanf(arg,"vignette=%s",buf))
        {
            vignetteFile = buf;
            printf("loading vignette from %s!\n", vignetteFile.c_str());
            return;
        }

        if(1==sscanf(arg,"gamma=%s",buf))
        {
            gammaFile = buf;
            printf("loading gammaCalib from %s!\n", gammaFile.c_str());
            return;
        }

        if(1==sscanf(arg,"mode=%d",&option))
        {

            mode = option;
            if(option==0)
            {
                printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
            }
            if(option==1)
            {
                printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
                setting_photometricCalibration = 0;
                setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
                setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            }
            if(option==2)
            {
                printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
                setting_photometricCalibration = 0;
                setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
                setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
                setting_minGradHistAdd=3;
            }
            return;
        }
        printf("could not parse argument \"%s\"!!\n", arg);
    }





    void vidCb(const sensor_msgs::ImageConstPtr img)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        assert(cv_ptr->image.type() == CV_8U);
        assert(cv_ptr->image.channels() == 1);


        if(setting_fullResetRequested)
        {
            std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
            delete fullSystem;
            for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
            fullSystem = new FullSystem();
            fullSystem->linearizeOperation=false;
            fullSystem->outputWrapper = wraps;
            if(undistorter->photometricUndist != 0)
                fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
            setting_fullResetRequested=false;
        }

        MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
        ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
        fullSystem->addActiveFrame(undistImg, frameID);
        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;

        if(wraps.size()!=0) {
            IOWrap::Output3DWrapper * wrap = wraps.back();
            std::vector<SE3>* poses = wrap->getCameraPose();
            if(poses->size()!=0)
            {
                SE3 pose = poses->back();
                std::cout << "----------Pose Size: "<< poses->size() << "--------" << std::endl;
                std::cout << "-----------------------Camera Pose!!!!!----------------------" << std::endl;
                std::cout << pose.matrix3x4() << std::endl;
                auto rt_matrix = pose.matrix3x4();
                geometry_msgs::Transform dso_est_tf;
                dso_est_tf.translation.x = rt_matrix(0,3);
                dso_est_tf.translation.y = rt_matrix(1,3);
                dso_est_tf.translation.z = rt_matrix(2,3);
                dso_est_tf.rotation.x = pose.unit_quaternion().x();
                dso_est_tf.rotation.y = pose.unit_quaternion().y();
                dso_est_tf.rotation.z = pose.unit_quaternion().z();
                dso_est_tf.rotation.w = pose.unit_quaternion().w();
                transform_pub.publish(dso_est_tf);
//                Sophus::SE3f posef = pose;
//                geometry_msgs::Transform dso_est_tf;
//                dso_est_tf.translation.x = posef.translation();
//                posef.unit_quaternion().x;
//                pose.unit_quaternion().y;
//                pose.unit_quaternion().z;
//                posef.
            }
        }
        frameHessian = fullSystem->getFrameHessian();

        std::cout << "----------frameHessian: "<< frameHessian << "--------" << std::endl;
        //std::cout << "----------WG[0]: "<< wG[0] << "--------" << std::endl;
        if(frameHessian!=NULL){
            Eigen::Vector3f* fd = frameHessian->dI;
            int wh = hG[0]*wG[0];
            MinimalImageB3* imgB3 = new MinimalImageB3(wG[0],hG[0]);
            imgB3s.push_back(imgB3);
            for(int i=0;i<wh;i++)
            {
                int c = fd[i][0]*0.9f;
                if(c>255) c=255;
                imgB3->at(i) = Vec3b(c,c,c);
            }
            std::cout << "----------ImgB3s size(): "<< imgB3s.size() << "--------" << std::endl;
            for(ImmaturePoint* ph : frameHessian->immaturePoints)
            {
                if(ph==0) continue;

                if(ph->lastTraceStatus==ImmaturePointStatus::IPS_GOOD)
                    imgB3->setPixelCirc(ph->u+0.5f, ph->v+0.5f, Vec3b(0,255,0));
                if(ph->lastTraceStatus==ImmaturePointStatus::IPS_OOB)
                    imgB3->setPixelCirc(ph->u+0.5f, ph->v+0.5f, Vec3b(255,0,0));
                if(ph->lastTraceStatus==ImmaturePointStatus::IPS_OUTLIER)
                    imgB3->setPixelCirc(ph->u+0.5f, ph->v+0.5f, Vec3b(0,0,255));
                if(ph->lastTraceStatus==ImmaturePointStatus::IPS_SKIPPED)
                    imgB3->setPixelCirc(ph->u+0.5f, ph->v+0.5f, Vec3b(255,255,0));
                if(ph->lastTraceStatus==ImmaturePointStatus::IPS_BADCONDITION)
                    imgB3->setPixelCirc(ph->u+0.5f, ph->v+0.5f, Vec3b(255,255,255));
                if(ph->lastTraceStatus==ImmaturePointStatus::IPS_UNINITIALIZED)
                    imgB3->setPixelCirc(ph->u+0.5f, ph->v+0.5f, Vec3b(0,0,0));
            }
        }
        if(imgB3s.size()>0){
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv::Mat(imgB3s.back()->h, imgB3s.back()->w, CV_8UC3, imgB3s.back()->data)).toImageMsg();
            debug_img_pub.publish(msg);
        }

        frameID++;
        delete undistImg;

    }


    dso_wrapper( int argc, char** argv )
    {

        image_transport::ImageTransport it(nh);
        debug_img_pub = it.advertise("debug/image_raw", 1);
        transform_pub = nh.advertise<geometry_msgs::Transform>("dso/pose",1);
        for(int i=1; i<argc;i++) parseArgument(argv[i]);
        disableAllDisplay = true;

        setting_desiredImmatureDensity = 1000;
        setting_desiredPointDensity = 1200;
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations=4;
        setting_minOptIterations=1;
        setting_logStuff = false;
        setting_kfGlobalWeight = 1.3;


        // printf("MODE WITH CALIBRATION, but without exposure times!\n");
        // setting_photometricCalibration = 2;
        // setting_affineOptModeA = 0;
        // setting_affineOptModeB = 0;



        undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

        setGlobalCalib(
                (int)undistorter->getSize()[0],
                (int)undistorter->getSize()[1],
                undistorter->getK().cast<float>());


        fullSystem = new FullSystem();
        fullSystem->linearizeOperation=false;


//    if(!disableAllDisplay)
//	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
//	    		 (int)undistorter->getSize()[0],
//	    		 (int)undistorter->getSize()[1]));


        if(useSampleOutput)
        {
            fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());
        }


        if(undistorter->photometricUndist != 0)
            fullSystem->setGammaFunction(undistorter->photometricUndist->getG());


        imgSub = nh.subscribe("image", 1, &dso_wrapper::vidCb,this);
        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::waitForShutdown();
    }
    ~dso_wrapper(){

        for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
        {
            ow->join();
            delete ow;
        }
        //fullSystem->printResult("result.txt");

        delete undistorter;
        delete fullSystem;
        //return 0;
    }
};

// main function inside
int main(int argc, char** argv){

    ros::init(argc, argv, "dso_ros_wrapper");
    dso_wrapper my_wrapper(argc, argv);
    delete &my_wrapper;
}
