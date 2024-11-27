/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <unistd.h>
#include "hwBase.h"

// #define RUN_RGBO
#define RUN_MONO

#define OUTWIDTH 640
#define OUTHEIGHT 480
#define SCA_F_MODE 0
#define LDC_F_MODE 1
#define SGM_F_MODE 2

using namespace std;


int main(int argc, char **argv)
{
    char leftBuffer[OUTHEIGHT * OUTWIDTH];
    char rightBuffer[OUTHEIGHT * OUTWIDTH];
#ifdef RUN_RGBO
    char *sgmO = (char *)malloc(OUTHEIGHT * OUTWIDTH * 2);
    char *sgmC = (char *)malloc(OUTHEIGHT * OUTWIDTH * 2);
#endif

    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    hwBaseDeInit();
    printf("hwbaseInit\n");
    hwBaseInit(0, 3);  //(0,3)
    printf("init success\n");

    ofstream ofile;
    ofile.open("system_log.txt");

#ifdef RUN_RGBO
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
#endif
#ifdef RUN_MONO
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
#endif
    for(int ni=0; ni>=0; ni++) // 无限循环
    {
        ofile << "IMAGE " << ni << endl;

        // Read image and depthmap from camera
        getFrame(LDC_F_MODE, leftBuffer, OUTHEIGHT * OUTWIDTH, rightBuffer, OUTHEIGHT * OUTWIDTH, 0);
#ifdef RUN_RGBO
        getFrame(SGM_F_MODE, (char *)sgmO, OUTHEIGHT * OUTWIDTH * 2, (char *)sgmC, OUTHEIGHT * OUTWIDTH * 2, 0);
#endif
        cv::Mat imRGB(OUTHEIGHT, OUTWIDTH, CV_8UC1, leftBuffer);
        // cv::Mat imRGB(OUTHEIGHT, OUTWIDTH, CV_8UC1, rightBuffer);
        
#ifdef RUN_RGBO
        // short *pShort = (short *)sgmO;
        // 创建一个浮点型矩阵来存储转换后的数据
        cv::Mat sgmImage(OUTHEIGHT, OUTWIDTH, CV_16UC1);

        // 将16位定点数转换为浮点数
        for (int i = 0; i < OUTHEIGHT; ++i) {
            for (int j = 0; j < OUTWIDTH; ++j) {
                // 定点数转换为浮点数，假设定点数包含4位小数
                sgmImage.at<short>(i, j) = (((short *)sgmO)[i * OUTWIDTH + j] & 0xFFFF) >> 4;
            }
        }
        // 归一化
        cv::Mat imD;
        cv::normalize(sgmImage, imD, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        if(imRGB.empty() || imD.empty()) {
            cerr << "Error: Could not capture images from camera." << endl;
            return 1;
        }
#endif        
        // Get the current timestamp
        double tframe = std::chrono::duration_cast<std::chrono::duration<double> >(
                    std::chrono::system_clock::now().time_since_epoch()).count();
        // 计算FPS
        static int frameCount = 0;
        static double lastTime;
        frameCount++;
        double elapsedTime;
        if (ni == 0) {
            elapsedTime = 0;
        } else {
            elapsedTime += tframe - lastTime;
        }
        if (elapsedTime >= 1.0) {
            double fps = frameCount / elapsedTime;
            ofile << "Current FPS: " << fps << ", frameCount:" << frameCount << ", elapsedTime: " << elapsedTime << endl;
            frameCount = 0;
            elapsedTime = 0;
        }
        lastTime = tframe;


// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif

        // Pass the image to the SLAM system
#ifdef RUN_RGBO
        SLAM.TrackRGBD(imRGB,imD,tframe);
#endif
#ifdef RUN_MONO
        SLAM.TrackMonocular(imRGB,tframe);
#endif

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif

//         double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    }
    ofile.close();

    // Stop all threads
    SLAM.Shutdown();

#ifdef RUN_RGBO
    free(sgmO);
    free(sgmC);
#endif
    return 0;
}
