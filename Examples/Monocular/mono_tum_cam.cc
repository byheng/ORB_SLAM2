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
#include <unistd.h>
#include<opencv2/core/core.hpp>

#include<System.h>
#include "hwBase.h"

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

    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    hwBaseDeInit();
    printf("hwbaseInit\n");
    hwBaseInit(0, 3);  //(0,3)
    printf("init success\n");

    ofstream ofile;
    ofile.open("system_log.txt");


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Main loop
    for(int ni=0; ni>0; ni++)
    {
        ofile << "IMAGE " << ni << endl;

        getFrame(LDC_F_MODE, leftBuffer, OUTHEIGHT * OUTWIDTH, rightBuffer, OUTHEIGHT * OUTWIDTH, 0);
        cv::Mat im(OUTHEIGHT, OUTWIDTH, CV_8UC1, leftBuffer);
        double tframe = std::chrono::duration_cast<std::chrono::duration<double> >(
            std::chrono::system_clock::now().time_since_epoch()).count();

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << endl;
            return 1;
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);
    }
    ofile.close();

    // Stop all threads
    SLAM.Shutdown();

    
    return 0;
}

