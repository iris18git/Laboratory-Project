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


#include<opencv2/core/core.hpp>

#include <System.h>
#include <string>
#include <experimental/filesystem>
#include <stdio.h>
#include <Converter.h>
#include <fstream>

using namespace std;
namespace fs = std::experimental::filesystem;

void saveMap(ORB_SLAM2::System &SLAM){ //save map fuction that saves csv file of data points to /tmp
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for(auto p : mapPoints) {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    pointData.close();
}

int main(int argc, char **argv) //main function
{
    if(argc != 4) //program should get following arguments - vocabulary file path for orbslam, yaml configuration file path, and imgs directory path
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder" << endl;
        return 1;
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    //vTimesTrack.resize(nImages);


    bool x = false;
    int c;

    while(!x) { //wait for (o)"k" signal to start orb slam, should be given by user after drone starts taking pics and putting them in imgs folder
    	c = getchar();
    	if (c == int('k')){
    		x = true;
    	}
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im;

   int timeStamps = 0;
   int counter = 0;
   for(;counter<=10;timeStamps++){ //loop runs until there where ten times in a row with no new photos in imgs folder
        string path_name = argv[3];

        //--- filenames are unique so we can use a set
        set<fs::path> sorted_by_name;

        for (auto &entry : fs::directory_iterator(path_name)) // sorted set of images in imgs folder
            sorted_by_name.insert(entry.path());

	cout<<sorted_by_name.size()<<endl;
	if(sorted_by_name.size()==2){ // check if imgs folder has no new img (still only 2 imgs in folder which we don't delete)
		counter++;
	}
	else{
		counter=0;
	}

        auto img = ++sorted_by_name.rbegin(); //get second to last img


        //delete all element until second to last img (so it will be live)
        for (auto it = begin (sorted_by_name); (*it).c_str() != (*img).c_str(); ++it) {
        	cout << "delete " << (*it).c_str() << " ---------- until: " << (*img).c_str() << endl;
            remove((*it).c_str());
        }

	cout<<"done"<<endl;
        im = cv::imread((*img).c_str(),CV_LOAD_IMAGE_UNCHANGED);
        double tframe = timeStamps;

        if(im.empty()){
            cerr << endl << "Failed to load image" << endl;
            break;
        }

        //now we will pass img to slam

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system - track monocular function for point mapping
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);
    }

    cout<<"hii"<<endl;
    // Stop all threads
    SLAM.Shutdown();
    saveMap(SLAM); //saving map

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<vTimesTrack.size(); ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl;

    // Save camera trajectory - we will use this file to understand the drone's position
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}