/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./rgbd_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string DatasetPath = argv[3];
    
    LoadImages(DatasetPath, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    cout << "Done Loading Images" << endl;

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if (vstrImageFilenamesRGB.empty())
    {
        cerr << endl
             << "No images found in provided path." << endl;
        return 1;
    }
    else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size())
    {
        cerr << endl
             << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true, 0, "KITTI_ATLAS_rgbd.txt");

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read image and depthmap from file
        // cout << "RGB Image: " << vstrImageFilenamesRGB[ni] << endl;
        // cout << "Depth Image: " << vstrImageFilenamesD[ni] << endl;
        imRGB = cv::imread(vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGB.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << string(DatasetPath) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imD, tframe);


        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();


        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // // Wait to load the next frame
        // double T = 0;
        // if (ni < nImages - 1)
        //     T = vTimestamps[ni + 1] - tframe;
        // else if (ni > 0)
        //     T = tframe - vTimestamps[ni - 1];

        // if (ttrack < T)
        //     usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    // SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    SLAM.SaveAtlas(1);

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    cout << "Start Loading TimeStamps" << endl;
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "times.txt";
    fTimes.open(strPathTimeFile);
    cout << strPathTimeFile << endl;
    while (!fTimes.eof()){
        string s;
        getline(fTimes, s);
        if (!s.empty()){
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    cout << "Start Loading Images" << endl;
    string strPrefixRGB = strPathToSequence + "image_0/";
    string strPrefixDepth = strPathToSequence + "image_0_Depth_NearestNeighbor_and_Filling/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenamesRGB.resize(nTimes);
    vstrImageFilenamesD.resize(nTimes);

    const int CounterOffset = 0;


    for (int i = 0; i < nTimes; i++) // first depth image in this dataset is image 5
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i + CounterOffset;
        vstrImageFilenamesRGB[i] = strPrefixRGB + ss.str() + ".png";
        vstrImageFilenamesD[i] = strPrefixDepth + ss.str() + ".png";
    }
}
