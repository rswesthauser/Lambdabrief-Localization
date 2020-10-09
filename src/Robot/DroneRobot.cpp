/**
 * This file is part of λ-BRIEF Localization
 *
 * Copyright (c) 2020 Ricardo Westhauser <rswesthauser at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https://github.com/rswesthauser/Lambdabrief-Localization>
 *
 * λ-BRIEF Localization is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * λ-BRIEF Localization is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with λ-BRIEF Localization.  If not, see <https://www.gnu.org/licenses/>.
**/

#include "DroneRobot.h"
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "src/VegetationIndex/Ndvi.h"
#include "src/VegetationIndex/Grvi.h"
#include "src/VegetationIndex/VegetationIndex.h"
#include "config.h"
#include "src/VegetationIndex/VegetatonIndexGen.h"
#include "src/Utils/RadiusVolumeTransferFunctions.h"

DroneRobot::DroneRobot()
{       
}

DroneRobot::DroneRobot(string& mapPath, string& trajectoryName, vector< heuristicType* > &heuristicTypes, bool quiet, string& oName, int startVal, int finishVal)
{
    this->step = 0;
    numQuantization = 25;
    runQuiet = quiet;
    if(runQuiet)
    {
        cout << "Initializing" << flush;
        disableCout();
    }

    start=startVal;
    finish=finishVal;
    current=start;
    outputName=oName;

    // Read fullMap
    originalMap = cv::imread(mapPath,CV_LOAD_IMAGE_COLOR);

    if(! originalMap.data )                              // Check for invalid input
    {
        cout<<"Path:"<<mapPath<<endl;
        return;
    }
    isBRIEF = false;

    if(heuristicTypes[0]->strategy == BRIEF){
        isBRIEF = true;

        if(mapPreProc == 1)
            originalMap = quantization(originalMap, numQuantization);        
    }

    globalMaps.push_back(originalMap);

    //Converting map to grayscale
    cv::Mat grayMap;
    cvtColor(originalMap,grayMap,CV_BGR2GRAY);
    cvtColor(grayMap,grayMap,CV_GRAY2BGR);
    globalMaps.push_back(grayMap);

    //Converting map to Lab
    cv::Mat labMap, filteredLabMap;
    cvtColor(originalMap, labMap, CV_BGR2Lab);
    if(mapPreProc == 2)
    {
        bilateralFilter(labMap, filteredLabMap, 9, 50, 20, cv::BORDER_DEFAULT);
        globalMaps.push_back(filteredLabMap);
    }
    else if(mapPreProc == 1)
        globalMaps.push_back(labMap);

    //Converting map to HSV
    cv::Mat hsvMap;
    cvtColor(originalMap, hsvMap, CV_BGR2HSV);
    globalMaps.push_back(hsvMap);

    if(runQuiet)
    {
        enableCout();
        cout << "." << flush;
        disableCout();
    }

    // Open ground truth file, if available
    rawname = trajectoryName.substr(0, trajectoryName.find_last_of("."));
    truthFile.open(rawname+"_truth.txt",std::fstream::in);
    if(!truthFile.is_open())
    {
        availableGTruth = false;
    }
    else
    {
        flPrimeiraLeituraGTRawOdom = true;
        realPose = readGroundTruth();
        availableGTruth = true;
    }

    // Open corrected ground truth file, if available
    rawname = trajectoryName.substr(0, trajectoryName.find_last_of("."));
    corrTruthFile.open(rawname+"_truth_corr.txt",std::fstream::in);
    if(!truthFile.is_open())
    {
        availableCorrectedGTruth = false;
    }
    else
    {
        readCorrectedGroundTruth();
        availableCorrectedGTruth = true;
        cout<< "Corrected Ground Truth file available." << endl;
    }

    // Open angles file, if available
    anglesFile.open(rawname+"_angles.txt",std::fstream::in);
    if(!anglesFile.is_open())
        availableAngles = false;
    else
    {
        readDroneAngles();
        availableAngles = true;
        cout<< "Angles file available." << endl;
    }


    // Open odometry file, if available
    odomFile.open(rawname+"_odom.txt",std::fstream::in);
    if(!odomFile.is_open())
    {
        // Check if there are available offline odoms
        vector<string> opaths = Utils::getListOfFiles(rawname);
        cout << "Num offline odoms " << opaths.size() << endl;

        if(!opaths.empty())
        {
            std::default_random_engine generator;
            std::uniform_int_distribution<int> randomVal(0,opaths.size());
            int id=randomVal(generator);

            odomFile.open(rawname+"/"+opaths[id],std::fstream::in);
            offlineOdom = true;
            isRawOdom = false;
        }
        else
        {
            offlineOdom = false;

            /******************** Prepare odom file *****************************/
            time_t t = time(0);
            struct tm *now = localtime(&t);
            stringstream odomName;

            odomName << rawname << "/odom-" << -100+now->tm_year
                            << setfill('0') << setw(2) << 1+now->tm_mon
                            << setfill('0') << setw(2) << now->tm_mday << '-'
                            << setfill('0') << setw(2) << now->tm_hour
                            << setfill('0') << setw(2) << now->tm_min
                            << setfill('0') << setw(2) << now->tm_sec << ".txt";
            cout << odomName.str() << endl; cout.flush();

            odomFile.open(odomName.str(),std::fstream::out);
        }
    }
    else
    {
        offlineOdom = true;
        isRawOdom = true;
        readRawOdometryFromFile(prevRawOdom);
    }

    if(runQuiet){
        enableCout();
        cout << "." << flush;
        disableCout();
    }

    slowMethod=false;
    // Initialize heuristics
    for(int i=0;i<heuristicTypes.size();++i)
    {
        heuristicType* hT = heuristicTypes[i];
        Heuristic* heur = NULL;
        MapGrid* mg = NULL;

        int id=getNextProperHeuristicID(hT->strategy);

        if(hT->strategy == BRIEF)
        {
            heur = new BriefHeuristic(BRIEF, id, hT->colorDifference, hT->threshold);
            if(hT->lowThreshold != -1) ((BriefHeuristic*)heur)->lowThreshold = hT->lowThreshold;
            if(hT->multiplierThreshold != -1) ((BriefHeuristic*)heur)->multiplierThreshold = hT->multiplierThreshold;
            if(hT->margin != -1) ((BriefHeuristic*)heur)->margin = hT->margin;
            if(hT->numberPairs != -1) ((BriefHeuristic*)heur)->totalPairs = hT->numberPairs;
            ((BriefHeuristic*)heur)->printInfo();
        }
        if(runQuiet)
        {
            enableCout();
            cout << "." << flush;
            disableCout();
        }

        heuristics.push_back(heur);
        cachedMaps.push_back(mg);
    }

    if(runQuiet)
    {
        enableCout();
        cout << "." << flush;
        disableCout();
    }

    // Read images names
    fstream input;
    input.open(trajectoryName,std::fstream::in);    
    while(input.peek() != fstream::traits_type::eof())
    {
        string tempStr;
        getline(input,tempStr);
        imagesNames.push_back(tempStr);
    }
    imagesNames.erase(imagesNames.begin(), imagesNames.begin() + (imgIniExec - imgIniDs));//Remove o início
    imagesNames.erase(imagesNames.begin() + (imgFimExec - imgIniExec + 1), imagesNames.begin() + (imgFimExec - imgIniExec + 1) +  (imgFimDs - imgFimExec));
    cout << "Num images " << imagesNames.size() << endl;
    input.close();

    //Read the RED channel image names    
    fstream inputRED;
    trajectoryName.replace(trajectoryName.find(".txt"), 4, "_RED.txt");    
    inputRED.open(trajectoryName,std::fstream::in);
    while(inputRED.peek() != fstream::traits_type::eof())
    {
        string tempStr;
        getline(inputRED,tempStr);
        imageNamesRED.push_back(tempStr);
    }
    imageNamesRED.erase(imageNamesRED.begin(), imageNamesRED.begin() + (imgIniExec - imgIniDs));//Remove o início
    imageNamesRED.erase(imageNamesRED.begin() + (imgFimExec - imgIniExec + 1), imageNamesRED.begin() + (imgFimExec - imgIniExec + 1) +  (imgFimDs - imgFimExec));
    cout << "Num RED images " << imageNamesRED.size() << endl;
    inputRED.close();

    //Read the NIR channel image names
    fstream inputNIR;
    trajectoryName.replace(trajectoryName.find("_RED.txt"), 8, "_NIR.txt");
    inputNIR.open(trajectoryName,std::fstream::in);
    while(inputNIR.peek() != fstream::traits_type::eof())
    {
        string tempStr;
        getline(inputNIR,tempStr);
        imageNamesNIR.push_back(tempStr);
    }
    imageNamesNIR.erase(imageNamesNIR.begin(), imageNamesNIR.begin() + (imgIniExec - imgIniDs));//Remove o início
    imageNamesNIR.erase(imageNamesNIR.begin() + (imgFimExec - imgIniExec + 1), imageNamesNIR.begin() + (imgFimExec - imgIniExec + 1) +  (imgFimDs - imgFimExec));
    cout << "Num NIR images " << imageNamesNIR.size() << endl;
    inputNIR.close();

    //Read the REG channel image names
    fstream inputREG;
    trajectoryName.replace(trajectoryName.find("_NIR.txt"), 8, "_REG.txt");
    inputREG.open(trajectoryName,std::fstream::in);
    while(inputREG.peek() != fstream::traits_type::eof())
    {
        string tempStr;
        getline(inputREG,tempStr);
        imageNamesREG.push_back(tempStr);
    }
    imageNamesREG.erase(imageNamesREG.begin(), imageNamesREG.begin() + (imgIniExec - imgIniDs));//Remove o início
    imageNamesREG.erase(imageNamesREG.begin() + (imgFimExec - imgIniExec + 1), imageNamesREG.begin() + (imgFimExec - imgIniExec + 1) +  (imgFimDs - imgFimExec));
    cout << "Num REG images " << imageNamesREG.size() << endl;
    inputREG.close();

    //Read the GRE channel image names
    fstream inputGRE;
    trajectoryName.replace(trajectoryName.find("_REG.txt"), 8, "_GRE.txt");
    inputGRE.open(trajectoryName,std::fstream::in);
    while(inputGRE.peek() != fstream::traits_type::eof())
    {
        string tempStr;
        getline(inputGRE,tempStr);
        imageNamesGRE.push_back(tempStr);
    }
    imageNamesGRE.erase(imageNamesGRE.begin(), imageNamesGRE.begin() + (imgIniExec - imgIniDs));//Remove o início
    imageNamesGRE.erase(imageNamesGRE.begin() + (imgFimExec - imgIniExec + 1), imageNamesGRE.begin() + (imgFimExec - imgIniExec + 1) +  (imgFimDs - imgFimExec));
    cout << "Num GRE images " << imageNamesGRE.size() << endl;
    inputGRE.close();

    if(runQuiet){
        enableCout();
        cout << " Running " << endl;
        disableCout();
    }
}

void DroneRobot::reinitialize()
{
    // Open ground truth file, if available
    if(truthFile.is_open())
        truthFile.close();
    truthFile.open(rawname+"_truth.txt",std::fstream::in);

    if(!truthFile.is_open())
    {
        availableGTruth = false;
    }
    else{
        availableGTruth = true;       
        realPose = readGroundTruth();
    }

    // Open angles file, if available
    anglesFile.open(rawname+"_angles.txt",std::fstream::in);
    if(!anglesFile.is_open())
        availableAngles = false;
    else
    {
        readDroneAngles();
        availableAngles = true;
        cout<< "Angles file available." << endl;
    }

    // Open odometry file, if available
    if(odomFile.is_open())
        odomFile.close();
    odomFile.open(rawname+"_odom.txt",std::fstream::in);
    if(!odomFile.is_open())
    {
        // Check if there are available offline odoms
        vector<string> opaths = Utils::getListOfFiles(rawname);
        cout << "Num offline odoms " << opaths.size() << endl;

        if(!opaths.empty()){
            std::default_random_engine generator;
            std::uniform_int_distribution<int> randomVal(0,opaths.size());
            int id=randomVal(generator);

            odomFile.open(rawname+"/"+opaths[id],std::fstream::in);
            offlineOdom = true;
            isRawOdom = false;
        }
        else
        {
            offlineOdom = false;

            /******************** Prepare odom file *****************************/
            time_t t = time(0);
            struct tm *now = localtime(&t);
            stringstream odomName;

            odomName << rawname << "/odom-" << -100+now->tm_year
                            << setfill('0') << setw(2) << 1+now->tm_mon
                            << setfill('0') << setw(2) << now->tm_mday << '-'
                            << setfill('0') << setw(2) << now->tm_hour
                            << setfill('0') << setw(2) << now->tm_min
                            << setfill('0') << setw(2) << now->tm_sec << ".txt";
            cout << odomName.str() << endl; cout.flush();

            odomFile.open(odomName.str(),std::fstream::out);
        }
    }
    else
    {
        offlineOdom = true;
        isRawOdom = true;
        readRawOdometryFromFile(prevRawOdom);
    }

    string oName;
    if(!outputName.empty())
        oName = outputName+to_string(current)+".txt";
    mcLocalization->restart(realPose,oName);

    step = 0;

    if(runQuiet)
    {
        enableCout();
        cout << " Running " << endl;
        disableCout();
    }

}

DroneRobot::~DroneRobot()
{
}

int DroneRobot::getNextProperHeuristicID(STRATEGY type)
{
    int id=0;
    for(int s=0; s<heuristics.size(); s++)
        if(heuristics[s]->getType() == type)
            id++;
    return id;
}

void DroneRobot::generateObservations(string imagePath)
{
    // Show global map (resized to 20% of normal size).
    cv::Mat window;
    double scale = 1.0/5.0;
    resize(globalMaps[0],window,cv::Size(0,0),scale,scale);
    imshow( "Global Map", window );

    stringstream ss;
    int id=0;

    // Read odom
    Pose3d p;
    while(readRawOdometryFromFile(p))
    {
        cout << "x:" << p.x << " y:" << p.y << " scale: " << p.z << " th:" << RAD2DEG(p.yaw) << endl;
        circle(window,cv::Point2f(p.x*scale,p.y*scale),10,cv::Scalar(0,0,255));
        imshow( "Trajectory", window );

        cv::Mat z = Utils::getRotatedROIFromImage(p,cv::Size2f(320,240),globalMaps[0]);
        imshow( "Observation", z );

        // Save image
        ss.str("");
        ss << imagePath << setfill('0') << setw(6) << id << ".png";
        cout << "Image: " << ss.str() << endl;
        imwrite(ss.str(), z );

        cv::waitKey(10);
        id++;
    }
}

void DroneRobot::initialize(ConnectionMode cmode, LogMode lmode, string fname, int numParticles)
{
    // Create windows for the global map and the local map
    if(!multispectralCam)
        cv::namedWindow( "Local Map", cv::WINDOW_KEEPRATIO );

    string oName;
    if(!outputName.empty())
        oName = outputName+to_string(current)+".txt";

    mcLocalization = new MCL(heuristics, cachedMaps, globalMaps, realPose, oName, numParticles, droneAngles, CorrectedGT);
    step = 0;
    ready_ = true;
}

//Called by main after initialize
void DroneRobot::run()
{    
    if(step >= imagesNames.size())
    {
        if(runQuiet){
            enableCout();
            cout << "\r" << step*100/imagesNames.size() << "%" << endl;
            disableCout();
        }

        if(current==finish)
            exit(0);
        else{
            current++;
            reinitialize();
        }
    }

    // Read image seen by drone   
    cv::Mat currentMap = cv::imread(imagesNames[step],CV_LOAD_IMAGE_COLOR);
    cv::Mat currentMapOriginal = currentMap.clone();
    currentImage++;

    if(! currentMap.data)                              // Check for invalid input
    {
        cout <<  "Could not open or find local map" << std::endl;
        cout<<"Endereco:"<<imagesNames[step]<<endl;
        return;
    }
    if(multispectralCam)
    {
        // Read image seen by drone (RED channel)        
        currentREDMap = cv::imread(imageNamesRED[step], cv::IMREAD_GRAYSCALE);
        if(! currentREDMap.data)                              // Check for invalid input
        {
            cout <<  "Could not open or find local map (RED)" << std::endl;
            cout<<"Endereco:"<<imageNamesRED[step]<<endl;
            return;
        }

        // Read image seen by drone (NIR channel)       
        currentNIRMap = cv::imread(imageNamesNIR[step], cv::IMREAD_GRAYSCALE);
        if(! currentNIRMap.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find local map (NIR)" << std::endl ;         
            cout<<"Endereco:"<<imageNamesNIR[step]<<endl;
            return;
        }        

        // Read image seen by drone (Red Edge channel)  
        currentREGMap = cv::imread(imageNamesREG[step], cv::IMREAD_GRAYSCALE);
        if(! currentREGMap.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find local map (REG)" << std::endl ;
            cout<<"Endereco:"<<imageNamesREG[step]<<endl;
            return;
        }

        // Read image seen by drone (GRE channel)
        currentGREMap = cv::imread(imageNamesGRE[step], cv::IMREAD_GRAYSCALE);
        if(! currentGREMap.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find local map (GRE)" << std::endl ;
            cout<<"Endereco:"<<imageNamesGRE[step]<<endl;
            return;
        }
    }

    //IMAGE QUANTIZATION
    if(isBRIEF)
    {
        if(mapPreProc == 1)
            currentMap = quantization(currentMap, numQuantization);        

        else if(mapPreProc == 2)
        {
            cv::Mat filteredLocalMap;
            bilateralFilter(currentMap, filteredLocalMap, 9, 50, 20, cv::BORDER_DEFAULT);
            currentMap = filteredLocalMap;
        }

        if(msImgPreProc != 0)
        {
            currentREDMap = msImagePreProcessing(currentREDMap);
            currentNIRMap = msImagePreProcessing(currentNIRMap);
            currentREGMap = msImagePreProcessing(currentREGMap);
            currentGREMap = msImagePreProcessing(currentGREMap);
        }
    }

    if(!runQuiet)
    {
        if(!multispectralCam)        
            imshow( "Local Map", currentMap );        
        cv::waitKey(10);
    }

    cout << "Image" << step << ' ';
    step += 1;

    if((slowMethod || step%5==0) && runQuiet){
        enableCout();
        cout << "\r" << step*100/imagesNames.size() << "%" << flush;
        disableCout();
    }

    bool odom_reliable = true;

    // Compute or read Odometry
    if(step>1){
        if(offlineOdom){
            //ODOMETRY AVAILABLE - here
            if(isRawOdom)
            {
                odometry_ = readOdometry();
            }
            else
            {
                pair<Pose3d,bool> od = readOdometryNew();
                odometry_ = od.first;
                odom_reliable = od.second;
            }
        }
        else
        {
            pair<Pose3d,bool> od = findOdometry(prevMap,currentMap);
            odom_reliable = od.second;
            if(odom_reliable)
                odometry_ = od.first;
            else
                odometry_ = Pose3d();
            odomFile << odometry_.x << ' ' << odometry_.y << ' ' << odometry_.z << ' ' << odometry_.yaw << ' ' << odom_reliable << endl;
        }
    }
    else
    {
        odometry_ = Pose3d();
    }
    prevOdometry = odometry_;
    prevMap = currentMap;

    //Use in the Odometry
    prevNDVIMap = currentNDVIMap;

    if(availableGTruth)
    {
        if(RAW_ODOM && flPrimeiraLeituraGTRawOdom)
            flPrimeiraLeituraGTRawOdom = false;
        else
            realPose = readGroundTruth();
    }
    else
    {
        realPose = prevRawOdom;
    }

    // Obtain
    double time=0;

    // Run Monte Carlo Localization
    mcLocalization->run(odometry_, odom_reliable, currentMap, time, realPose, currentREDMap, currentNIRMap, currentREGMap, currentGREMap, currentMapOriginal);
    mcLocalization->writeLogFile3d();

    // Navigation
    switch(motionMode_)
    {
        case ENDING:
            cout << "Ending program" << endl;
            exit(0);
            break;
        default:
            break;
    }
}

cv::Mat DroneRobot::msImagePreProcessing(cv::Mat &img)
{
    cv::Mat new_image(img.rows, img.cols, img.type());

    if(msImgPreProc == 1)
         GaussianBlur(img, new_image, cv::Size(gbKernel, gbKernel), gbSigma, gbSigma, cv::BORDER_DEFAULT);
    else if(msImgPreProc == 2)
         bilateralFilter(img, new_image, bfD, bfSigmaColor, bfSigmaSpace, cv::BORDER_DEFAULT);
    return new_image;
}

cv::Mat DroneRobot::quantization(cv::Mat &img, float value)
{
    cv::Mat new_image(img.rows, img.cols, img.type());
    cv::Vec3b intensity, intensity2;

    for(int i = 0; i < img.rows;i++)
    {
        for(int j = 0; j < img.cols;j++)
        {
            intensity = img.at<cv::Vec3b>(i,j);
            for(int k = 0; k < 3; k++)
            {
                intensity2.val[k] = floor(((float)intensity.val[k]*value)/255.)*(255/value);
                if((float)intensity2.val[k] > 255) intensity2.val[k] = 255;
                if((float)intensity2.val[k] < 0) intensity2.val[k] = 0;


            }
            new_image.at<cv::Vec3b>(i,j) = intensity2;
        }
    }    
    return new_image;
}

pair<Pose3d, bool> DroneRobot::readOdometryNew()
{
    if(odomFile.peek() == fstream::traits_type::eof())
        return pair<Pose3d,bool>(Pose3d(),false);

    Pose3d odom;
    bool reliable = true;
    string tempStr;
    odomFile >> std::fixed >> std::setprecision(32);
    odomFile >> odom.x >> odom.y >> odom.z >> odom.yaw >> reliable;
    getline(odomFile,tempStr);

    return pair<Pose3d,bool>(odom,reliable);
}

Pose3d DroneRobot::readOdometry()
{
    Pose3d newRawOdom;
    readRawOdometryFromFile(newRawOdom);

    double deltaX = newRawOdom.x - prevRawOdom.x;
    double deltaY = newRawOdom.y - prevRawOdom.y;
    Pose3d odom;
    odom.x = cos(-prevRawOdom.yaw)*deltaX - sin(-prevRawOdom.yaw)*deltaY;
    odom.y = sin(-prevRawOdom.yaw)*deltaX + cos(-prevRawOdom.yaw)*deltaY;
    odom.yaw = Utils::getDiffAngle(newRawOdom.yaw,prevRawOdom.yaw);
    cout << "RAW " << RAD2DEG(newRawOdom.yaw)
         << " - "  << RAD2DEG(prevRawOdom.yaw)
         << " = "  << RAD2DEG(odom.yaw) << endl;

    prevRawOdom = newRawOdom;
    return odom;
}

bool DroneRobot::readRawOdometryFromFile(Pose3d& p)
{
   if(odomFile.peek() == fstream::traits_type::eof())
       return false;

   string tempStr;
   odomFile >> p.x >> p.y >> p.z >> p.yaw;
   getline(odomFile,tempStr);

   p.yaw = DEG2RAD(p.yaw);
   while(p.yaw > M_PI)
       p.yaw -= 2*M_PI;
   while(p.yaw < -M_PI)
       p.yaw += 2*M_PI;

   return true;
}

Pose3d DroneRobot::readGroundTruth()
{
   if(truthFile.peek() == fstream::traits_type::eof()) // keep the last good pose
       return realPose;

   string tempStr;
   Pose3d p;
   truthFile >> p.x >> p.y >> p.z >> p.yaw;
   getline(truthFile,tempStr);

   p.yaw = DEG2RAD(p.yaw);
   while(p.yaw > M_PI)
       p.yaw -= 2*M_PI;
   while(p.yaw < -M_PI)
       p.yaw += 2*M_PI;

   return p;
}

void DroneRobot::readCorrectedGroundTruth()
{
   string tempStr;
   Pose3d p;

   while(!(corrTruthFile.peek() == fstream::traits_type::eof()))
   {
       corrTruthFile >> p.x >> p.y >> p.z >> p.yaw;
       getline(corrTruthFile,tempStr);

       p.roll = DEG2RAD(p.roll);
       p.pitch = DEG2RAD(p.pitch);
       p.yaw = DEG2RAD(p.yaw);
       while(p.yaw > M_PI)
           p.yaw -= 2*M_PI;
       while(p.yaw < -M_PI)
           p.yaw += 2*M_PI;

       CorrectedGT.push_back(p);
   }
}

void DroneRobot::readDroneAngles()
{
   string tempStr;
   Pose3d p;

   while(!(anglesFile.peek() == fstream::traits_type::eof()))
   {
       anglesFile >> p.roll >> p.pitch >> p.yaw;
       getline(anglesFile,tempStr);

       p.roll = DEG2RAD(p.roll);
       p.pitch = DEG2RAD(p.pitch);
       p.yaw = DEG2RAD(p.yaw);

       p.yawInvertido = DEG2RAD(fmod((p.yaw + 180 + 180 + 360), 360)-180);
       droneAngles.push_back(p);
   }
}

void DroneRobot::drawMatchedImages(cv::Mat& prevImage, cv::Mat& curImage, const cv::Mat& wm, const int warp_mode)
{
    vector<cv::Point2f> yourPoints;
    yourPoints.push_back(cv::Point2f(0,0));
    yourPoints.push_back(cv::Point2f(curImage.cols,0));
    yourPoints.push_back(cv::Point2f(curImage.cols,curImage.rows));
    yourPoints.push_back(cv::Point2f(0,curImage.rows));
    yourPoints.push_back(cv::Point2f(curImage.cols/2,curImage.rows/2));
    yourPoints.push_back(cv::Point2f(curImage.cols,curImage.rows/2));
    yourPoints.push_back(cv::Point2f(curImage.cols/2,0));

    cv::Mat warp_matrix = wm.clone();

    vector<cv::Point2f> transformedPoints;
    transformedPoints.resize(yourPoints.size());

    cv::Mat transf;
    transf = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat aux = transf.colRange(0,3).rowRange(0,2);
    warp_matrix.copyTo(aux);

    perspectiveTransform(yourPoints, transformedPoints, transf.inv());

    vector<cv::Point2f> totalPoints;
    totalPoints.reserve( yourPoints.size() + transformedPoints.size() );
    totalPoints.insert( totalPoints.end(), yourPoints.begin(), yourPoints.end() );
    totalPoints.insert( totalPoints.end(), transformedPoints.begin(), transformedPoints.end() );

    cv::Rect r = boundingRect(totalPoints);
    cv::Mat displ = cv::Mat::eye(3, 3, CV_32F);
    displ.at<float>(0,2) = -r.x;
    displ.at<float>(1,2) = -r.y;
    perspectiveTransform(yourPoints, yourPoints, displ);
    perspectiveTransform(transformedPoints, transformedPoints, displ);

    cv::Mat blank(r.height, r.width, CV_8UC3, cv::Scalar(0));
    cv::Mat im2_aligned;
    warp_matrix.at<float>(0,2) += r.x;
    warp_matrix.at<float>(1,2) += r.y;

    if (warp_mode != cv::MOTION_HOMOGRAPHY)
        warpAffine(curImage, im2_aligned, warp_matrix, blank.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    else
        warpPerspective (curImage, im2_aligned, warp_matrix, blank.size(),cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

    blank += im2_aligned*0.5;

    cv::Mat aux1 = blank.colRange(-r.x,-r.x+prevImage.cols).rowRange(-r.y,-r.y+prevImage.rows);
    cv::Mat im1 = prevImage*0.5 + aux1;
    im1.copyTo(aux1);

    for (int i = 0; i < 4; i++)
        line(blank, yourPoints[i], yourPoints[(i+1)%4], cv::Scalar(0,255,0));
    for (int i = 0; i < 4; i++)
        line(blank, transformedPoints[i], transformedPoints[(i+1)%4], cv::Scalar(0,0,255));

    rectangle(blank,cv::Rect(yourPoints[4]-cv::Point2f(2.5,2.5), cv::Size2f(5,5)), cv::Scalar(0,255,0));
    circle(blank,transformedPoints[4],10,cv::Scalar(0,0,255));
    line(blank, yourPoints[4], transformedPoints[4], cv::Scalar(0,0,255));
    line(blank, transformedPoints[4], transformedPoints[5], cv::Scalar(255,255,255));
    line(blank, transformedPoints[4], transformedPoints[6], cv::Scalar(255,255,255));

    imshow("Image Blank", blank);
}

//Edge detection using Sobel
cv::Mat DroneRobot::GetGradient(cv::Mat src_gray)
{
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  int scale = 1;
  int delta = 0;
  int ddepth = CV_32FC1; ;

  // Calculate the x and y gradients using Sobel operator
  Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
  convertScaleAbs( grad_x, abs_grad_x );

  Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
  convertScaleAbs( grad_y, abs_grad_y );

  // Combine the two gradients
  cv::Mat grad;
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  return grad;
}

pair<Pose3d,bool> DroneRobot::findOdometry(cv::Mat& prevImage, cv::Mat& curImage)
{
    cv::Mat prevIm, curIm;
    pair<Pose3d,bool> odom;
    odom.second = false;

    double cT=0.04;

    prevIm = prevImage;
    curIm = curImage;

    if(USE_SOBEL)
    {
        prevIm = GetGradient(prevIm);
        curIm = GetGradient(curIm);
    }
    if(USE_NDVI)
    {
        prevIm = prevNDVIMap;
        curIm = currentNDVIMap;
    }
    if(ODOM_TYPE >= 1 && ODOM_TYPE <= 3)
        odom = findOdometryUsingFeatures(prevIm, curIm, cT);

    return odom;
}

pair<Pose3d,bool> DroneRobot::findOdometryUsingFeatures(cv::Mat& prevImage, cv::Mat& curImage, double cT)
{
    switch(ODOM_TYPE)
    {
        case(1)://SURF
            return findOdometryUsingFeaturesSurf(prevImage, curImage, cT);
            break;
        case(2)://SIFT
            return findOdometryUsingFeaturesSift(prevImage, curImage, cT);
            break;
        case(3)://ORB
            return findOdometryUsingFeaturesOrb(prevImage, curImage, cT);
            break;
    }
}

pair<Pose3d,bool> DroneRobot::findOdometryUsingFeaturesSurf(cv::Mat& prevImage, cv::Mat& curImage, double cT)
{
    cv::Mat img_1, img_2;
    cvtColor(prevImage, img_2, CV_BGR2GRAY);
    cvtColor(curImage, img_1, CV_BGR2GRAY);

#ifdef USE_SURF
    cv::Ptr<cv::Feature2D> detector=cv::xfeatures2d::SURF::create();
    cv::Ptr<cv::Feature2D> extractor=cv::xfeatures2d::SURF::create();
#else
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    cv::Ptr<cv::ORB> extractor = detector;
#endif

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;

    detector->detect(img_1,keypoints_1);
    extractor->compute(img_1,keypoints_1,descriptors_1);

    detector->detect(img_2,keypoints_2);
    extractor->compute(img_2,keypoints_2,descriptors_2);

#ifdef USE_SURF
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );
#endif

    double max_dist = 0; double min_dist = 100;

    for( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
        if( dist > max_dist )
            max_dist = dist;
    }

    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= max(2*min_dist, 0.02) )
        {
            good_matches.push_back( matches[i]);
        }
    }

    cv::Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    int minNumPoints = 6;

    std::vector<cv::Point2f> points1, points2;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        points1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
        points2.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H;
    if(good_matches.size()>=minNumPoints)
        H = findHomography( points1, points2, CV_RANSAC );

    std::vector<cv::Point2f> obj_corners(9);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( img_1.cols, 0 );
    obj_corners[2] = cvPoint( img_1.cols, img_1.rows );
    obj_corners[3] = cvPoint( 0, img_1.rows );
    obj_corners[4] = cvPoint( img_1.cols/2, img_1.rows/2 );
    obj_corners[5] = cvPoint( img_1.cols, img_1.rows/2 );
    obj_corners[6] = cvPoint( img_1.cols/2, 0 );
    obj_corners[7] = cvPoint( img_1.cols/2 + img_1.cols/4, img_1.rows/2 + img_1.rows/4 );
    obj_corners[8] = cvPoint( img_1.cols/2 - img_1.cols/4, img_1.rows/2 - img_1.rows/4 );

    std::vector<cv::Point2f> scene_corners(9);

    if(!H.empty()){
        perspectiveTransform( obj_corners, scene_corners, H);

        line( img_matches, scene_corners[0] + cv::Point2f( img_1.cols, 0), scene_corners[1] + cv::Point2f( img_1.cols, 0), cv::Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + cv::Point2f( img_1.cols, 0), scene_corners[2] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + cv::Point2f( img_1.cols, 0), scene_corners[3] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + cv::Point2f( img_1.cols, 0), scene_corners[0] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );

        line( img_matches, scene_corners[4] + cv::Point2f( img_1.cols, 0), scene_corners[5] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 0, 255), 4 );
        line( img_matches, scene_corners[4] + cv::Point2f( img_1.cols, 0), scene_corners[6] + cv::Point2f( img_1.cols, 0), cv::Scalar( 255, 0, 0), 4 );
    }

    Pose3d p;
    bool isGood = false;

    if(!H.empty())
    {
        H = H.inv();
        if(!runQuiet)
            drawMatchedImages(prevImage,curImage,H,cv::MOTION_HOMOGRAPHY);

        p.y = scene_corners[4].x - obj_corners[4].x;
        p.x = -(scene_corners[4].y - obj_corners[4].y);
        p.z = sqrt(pow(scene_corners[7].x - scene_corners[8].x,2) + pow(scene_corners[7].y - scene_corners[8].y,2))/
              sqrt(pow(obj_corners[7].x - obj_corners[8].x,2) + pow(obj_corners[7].y - obj_corners[8].y,2));

        if(p.z < 1.01 && p.z > 0.99) p.z = 1;

        p.yaw = atan2(scene_corners[5].y-scene_corners[4].y,scene_corners[5].x-scene_corners[4].x);
        isGood = true;

        double origDiag = Utils::getNorm(obj_corners[1]-obj_corners[3]);
        double ratioDisp = Utils::getNorm(scene_corners[4]-obj_corners[4])/(origDiag*0.5);
        if(ratioDisp > 0.8)
        {
            cout << "OPS - displacement larger than half diagonal of the image (" << ratioDisp << ")" << endl;
            isGood = false;
        }
        double diag1 = Utils::getNorm(scene_corners[1]-scene_corners[3]);
        double diag2 = Utils::getNorm(scene_corners[0]-scene_corners[2]);
        double ratioModDiag, ratioDiag;
        if(diag1>diag2)
        {
            ratioModDiag = diag1/diag2;
            ratioDiag = diag1/origDiag;
        }
        else
        {
            ratioModDiag = diag2/diag1;
            ratioDiag = diag2/origDiag;
        }
        if(ratioModDiag > 1.4 || fabs(ratioDiag-1.0) > 0.4)
        {
            cout << "OPS - diag sizes not similar between themselves (" << ratioModDiag
                 <<  ") or to the original image (" << ratioDiag << ")" << endl;
            isGood = false;
        }

        double angle = Utils::getDiffAngle(scene_corners[5]-scene_corners[4], scene_corners[6]-scene_corners[4]);
        if(fabs(angle - 90.0) > 25.0)
        {
            cout << "OPS - angle between the axes different than 90° (" << angle << ")" << endl;
            isGood = false;
        }

        double a = Utils::getNorm(scene_corners[1]-scene_corners[0]);
        double b = Utils::getNorm(scene_corners[2]-scene_corners[1]);
        double c = Utils::getNorm(scene_corners[3]-scene_corners[2]);
        double d = Utils::getNorm(scene_corners[0]-scene_corners[3]);
        double ratioX = std::max(a/c,c/a);
        double ratioY = std::max(b/d,d/b);
        if(ratioX > 1.8 || ratioY > 1.8)
        {
            cout << "OPS - ratio between oposed sides of the projected image not around 1.0 (" << ratioX << "," << ratioY << ")" << endl;
            isGood = false;
        }
        cout << "Odometry - x: " << p.x << " y: " << p.y << " z: " << p.z << " yaw: " << p.yaw << endl;
    }

    if(!runQuiet)
        imshow( "Good Matches", img_matches );

    return pair<Pose3d,bool>(p,isGood);
}

pair<Pose3d,bool> DroneRobot::findOdometryUsingFeaturesSift(cv::Mat& prevImage, cv::Mat& curImage, double cT)
{
    cv::Mat img_1, img_2;

    cvtColor(prevImage, img_2, CV_BGR2GRAY);
    cvtColor(curImage, img_1, CV_BGR2GRAY);

    cv::Ptr<cv::Feature2D> detector=cv::xfeatures2d::SIFT::create();
    cv::Ptr<cv::Feature2D> extractor=cv::xfeatures2d::SIFT::create();

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;

    detector->detect(img_1,keypoints_1);
    extractor->compute(img_1,keypoints_1,descriptors_1);

    detector->detect(img_2,keypoints_2);
    extractor->compute(img_2,keypoints_2,descriptors_2);

    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    for( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
        if( dist > max_dist )
            max_dist = dist;
    }

    std::vector< cv::DMatch > good_matches;
    for( int i = 0; i < matches.size(); i++)
    {
        if( matches[i].distance <= max(2*min_dist, 0.02) )
        {
            good_matches.push_back( matches[i]);
        }
    }

    cv::Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    int minNumPoints = 6;

    std::vector<cv::Point2f> points1, points2;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        points1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
        points2.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H;
    if(good_matches.size()>=minNumPoints)
        H = findHomography( points1, points2, CV_RANSAC );

    std::vector<cv::Point2f> obj_corners(9);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( img_1.cols, 0 );
    obj_corners[2] = cvPoint( img_1.cols, img_1.rows );
    obj_corners[3] = cvPoint( 0, img_1.rows );
    obj_corners[4] = cvPoint( img_1.cols/2, img_1.rows/2 );
    obj_corners[5] = cvPoint( img_1.cols, img_1.rows/2 );
    obj_corners[6] = cvPoint( img_1.cols/2, 0 );
    obj_corners[7] = cvPoint( img_1.cols/2 + img_1.cols/4, img_1.rows/2 + img_1.rows/4 );
    obj_corners[8] = cvPoint( img_1.cols/2 - img_1.cols/4, img_1.rows/2 - img_1.rows/4 );

    std::vector<cv::Point2f> scene_corners(9);

    if(!H.empty())
    {
        perspectiveTransform( obj_corners, scene_corners, H);

        line( img_matches, scene_corners[0] + cv::Point2f( img_1.cols, 0), scene_corners[1] + cv::Point2f( img_1.cols, 0), cv::Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + cv::Point2f( img_1.cols, 0), scene_corners[2] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + cv::Point2f( img_1.cols, 0), scene_corners[3] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + cv::Point2f( img_1.cols, 0), scene_corners[0] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );

        line( img_matches, scene_corners[4] + cv::Point2f( img_1.cols, 0), scene_corners[5] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 0, 255), 4 );
        line( img_matches, scene_corners[4] + cv::Point2f( img_1.cols, 0), scene_corners[6] + cv::Point2f( img_1.cols, 0), cv::Scalar( 255, 0, 0), 4 );
    }

    Pose3d p;
    bool isGood = false;

    if(!H.empty())
    {
        H = H.inv();
        if(!runQuiet)
            drawMatchedImages(prevImage,curImage,H,cv::MOTION_HOMOGRAPHY);

        p.y = scene_corners[4].x - obj_corners[4].x;
        p.x = -(scene_corners[4].y - obj_corners[4].y);
        p.z = sqrt(pow(scene_corners[7].x - scene_corners[8].x,2) + pow(scene_corners[7].y - scene_corners[8].y,2))/
              sqrt(pow(obj_corners[7].x - obj_corners[8].x,2) + pow(obj_corners[7].y - obj_corners[8].y,2));

        if(p.z < 1.01 && p.z > 0.99) p.z = 1;

        p.yaw = atan2(scene_corners[5].y-scene_corners[4].y,scene_corners[5].x-scene_corners[4].x);
        isGood = true;

        double origDiag = Utils::getNorm(obj_corners[1]-obj_corners[3]);
        double ratioDisp = Utils::getNorm(scene_corners[4]-obj_corners[4])/(origDiag*0.5);
        if(ratioDisp > 0.8){
            cout << "OPS - displacement larger than half diagonal of the image (" << ratioDisp << ")" << endl;
            isGood = false;
        }

        double diag1 = Utils::getNorm(scene_corners[1]-scene_corners[3]);
        double diag2 = Utils::getNorm(scene_corners[0]-scene_corners[2]);
        double ratioModDiag, ratioDiag;

        if(diag1>diag2)
        {
            ratioModDiag = diag1/diag2;
            ratioDiag = diag1/origDiag;
        }
        else
        {
            ratioModDiag = diag2/diag1;
            ratioDiag = diag2/origDiag;
        }

        if(ratioModDiag > 1.4 || fabs(ratioDiag-1.0) > 0.4)
        {
            cout << "OPS - diag sizes not similar between themselves (" << ratioModDiag
                 <<  ") or to the original image (" << ratioDiag << ")" << endl;
            isGood = false;
        }

        double angle = Utils::getDiffAngle(scene_corners[5]-scene_corners[4], scene_corners[6]-scene_corners[4]);
        if(fabs(angle - 90.0) > 25.0)
        {
            cout << "OPS - angle between the axes different than 90° (" << angle << ")" << endl;
            isGood = false;
        }

        double a = Utils::getNorm(scene_corners[1]-scene_corners[0]);
        double b = Utils::getNorm(scene_corners[2]-scene_corners[1]);
        double c = Utils::getNorm(scene_corners[3]-scene_corners[2]);
        double d = Utils::getNorm(scene_corners[0]-scene_corners[3]);
        double ratioX = std::max(a/c,c/a);
        double ratioY = std::max(b/d,d/b);

        if(ratioX > 1.8 || ratioY > 1.8)
        {
            cout << "OPS - ratio between oposed sides of the projected image not around 1.0 (" << ratioX << "," << ratioY << ")" << endl;
            isGood = false;
        }
        cout << "Odometry - x: " << p.x << " y: " << p.y << " z: " << p.z << " yaw: " << p.yaw << endl;
    }

    if(!runQuiet)
        imshow( "Good Matches", img_matches );

    return pair<Pose3d,bool>(p,isGood);
}

pair<Pose3d,bool> DroneRobot::findOdometryUsingFeaturesOrb(cv::Mat& prevImage, cv::Mat& curImage, double cT)
{
    cv::Mat img_1, img_2;

    cvtColor(prevImage, img_2, CV_BGR2GRAY);
    cvtColor(curImage, img_1, CV_BGR2GRAY);

    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    cv::Ptr<cv::ORB> extractor = detector;

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;

    detector->detect(img_1,keypoints_1);
    extractor->compute(img_1,keypoints_1,descriptors_1);

    detector->detect(img_2,keypoints_2);
    extractor->compute(img_2,keypoints_2,descriptors_2);

    cv::BFMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match(descriptors_1, descriptors_2, matches);

    double max_dist = 0; double min_dist = 100;

    for( int i = 0; i < matches.size(); i++)
    {
        double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
        if( dist > max_dist )
            max_dist = dist;
    }

    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < matches.size(); i++)
    {
        if( matches[i].distance <= max(2*min_dist, 0.02) )
        {
            good_matches.push_back( matches[i]);
        }
    }

    cv::Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    int minNumPoints = 6;

    std::vector<cv::Point2f> points1, points2;
    for( int i = 0; i < good_matches.size(); i++)
    {
        points1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
        points2.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H;
    if(good_matches.size()>=minNumPoints)
        H = findHomography( points1, points2, CV_RANSAC );

    std::vector<cv::Point2f> obj_corners(9);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( img_1.cols, 0 );
    obj_corners[2] = cvPoint( img_1.cols, img_1.rows );
    obj_corners[3] = cvPoint( 0, img_1.rows );
    obj_corners[4] = cvPoint( img_1.cols/2, img_1.rows/2 );
    obj_corners[5] = cvPoint( img_1.cols, img_1.rows/2 );
    obj_corners[6] = cvPoint( img_1.cols/2, 0 );
    obj_corners[7] = cvPoint( img_1.cols/2 + img_1.cols/4, img_1.rows/2 + img_1.rows/4 );
    obj_corners[8] = cvPoint( img_1.cols/2 - img_1.cols/4, img_1.rows/2 - img_1.rows/4 );

    std::vector<cv::Point2f> scene_corners(9);

    if(!H.empty())
    {
        perspectiveTransform( obj_corners, scene_corners, H);

        line( img_matches, scene_corners[0] + cv::Point2f( img_1.cols, 0), scene_corners[1] + cv::Point2f( img_1.cols, 0), cv::Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + cv::Point2f( img_1.cols, 0), scene_corners[2] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + cv::Point2f( img_1.cols, 0), scene_corners[3] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + cv::Point2f( img_1.cols, 0), scene_corners[0] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );

        line( img_matches, scene_corners[4] + cv::Point2f( img_1.cols, 0), scene_corners[5] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 0, 255), 4 );
        line( img_matches, scene_corners[4] + cv::Point2f( img_1.cols, 0), scene_corners[6] + cv::Point2f( img_1.cols, 0), cv::Scalar( 255, 0, 0), 4 );
    }

    Pose3d p;
    bool isGood = false;

    if(!H.empty())
    {
        H = H.inv();
        if(!runQuiet)
            drawMatchedImages(prevImage,curImage,H,cv::MOTION_HOMOGRAPHY);

        p.y = scene_corners[4].x - obj_corners[4].x;
        p.x = -(scene_corners[4].y - obj_corners[4].y);
        p.z = sqrt(pow(scene_corners[7].x - scene_corners[8].x,2) + pow(scene_corners[7].y - scene_corners[8].y,2))/
              sqrt(pow(obj_corners[7].x - obj_corners[8].x,2) + pow(obj_corners[7].y - obj_corners[8].y,2));

        if(p.z < 1.01 && p.z > 0.99) p.z = 1;

        p.yaw = atan2(scene_corners[5].y-scene_corners[4].y,scene_corners[5].x-scene_corners[4].x);
        isGood = true;

        double origDiag = Utils::getNorm(obj_corners[1]-obj_corners[3]);
        double ratioDisp = Utils::getNorm(scene_corners[4]-obj_corners[4])/(origDiag*0.5);
        if(ratioDisp > 0.8)
        {
            cout << "OPS - displacement larger than half diagonal of the image (" << ratioDisp << ")" << endl;
            isGood = false;
        }

        double diag1 = Utils::getNorm(scene_corners[1]-scene_corners[3]);
        double diag2 = Utils::getNorm(scene_corners[0]-scene_corners[2]);
        double ratioModDiag, ratioDiag;

        if(diag1>diag2)
        {
            ratioModDiag = diag1/diag2;
            ratioDiag = diag1/origDiag;
        }
        else
        {
            ratioModDiag = diag2/diag1;
            ratioDiag = diag2/origDiag;
        }

        if(ratioModDiag > 1.4 || fabs(ratioDiag-1.0) > 0.4)
        {
            cout << "OPS - diag sizes not similar between themselves (" << ratioModDiag
                 <<  ") or to the original image (" << ratioDiag << ")" << endl;
            isGood = false;
        }

        double angle = Utils::getDiffAngle(scene_corners[5]-scene_corners[4], scene_corners[6]-scene_corners[4]);

        if(fabs(angle - 90.0) > 25.0)
        {
            cout << "OPS - angle between the axes different than 90° (" << angle << ")" << endl;
            isGood = false;
        }

        double a = Utils::getNorm(scene_corners[1]-scene_corners[0]);
        double b = Utils::getNorm(scene_corners[2]-scene_corners[1]);
        double c = Utils::getNorm(scene_corners[3]-scene_corners[2]);
        double d = Utils::getNorm(scene_corners[0]-scene_corners[3]);
        double ratioX = std::max(a/c,c/a);
        double ratioY = std::max(b/d,d/b);

        if(ratioX > 1.8 || ratioY > 1.8)
        {
            cout << "OPS - ratio between oposed sides of the projected image not around 1.0 (" << ratioX << "," << ratioY << ")" << endl;
            isGood = false;
        }
        cout << "Odometry - x: " << p.x << " y: " << p.y << " z: " << p.z << " yaw: " << p.yaw << endl;
    }

    if(!runQuiet)
        imshow( "Good Matches", img_matches );

    return pair<Pose3d,bool>(p,isGood);
}

int selectMapID(int colorDiff)
{
    switch(colorDiff)
    {
    /// index of mapsColorConverted
    /// 0: RGB, 1: Intensity, 2: LAB
    case INTENSITYC:
        return 1;
    case CIELAB1976:
    case CIELAB1994:
    case CMCLAB1984:
    case CIELAB2000:
    case CIELAB1994MIX:
    case CIELAB2000MIX:
        return 2;
        break;
    default:
        return 0;
    }
}
