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

#ifndef DRONEROBOT_H
#define DRONEROBOT_H

#define USE_SURF 1

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "src/Robot/Robot.h"
#include "src/Heuristics/BriefHeuristic.h"
#include "src/Utils/Utils.h"
#include "config.h"
#include "src/Utils/RadiusVolumeTransferFunctions.h"

int selectMapID(int colorDiff);

// Creating a struct for heuristic config
struct heuristicType{
    // variables
    STRATEGY strategy;
    int kernelType;
    double radius;
    int colorDifference;
    double threshold;

    //brief variables
    float lowThreshold;
    float multiplierThreshold;
    int numberPairs;
    int margin;

    //unscented variable
    int unscentedNumberPoints;

    // constructor
    heuristicType(STRATEGY s = BRIEF, int kt=KGAUSSIAN, double r=1.0, int cd=-1, double t=2.3, float blt = -1, float bmt = -1, int bp = -1, int bm = -1, int unp = 1) :
        strategy(s),
        kernelType(kt),
        radius(r),
        colorDifference(cd),
        threshold(t),
        lowThreshold(blt),
        multiplierThreshold(bmt),
        numberPairs(bp),
        margin(bm),
        unscentedNumberPoints(unp)
    {}
};

class DroneRobot: public Robot
{
public:
    DroneRobot();
    DroneRobot(string& mapPath, string& trajectoryPath, vector< heuristicType* > &heuristicTypes, bool quiet, string& outputName, int start, int finish);
    ~DroneRobot();

    void initialize(ConnectionMode cmode, LogMode lmode, string fname, int numParticles);
    void run();

private:

    int getNextProperHeuristicID(STRATEGY type);
    void generateObservations(string imagePath);
    bool readRawOdometryFromFile(Pose3d& p);
    Pose3d readOdometry();
    pair<Pose3d, bool> readOdometryNew();    
    Pose3d readGroundTruth();
    void readCorrectedGroundTruth();
    pair<Pose3d, bool> findOdometry(cv::Mat &prevImage, cv::Mat &curImage);
    pair<Pose3d, bool> findOdometryUsingFeatures(cv::Mat &prevImage, cv::Mat &curImage, double cT=0.04);
    pair<Pose3d,bool>  findOdometryUsingFeaturesSurf(cv::Mat& prevImage, cv::Mat& curImage, double cT);
    pair<Pose3d,bool>  findOdometryUsingFeaturesSift(cv::Mat& prevImage, cv::Mat& curImage, double cT);
    pair<Pose3d,bool>  findOdometryUsingFeaturesOrb(cv::Mat& prevImage, cv::Mat& curImage, double cT);
    cv::Mat GetGradient(cv::Mat src_gray);
    cv::Mat quantization(cv::Mat &curImage, float value);
    cv::Mat msImagePreProcessing(cv::Mat &img);
    void drawMatchedImages(cv::Mat& prevImage, cv::Mat& curImage, const cv::Mat& warp_matrix, const int warp_mode = cv::MOTION_EUCLIDEAN);
    void reinitialize();
    void readDroneAngles();

    bool slowMethod;
    bool offlineOdom;
    bool isRawOdom;
    bool availableGTruth;
    bool availableCorrectedGTruth;
    bool availableAngles;
    bool isBRIEF;
    Pose3d prevRawOdom;
    Pose3d prevOdometry;
    Pose3d realPose;
    fstream odomFile;
    fstream truthFile;
    fstream corrTruthFile;
    fstream anglesFile;
    vector<Pose3d>droneAngles;
    Pose3d curentAngles;
    vector<Pose3d>CorrectedGT;

    int start;
    int finish;
    int current;
    string rawname;
    string outputName;

    STRATEGY locTechnique;

    // Heuristics vectors
    vector<Heuristic*> heuristics;
    vector<MapGrid*> cachedMaps;
    vector<cv::Mat> globalMaps;
    vector<string> imagesNames;

    //As imagens RED, NIR e REG serão utilizadas no cálculo dos diferentes índices, inclusive o NDVI
    vector<string> imageNamesRED;
    vector<string> imageNamesNIR;
    vector<string> imageNamesREG;
    vector<string> imageNamesGRE;

    // Used for feature matching
    cv::Ptr<cv::Feature2D> feature_detector;
    cv::Ptr<cv::Feature2D> feature_extractor;
    cv::FlannBasedMatcher feature_matcher;
    std::vector<cv::KeyPoint> keypoints_globalMap;
    cv::Mat descriptors_globalMap;
    vector< vector<cv::FlannBasedMatcher*> > hMatcher;
    vector< vector< vector<unsigned int>* > > idKeypoints;
    cv::Mat likelihood;

    cv::Mat prevMap;
    cv::Mat currentNIRMap;
    cv::Mat currentREDMap;
    cv::Mat currentREGMap;
    cv::Mat currentGREMap;
    cv::Mat currentNDVIMap;
    cv::Mat prevNDVIMap;

    unsigned int step;
    int numQuantization;

    bool flPrimeiraLeituraGTRawOdom; //Quando for usado raw odom, esse flag controla a sincronia entre a Odometry e o gorund truth, pq senão ocorre de o GT ficar uma poe a frente da Odometry.

};
#endif // DRONEROBOT_H
