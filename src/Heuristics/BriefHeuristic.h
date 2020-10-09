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

#ifndef BRIEFHEURISTIC_H
#define BRIEFHEURISTIC_H

#include "Heuristic.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "src/VegetationIndex/Ndvi.h"
#include "src/VegetationIndex/VegetationIndex.h"
#include "config.h"
#include "src/Utils/Utils.h"
#include <exception>
#include <thread>
#include "src/VegetationIndex/VegetatonIndexGen.h"

#define FLOAT_MIN -99999.0f
class BriefHeuristic : public Heuristic
{
public:
    BriefHeuristic(STRATEGY s, int id, int cd, double l);
    double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map=NULL);
    double calculateValue2(Pose3d p, cv::Mat *map);
    double calculateValue2(Pose3d p, cv::Mat *map, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal, Pose3d orientacao);
    bool pointIn(cv::Point points, cv::Mat image);
    void updateDroneDescriptor(cv::Mat& drone);
    void updateDroneDescriptor(cv::Mat& drone, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal, Pose3d orientacao, Pose3d realPose);
    void printInfo();
    void CalculateVIMask(cv::Mat &redMap, cv::Mat &nirMap, cv::Mat regMap, cv::Mat greMap, cv::Mat viMatrix);
    void drawPixelDistribution(cv::Mat& localMap);
    void pixelDistribution(cv::Mat& drone, cv::Mat &nirMap,cv::Mat &redMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat viMatrix);
    void gaussianMask(cv::Size &size, cv::Mat &output, int x0, int y0, float sigmaX, float sigmaY, float amplitude = 1.0f);
    int totalPairs;
    float lowThreshold;
    float multiplierThreshold;
    int margin;
    int width, height;
    double totPx;
    vector<int> droneDescriptor;
    int type;
    vector< vector<cv::Point> > pairs;
    cv::Point transform(cv::Point pt, cv::Mat rot, cv::Point trans, int max_x, int max_y);

    //Vegetation Mask
    vector< float > pairWeight;
    vector< vector<float> > pairVI;
    double sumNdviWeightsDrone;
    std::stringstream abBRIEFLog;
};

#endif // BRIEFHEURISTIC_H
