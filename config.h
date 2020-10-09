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

#ifndef CONFIG_H
#define CONFIG_H
#include <strings.h>
#include <opencv2/core/core.hpp>

//Geral
extern int pixelDistMethod;      //1 Gaussian, 2 Fitness proportionate selection, 3 Random Pool Selection (Rejection sampling)
extern int msImgPreProc;         //1 Gaussian filter, 2 bilateral filter, 0 without filter
extern int mapPreProc;           //1 quantization, 2 bilateral filtering
extern int ndvi_type;
extern int grvi_type;
extern double minScale;
extern double maxScale;
extern bool multispectralCam;
extern double gbSigma;           // Gaussian Blur - sigma
extern double gbKernel;          // Gaussian Blur - kernel size
extern int bfD;                  // Biltaeral Filter - Diameter of each pixel neighborhood that is used during filtering. If it is non-positive, it is computed from sigmaSpace.
extern double bfSigmaColor;      // Biltaeral Filter - sigma color
extern double bfSigmaSpace;      // Biltaeral Filter - sigma space
extern int poolSelFit;           // 0: defaut operation, 2: disregard all vegetation, 2: disregard common vegetation, working only with the most dense
extern float multiGaussianMask;  // Multiplier of the Gaussian mask used in conjunction with roulette by VI. Ex: with 0.5, the Gaussian mask will only exert 50% influence ...
extern bool quiet;               // Display the graphic interface or not
extern int currentImage;         // Initialized at 0 (currentImage + iniexec + 1 = current image number)
extern double tmpExecPxDist;     // Counts the total execution time of the calls to the pixel distribution.
extern int PxDistCall;    // Number of times the pixel distribution was called.
extern double tmpExecPxCalc;     // Counts the total execution time of the calls to the calculation of the pixel positions.
extern int qtdChamadasPxCalc;    // Number of times the pixel position calculation was called
extern cv::Mat originalMap;      // Original global map, with no processing
extern int rollPitchAngRange;    // Range in which roll and pitch can vary, in particle estimation
extern float alphaVegetationMask;// Used to define the relevance of the vegetation mask, in relation to the vegetation mask.
extern float betaHammingWeight;  // Used to calculate the weight of the pixel pairs, based on the NDVI vegetation mask.


//Constants================
bool const RAW_ODOM           = false;    //Odometry using the Ground Truth data (fake odometry)
int  const NUM_THREADS        = 6;        //Number of Threads
//==========================


// BRIEF HEURISTIC ========
/* 0 - gray                     = 1 bit   (Use in grayscale datasets)
 * 1 - Lab                      = 3 bits
 * 2 - Lab                      = 6 bits
 * 3 - Integer Difference       = Other
 * 4 - Lab wo L                 = 2 bits    //LAB without L
*/
//type = 4=>>> abBRIEF and λ-BRIEF
//type = 0 =>>> BRIEF - grayscale
int const BRIEF_HEURISTIC_TYPE = 4;
//==========================

//Odometry =================
int const ODOM_TYPE = 1; //1 = SURF, 2 = SIFT, 3 = ORB
    //Image pre-processing
    bool const USE_SOBEL = false; //Using Sobel before processing the Odometry.
    bool const USE_NDVI = false;  //Using NDVI before processing the Odometry.
//==========================

//Dataset limits
extern int imgIniDs;
extern int imgIniExec;
extern int imgFimDs;
extern int imgFimExec;

extern double percentualPxVegImg;

#endif // CONFIG_H
