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

#include "BriefHeuristic.h"
#include <time.h>
#include "config.h"
#include <math.h>
#include <random>
#include <iostream>
#include <fstream>
#include <vector>
#include "src/Utils/RadiusVolumeTransferFunctions.h"

BriefHeuristic::BriefHeuristic(STRATEGY s, int id, int cd, double l):Heuristic(s,id,l,cd)
{
    this->totalPairs = 1000;
    this->lowThreshold = 0.5;
    this->multiplierThreshold = 4;
    this->margin = 10;
    this->width = 0;
    this->height = 0;
    this->sumNdviWeightsDrone = 0;

    type = BRIEF_HEURISTIC_TYPE;
    cout<<"\ntype: "<< type;
}

void BriefHeuristic::printInfo()
{
    cout << endl << "Brief parameters:\nPairs:\t\t" << totalPairs << "\nLow Threshold:\t" << lowThreshold << "\nMultiplier:\t" << multiplierThreshold << "\nMargin:\t\t" << margin << endl << endl;
}

//DRONE
void BriefHeuristic::updateDroneDescriptor(cv::Mat& drone, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal, Pose3d orientacao, Pose3d realPose)
{
    this->width = drone.cols;
    this->height = drone.rows;
    this->totPx = width * height;
    cv::Mat viMatrix;

    viMatrix = cv::Mat(drone.rows, drone.cols, CV_32F, 0.0);//VI float

    if(pairs.size() < 1)
    {
        for(int p=0; p<totalPairs; p++)
        {
            vector<cv::Point> pair(2);
            pairs.push_back(pair);

            vector<float> nrpair(2);
            pairVI.push_back(nrpair);

            float pw(1);
            pairWeight.push_back(pw);
        }
    }

    pixelDistribution(drone, nirMap, redMap, regMap, greMap, viMatrix);

    if(!quiet)
        drawPixelDistribution(currentMapOriginal);    

    vector<int> bin;    
    for(int c=0;c<pairs.size();c++)
    {
        vec3 color1(getValuefromPixel(pairs[c][0].x,pairs[c][0].y,&drone));
        vec3 color2(getValuefromPixel(pairs[c][1].x,pairs[c][1].y,&drone));

        //LAB WITH OUT L
        if(color1.r>color2.r) bin.push_back(1);
        else bin.push_back(0);
        if(color1.g>color2.g) bin.push_back(1);
        else bin.push_back(0);
    }

    droneDescriptor = bin;
    this->CalculateVIMask(redMap, nirMap, regMap, greMap, viMatrix);
}

//Two-dimensional Gaussian function
void BriefHeuristic::gaussianMask(cv::Size &size, cv::Mat &output, int x0, int y0, float sigmaX, float sigmaY, float amplitude)
{
    cv::Mat temp = cv::Mat(size, CV_32F);
    float X = 0, Y = 0, value = 0;

    for (int y = 0; y < size.height; y++)
    {
        for (int x = 0; x < size.width; x++)
        {
            X = ((x - x0) * (x - x0)) / (2.0f * sigmaX * sigmaY);
            Y = ((y - y0) * (y - y0)) / (2.0f * sigmaX * sigmaY);
            value = amplitude * exp( -(X + Y));

            temp.at<float>(y, x) = value;
            X = 0; Y = 0; value = 0;
        }
    }
    cv::normalize(temp, temp, 0.0f, 1.0f, cv::NORM_MINMAX);
    output = temp;
}

void BriefHeuristic::pixelDistribution(cv::Mat& drone, cv::Mat &nirMap,cv::Mat &redMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat viMatrix)
{
    VegetationIndex *vegIdx = VegetatonIndexGen::InitVegetationIndex();
    int x = 0, y = 0, pixelPair = 0;
    cv::Point point;    
    double sigmaX = 0;
    double sigmaY = 0;
    clock_t tempo;
    tempo = clock();

    if(pixelDistMethod == 11)
    {
        //Original => BRIEF paper
        sigmaX = 1.0/25.0*pow(drone.cols,2);
        sigmaY = 1.0/25.0*pow(drone.rows,2);
    }
    else
    {
        sigmaX = drone.cols/5;
        sigmaY = drone.rows/5;
    }

    cv::RNG rng(time(NULL) * cv::getTickCount());
    cv::Mat gaussianMsk;
    cv::Size sz = cv::Size(drone.cols, drone.rows);

    //Gaussian Mask
    if(pixelDistMethod == 51 || pixelDistMethod == 7 || pixelDistMethod == 52 || pixelDistMethod == 2)
    {
        gaussianMask(sz, gaussianMsk, drone.cols/2, drone.rows/2, sigmaX, sigmaY);//(float)Valores entre 0 e 1
    }

    switch(pixelDistMethod)
    {
        //Fitness proportionate selection
        case(2)://(OK) Roulette-wheel selection via stochastic acceptance
        {
            int rows = nirMap.rows;
            int cols = nirMap.cols;
            std::vector< std::vector<double> > viMat(rows, std::vector<double>(cols));
            //float viMat[rows][cols];
            double sumScaled = 0;
            double random = 0;
            double maximumFitness = FLOAT_MIN;
            int totPxVeg = 0;
            cv::Mat gray(rows, cols, CV_8UC1);
            cv::Mat grayEqualized(rows, cols, CV_8UC1);
            cv::Mat grayEqualizedVis(rows, cols, CV_8UC1);
            double tempoTotalCalcVI = 0;
            clock_t tempoCalcVI;

            /* initialize random seed: */
            srand (time(NULL) * cv::getTickCount());
            tempoCalcVI = clock();
            int ndviCount = 0, ndviCount2 = 0;

            //Calculation of vegetation index for the entire image
            #pragma omp parallel for num_threads(NUM_THREADS) shared(totPxVeg)
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                   viMat[i][j] = vegIdx->setIndexD(nirMap.at<uchar>(i, j),
                           regMap.at<uchar>(i, j),
                           redMap.at<uchar>(i, j),
                           greMap.at<uchar>(i, j));
                   if(viMat[i][j] >= 0.3)//Great chances of containing vegetation
                      #pragma omp atomic update
                      totPxVeg++;
                   viMatrix.at<float>(i,j) =  viMat[i][j];
                   viMat[i][j] = viMat[i][j] * -1;//Invert the probability, because in the default, large values indicate vegetation, but I want to make the large values refer to the probability of no vegetation.

                   //Scale to a positive range, already converting to what would be expected from an image 0 - 255
                   gray.at<uchar>(i,j) = Utils::Normalize(viMat[i][j], 0, 255, -1, 1);
                }
            }

            //Vegetation Index runtime
            tempoTotalCalcVI = (clock() - tempoCalcVI) / (double)CLOCKS_PER_SEC;
            cout << "VI calculation time: " << tempoTotalCalcVI << " s \n";
            //Calculation of the percentage of vegetation in the image
            percentualPxVegImg = (totPxVeg/(totPx/100))/100;
            cout << "\nPercentage of vegetation: " << percentualPxVegImg << "\n";
            //Image equalization to highlight differences
            cv::equalizeHist(gray, grayEqualized);            
            if(!quiet)
            {
                cv::namedWindow("Vegetation Mask");
                cv::moveWindow("Vegetation Mask", 300, 142 + (0.45 * grayEqualizedVis.rows));
                cv::resize(grayEqualized, grayEqualizedVis, cv::Size(), 0.45, 0.45);
                imshow("Vegetation Mask",grayEqualizedVis);
                imwrite("/home/raziel/Documents/Images/NDVI/NDVI" + std::to_string(currentImage) + ".jpg", grayEqualizedVis);
            }
            //Scale the matrix of VIs and already apply the alpha calculation
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {   //Scale from 0 to 255 to 0 to 1.
                    viMat[i][j] = Utils::Normalize(grayEqualized.at<uchar>(i,j), 0, 1, 0, 255);
                    viMat[i][j] = (alphaVegetationMask * viMat[i][j]) + (1-alphaVegetationMask) * gaussianMsk.at<float>(i, j);
                    sumScaled += viMat[i][j];
                }
            }
            //Normalize the matrix of VIs so that the sum totals 1
            for (int i = 0; i < nirMap.rows; i++)
            {
                for (int j = 0; j < nirMap.cols; j++)
                {
                    viMat[i][j] = viMat[i][j]/sumScaled;

                    if(viMat[i][j] > maximumFitness)
                        maximumFitness = viMat[i][j];
                }
            }            
            //Roulette calculation
            for(int pair = 0; pair < totalPairs; pair++)//Iterate through all pairs of pixels
            {
                pixelPair = 0;
                while(pixelPair < 2)//Iterate through the pair's pixels
                {
                    //Select a pixel
                    x = floor(rand() % drone.cols);
                    y = floor(rand() % drone.rows);

                    random = ((double) rand() / (RAND_MAX));
                    if(random < (viMat[y][x]/maximumFitness))
                    {
                        pairs[pair][pixelPair] = cv::Point(x, y);
                        pixelPair ++;                      
                    }
                }
            }
            break;
        }
        case(3)://Roulette-wheel
        case(31)://Roulette-wheel + Gaussian (float)
              {
                  int rows = nirMap.rows;
                  int cols = nirMap.cols;
                  float viMat[rows][cols];
                  float sumNormalized = 0, sumScaled = 0;
                  float limite = 0, aux = 0;

                  /* initialize random seed: */
                  srand (time(NULL) * cv::getTickCount());

                  cv::Mat gray(rows, cols, CV_8UC1);
                  cv::Mat grayEqualized(rows, cols, CV_8UC1);
                  cv::Mat grayEqualizedVis(rows, cols, CV_8UC1);
                  int totPxVeg = 0;
                  double tempoTotalCalcVI = 0, tempoTotalCalcAlfaBeta = 0, tempoTotalNormalizacao = 0, tempoTotalRoleta = 0;
                  clock_t tempoCalcVI, tempoCalcAlfaBeta, tempoNormalizacao, tempoRoleta;

                  tempoCalcVI = clock();
                  //Calculation of the vegetation index for the entire image
                  #pragma omp parallel for num_threads(NUM_THREADS) shared(totPxVeg)
                  for (int i = 0; i < rows; i++)
                  {
                      for (int j = 0; j < cols; j++)
                      {                          
                         viMat[i][j] = vegIdx->setIndex(nirMap.at<uchar>(i, j),
                                 regMap.at<uchar>(i, j),
                                 redMap.at<uchar>(i, j),
                                 greMap.at<uchar>(i, j));

                         if(viMat[i][j] >= 0.3)// Great chances of containing vegetation
                            #pragma omp atomic update
                            totPxVeg++;


                         viMatrix.at<float>(i,j) =  viMat[i][j];
                         viMat[i][j] = viMat[i][j] * -1;// Invert the probability, because in the default, large values indicate vegetation, but I want to make the large values refer to the probability of no vegetation.


                         // Scale to a positive range, already converting to what would be expected from an image 0 - 255
                         gray.at<uchar>(i,j) = Utils::Normalize(viMat[i][j], 0, 255, -1, 1);   
                      }
                  }

                  tempoTotalCalcVI = (clock() - tempoCalcVI) / (double)CLOCKS_PER_SEC;
                  cout << "Tempo do calculo do VI: " << tempoTotalCalcVI << " s \n";

                  percentualPxVegImg = (totPxVeg/(totPx/100))/100;
                  cout << "\nPercentual de vegetação: " << percentualPxVegImg << "\n";
                  cv::equalizeHist(gray, grayEqualized);

                  if(!quiet)
                  {
                      cv::namedWindow("grayWin");
                      cv::resize(gray, gray, cv::Size(), 0.25, 0.25);
                      cv::imshow("grayWin", gray);
                      cv::namedWindow("grayEqualizedWin");
                      cv::resize(grayEqualized, grayEqualizedVis, cv::Size(), 0.25, 0.25);
                      imshow("grayEqualizedWin",grayEqualizedVis);
                  }

                  tempoCalcAlfaBeta = clock();
                  for (int i = 0; i < rows; i++)
                  {
                      for (int j = 0; j < cols; j++)
                      {   //Escala de 0 a 255 para de 0 a 1.
                          viMat[i][j] = Utils::Normalize(grayEqualized.at<uchar>(i,j), 0, 1, 0, 255);
                          viMat[i][j] = (alphaVegetationMask * viMat[i][j]) + (1-alphaVegetationMask) * gaussianMsk.at<float>(i, j);
                          sumScaled += viMat[i][j];
                      }
                  }
                  tempoTotalCalcAlfaBeta = (clock() - tempoCalcAlfaBeta) / (double)CLOCKS_PER_SEC;
                  cout << "Tempo do calculo alfa beta: " << tempoTotalCalcAlfaBeta << " s \n";

                  tempoNormalizacao = clock();
                  // Normalize the matrix of VIs so that the sum totals 1
                  for (int i = 0; i < rows; i++)
                  {
                      for (int j = 0; j < cols; j++)
                      {
                          viMat[i][j] = viMat[i][j]/sumScaled;
                          sumNormalized += viMat[i][j];
                      }
                  }
                  tempoTotalNormalizacao = (clock() - tempoNormalizacao) / (double)CLOCKS_PER_SEC;
                  cout << "Tempo da Normalizacao: " << tempoTotalNormalizacao << " s \n";

                  tempoRoleta = clock();
                  //Roleta
                  #pragma omp parallel for num_threads(NUM_THREADS) private(aux,limite,pixelPair)
                  for(int pair = 0; pair < totalPairs; pair++)
                  {
                      pixelPair = 0;
                      while(pixelPair < 2)
                      {
                          aux = 0;
                          limite = Utils::RandomFloat(0, sumNormalized);

                          for (int iElem = 0; iElem < rows && limite > aux; iElem++)
                          {
                              for (int jElem = 0; jElem < cols && limite > aux; jElem++)
                              {
                                  aux += viMat[iElem][jElem];// Incrementing here I don't need the loopings to calculate the individual porbility of each item

                                  if(limite <= aux)
                                  {
                                      if(jElem > 0)
                                          pairs[pair][pixelPair] = cv::Point(jElem-1, iElem);
                                      else//Linha anterior
                                          if(iElem > 0)
                                              pairs[pair][pixelPair] = cv::Point(jElem, iElem - 1);
                                          else
                                          {
                                              pairs[pair][pixelPair] = cv::Point(jElem, iElem);
                                              cout<< "Linha e coluna 0.";
                                          }

                                      pixelPair++;

                                      if(pixelPair == 1)
                                      {
                                          iElem = 0; jElem = 0;
                                          limite = Utils::RandomFloat(0, sumNormalized);
                                      }
                                  }

                                  if(pixelPair == 2)// Advance to the next pair of pixels
                                      break;
                              }
                              if(pixelPair == 2)// Advance to the next pair of pixels
                                  break;
                          }
                      }
                  }
                  tempoTotalRoleta = (clock() - tempoRoleta) / (double)CLOCKS_PER_SEC;
                  cout << "Roulette Time: " << tempoTotalRoleta << " s \n";

                  if(!quiet)
                  {
                      //Reverse normalization to improve visualization
                      for (int i = 0; i < rows; i++)
                      {
                          for (int j = 0; j < cols; j++)
                          {
                              viMat[i][j] = viMat[i][j] * sumScaled;
                          }
                      }

                      cv::namedWindow("pixelDistMaskWin");
                      cv::Mat pixelDistMask(rows, cols, CV_32FC1, viMat);
                      cv::resize(pixelDistMask, pixelDistMask, cv::Size(), 0.25, 0.25);
                      cv::imshow("pixelDistMaskWin", pixelDistMask);
                  }
                  break;
              }
        case(4)://(OK) Tournament
        case(41)://(OK) Tournament + Gaussian
        {
            int k = 3; //Tournament size (number of individuals selected at a time)
            int x[k], y[k];
            cv::Point points[k];
            float fitnessScore[k];
            int idxMelhorIndv = 0;

            /* initialize random seed: */
            srand (time(NULL) * cv::getTickCount());

            for(int i = 0; i < totalPairs; i++)
            {
                pixelPair = 0;
                while(pixelPair < 2)
                {
                    for(int individuo = 0; individuo < k; individuo++)
                    {
                        if(pixelDistMethod == 61)
                        {
                            cv::theRNG().state = time(NULL) * cv::getTickCount();

                            //Gaussian Selection
                            x[individuo] = (int)rng.gaussian(sigmaX)+drone.cols/2;
                            y[individuo] = (int)rng.gaussian(sigmaY)+drone.rows/2;
                        }
                        else
                        {
                            srand (time(NULL) * cv::getTickCount());
                            x[individuo] = floor(rand() % drone.cols);
                            y[individuo] = floor(rand() % drone.rows);
                        }

                        points[individuo] = cv::Point(x[individuo], y[individuo]);

                        if(pointIn(points[individuo], drone))
                        {
                            fitnessScore[individuo] = vegIdx->setIndex(nirMap.at<uchar>(y[individuo], x[individuo]),
                                                            regMap.at<uchar>(y[individuo], x[individuo]),
                                                            redMap.at<uchar>(y[individuo], x[individuo]),
                                                            greMap.at<uchar>(y[individuo], x[individuo]));
                            if(individuo > 0)
                            {
                                if(fitnessScore[individuo] < fitnessScore[individuo-1])
                                    idxMelhorIndv = individuo;
                                else
                                    idxMelhorIndv = individuo - 1;
                            }
                        }
                        else
                            individuo --;
                    }
                    pairs[i][pixelPair] = cv::Point(x[idxMelhorIndv], y[idxMelhorIndv]);
                    pixelPair ++;
                }
            }
            break;
        }
        case(1)://Gaussian
        case(11)://Old Gaussian
        default://(OK) Gaussian
        {
            for(int x=0;x<totalPairs;x++)
            {
                do
                {
                    pairs[x][0] = cv::Point((int)rng.gaussian(sigmaX)+drone.cols/2,
                                            (int)rng.gaussian(sigmaY)+drone.rows/2);
                }
                while(!pointIn(pairs[x][0], drone));

                do
                {              
                    pairs[x][1] = cv::Point((int)rng.gaussian(sigmaX)+drone.cols/2,
                                            (int)rng.gaussian(sigmaY)+drone.rows/2);
                }
                while(!pointIn(pairs[x][1], drone));
            }            
            break;
        }
    }

    tmpExecPxDist += (clock() - tempo) / (double)CLOCKS_PER_SEC;
    PxDistCall++;

    cout << "Average execution time of the pixel distribution: " << tmpExecPxDist/PxDistCall << " s \n";
}

void BriefHeuristic::drawPixelDistribution(cv::Mat& localMap)
{
     cv::Mat map = localMap.clone();

    for(int x=0;x<totalPairs;x++)
    {
        cv::rectangle( map, cv::Point2f( pairs[x][0].x-1, pairs[x][0].y-1 ), cv::Point2f( pairs[x][0].x+1, pairs[x][0].y+1 ), cv::Scalar( 0, 0, 255 ) );
        cv::rectangle( map, cv::Point2f( pairs[x][1].x-1, pairs[x][1].y-1 ), cv::Point2f( pairs[x][1].x+1, pairs[x][1].y+1 ), cv::Scalar( 0, 0, 255 ) );

        cv::rectangle( map, cv::Point2f( pairs[x][0].x-2, pairs[x][0].y-2 ), cv::Point2f( pairs[x][0].x+2, pairs[x][0].y+2 ), cv::Scalar( 0, 0, 255 ) );
        cv::rectangle( map, cv::Point2f( pairs[x][1].x-2, pairs[x][1].y-2 ), cv::Point2f( pairs[x][1].x+2, pairs[x][1].y+2 ), cv::Scalar( 0, 0, 255 ) );

        cv::rectangle( map, cv::Point2f( pairs[x][0].x-3, pairs[x][0].y-3 ), cv::Point2f( pairs[x][0].x+3, pairs[x][0].y+3 ), cv::Scalar( 0, 0, 255 ) );
        cv::rectangle( map, cv::Point2f( pairs[x][1].x-3, pairs[x][1].y-3 ), cv::Point2f( pairs[x][1].x+3, pairs[x][1].y+3 ), cv::Scalar( 0, 0, 255 ) );
    }

    cv::namedWindow("Pixel Distribution");
    cv::moveWindow("Pixel Distribution", 300, 70);
    cv::resize(map, map, cv::Size(), 0.45, 0.45);
    imshow("Pixel Distribution", map);
}

//DRONE
void BriefHeuristic::updateDroneDescriptor(cv::Mat& drone)
{
    width = drone.cols;
    height = drone.rows;

    if(pairs.size() < 1){
        for(int p=0; p<totalPairs; p++)
        {
            vector<cv::Point> pair(2);
            pairs.push_back(pair);
        }
    }

    cv::RNG rng;
    for(int x=0;x<totalPairs;x++)
    {
        do{
            pairs[x][0] = cv::Point((int)rng.gaussian(1.0/25.0*pow(drone.cols,2))+drone.cols/2, (int)rng.gaussian(1.0/25.0*pow(drone.rows,2))+drone.rows/2);
        }while(!pointIn(pairs[x][0], drone));
        do{
            pairs[x][1] = cv::Point((int)rng.gaussian(1.0/25.0*pow(drone.cols,2))+drone.cols/2, (int)rng.gaussian(1.0/25.0*pow(drone.rows,2))+drone.rows/2);
        }while(!pointIn(pairs[x][1], drone));
    }

    vector<int> bin;
    for(int c=0;c<pairs.size();c++)
    {
        vec3 color1(getValuefromPixel(pairs[c][0].x,pairs[c][0].y,&drone));
        vec3 color2(getValuefromPixel(pairs[c][1].x,pairs[c][1].y,&drone));

        //Lab with 3 bits
        if(type == 1)
        {
            if(color1.r>color2.r) bin.push_back(1);
            else bin.push_back(0);
            if(color1.g>color2.g) bin.push_back(1);
            else bin.push_back(0);
            if(color1.b>color2.b) bin.push_back(1);
            else bin.push_back(0);
        }

        //Lab with 6 bits
        else if(type == 2)
        {
            if(color1.r > color2.r){
                bin.push_back(1);
                bin.push_back(0);
            }else if(color1.r < color2.r){
                bin.push_back(0);
                bin.push_back(1);
            }else{
                bin.push_back(0);
                bin.push_back(0);
            }
            if(color1.g > color2.g){
                bin.push_back(1);
                bin.push_back(0);
            }else if(color1.g < color2.g){
                bin.push_back(0);
                bin.push_back(1);
            }else{
                bin.push_back(0);
                bin.push_back(0);
            }
            if(color1.b > color2.b){
                bin.push_back(1);
                bin.push_back(0);
            }else if(color1.b < color2.b){
                bin.push_back(0);
                bin.push_back(1);
            }else{
                bin.push_back(0);
                bin.push_back(0);
            }
        }

        //GRAY
        else if(type == 0)
        {
            float r = color1.b * 0.114;//B
            float g = color1.g * 0.587;//G
            float b = color1.r * 0.299;//R

            int sum1 = r + g + b;

            r = color2.b * 0.114;//B
            g = color2.g * 0.587;//G
            b = color2.r * 0.299;//R

            int sum2 = r + g + b;

            if(sum1 > sum2) bin.push_back(1);
            else bin.push_back(0);
        }

        else if(type == 3)
        {
            int diff = (color1.r-color2.r);
            bin.push_back(diff);
            diff = (color1.g-color2.g);
            bin.push_back(diff);
            diff = (color1.b-color2.b);
            bin.push_back(diff);
        }
        //LAB WITH OUT L
        else if(type == 4)
        {
            if(color1.r>color2.r) bin.push_back(1);
            else bin.push_back(0);
            if(color1.g>color2.g) bin.push_back(1);
            else bin.push_back(0);
        }
    }
    droneDescriptor = bin;
}

// Method responsible for calculating the weight mask of the drone descriptor, based on a vegetation index
void BriefHeuristic::CalculateVIMask(cv::Mat &redMap, cv::Mat &nirMap, cv::Mat regMap, cv::Mat greMap, cv::Mat viMatrix)
{
    VegetationIndex *vegIdx = VegetatonIndexGen::InitVegetationIndex();
    sumNdviWeightsDrone = 0;

    for(int i= 0; i < totalPairs; i++)
    {
        if(pixelDistMethod != 51 && pixelDistMethod != 52 && pixelDistMethod != 2)//Nas roletas (51, 52 e 2), o NDVI já é calculado para toda a imagem
        {
            //Calculate the VI for the pair
            //Px 1
            pairVI[i][0] = vegIdx->setIndex(nirMap.at<uchar>(pairs[i][0].y, pairs[i][0].x),
                    regMap.at<uchar>(pairs[i][0].y, pairs[i][0].x),
                    redMap.at<uchar>(pairs[i][0].y, pairs[i][0].x),
                    greMap.at<uchar>(pairs[i][0].y, pairs[i][0].x));

            //Px 2
            pairVI[i][1] = vegIdx->setIndex(nirMap.at<uchar>(pairs[i][1].y, pairs[i][1].x),
                    regMap.at<uchar>(pairs[i][1].y, pairs[i][1].x),
                    redMap.at<uchar>(pairs[i][1].y, pairs[i][1].x),
                    greMap.at<uchar>(pairs[i][1].y, pairs[i][1].x));
        }
        else
        {            
            pairVI[i][0] = viMatrix.at<float>(pairs[i][0].y,pairs[i][0].x);
            pairVI[i][1] = viMatrix.at<float>(pairs[i][1].y,pairs[i][1].x);
        }

        // Scale the NDVI sum of the pair's pixels, from -2 to 2, to 0 to 1
        pairWeight[i] = Utils::Normalize((pairVI[i][0] + pairVI[i][1])*-1, 0, 1, -2, 2);//Multiplico a soma do NDVI por -1 para fazer  com que a grande concentração de vegetação tenda a 2, ao invés de -2. Assim ao escalar para 0-1, muita vegetação ficará próximo de 0.

        //(NDVI * weight) + 1
        //((NDVIpx1 + NDVIpx2) * weight) 1
        pairWeight[i] = (pairWeight[i] * betaHammingWeight) + 1;
        // 1 is the minimum value that a pair of pixels can take (maximum vegetation)
        // while the maximum value a pixel can take is 2 (no vegetation)
        // Since beta ranges from 0 to 1
        sumNdviWeightsDrone += pairWeight[i];
    }
}

//MAP
//With multispectral camera
double BriefHeuristic::calculateValue2(Pose3d p, cv::Mat *map, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal, Pose3d orientacao)
{
    vector<int> bin;
    clock_t tempo;
    tempo = clock();

    int bitsPerPair;
    switch (type)
    {
        case 0:
            bitsPerPair = 1;
            break;
        case 1:
            bitsPerPair = 3;
            break;
        case 2:
            bitsPerPair = 6;
            break;
        case 4:
            bitsPerPair = 2;
            break;
    }

    double scale = p.z;
    float theta = p.yaw*(-1)-1.5708;//-1.5708 = -90 graus
    int xPad = width/2 - 0.5;
    int yPad = height/2 - 0.5;

    double dir[4];//Initial direction (Pointing p down) [x y z 0], and located at the origin
    double cosPitch;
    double sinPitch;
    double cosYaw;
    double sinYaw;
    double cosRoll;
    double sinRoll;
    double particleYaw = p.yaw*(-1)-1.5708;//-1.5708 = -90 degrees
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);

    for(int c=0; c<pairs.size(); c++)
    {
        // Projection of the pairs of points of the drone descriptor, in the "patch" of the map.
        // Rotation and scale
        // Pixel 1 (projects the pair point on the ground - it would be what the particle sees with zero roll and pitch, only with the yaw)
        cv::Point point1 = cv::Point(((pairs[c][0].x-xPad)*scale*cosTheta+(pairs[c][0].y-yPad)*scale*sinTheta)+p.x,
                                    ((pairs[c][0].x-xPad)*scale*-sinTheta+(pairs[c][0].y-yPad)*scale*cosTheta)+p.y);

        if(point1.x < 0 || point1.x >= map->cols || point1.y < 0 || point1.y >= map->rows)
        {
            bin.push_back(HEURISTIC_UNDEFINED_INT);
            bin.push_back(HEURISTIC_UNDEFINED_INT);
            bin.push_back(HEURISTIC_UNDEFINED_INT);
        }
        else
        {
            //Pixel 2
            cv::Point point2 = cv::Point(((pairs[c][1].x-xPad)*scale*cosTheta+(pairs[c][1].y-yPad)*scale*sinTheta)+p.x,
                    ((pairs[c][1].x-xPad)*scale*-sinTheta+(pairs[c][1].y-yPad)*scale*cosTheta)+p.y);

            if(point2.x < 0 || point2.x >= map->cols || point2.y < 0 || point2.y >= map->rows){
                bin.push_back(HEURISTIC_UNDEFINED_INT);
                bin.push_back(HEURISTIC_UNDEFINED_INT);
                bin.push_back(HEURISTIC_UNDEFINED_INT);
            }
            else
            {
                cv::Vec3b color1 = map->at<cv::Vec3b>(point1.y, point1.x);
                cv::Vec3b color2 = map->at<cv::Vec3b>(point2.y, point2.x);

                //Lab
                if(type == 1)
                {
                    if(color1[0]>color2[0]) bin.push_back(1);
                    else bin.push_back(0);
                    if(color1[1]>color2[1]) bin.push_back(1);
                    else bin.push_back(0);
                    if(color1[2]>color2[2]) bin.push_back(1);
                    else bin.push_back(0);
                }

                else if(type == 2)
                {
                    if(color1[0] > color2[0]){
                        bin.push_back(1);
                        bin.push_back(0);
                    }else if(color1[0] < color2[0]){
                        bin.push_back(0);
                        bin.push_back(1);
                    }else{
                        bin.push_back(0);
                        bin.push_back(0);
                    }
                    if(color1[1] > color2[1]){
                        bin.push_back(1);
                        bin.push_back(0);
                    }else if(color1[1] < color2[1]){
                        bin.push_back(0);
                        bin.push_back(1);
                    }else{
                        bin.push_back(0);
                        bin.push_back(0);
                    }
                    if(color1[2] > color2[2]){
                        bin.push_back(1);
                        bin.push_back(0);
                    }else if(color1[2] < color2[2]){
                        bin.push_back(0);
                        bin.push_back(1);
                    }else{
                        bin.push_back(0);
                        bin.push_back(0);
                    }
                }

                //GRAY
                else if(type == 0)
                {
                    float r = color1.val[0] * 0.114;//B
                    float g = color1.val[1] * 0.587;//G
                    float b = color1.val[2] * 0.299;//R

                    int sum1 = r + g + b;

                    r = color2.val[0] * 0.114;//B
                    g = color2.val[1] * 0.587;//G
                    b = color2.val[2] * 0.299;//R

                    int sum2 = r + g + b;

                    if(sum1 > sum2) bin.push_back(1);
                    else bin.push_back(0);
                }

                else if(type == 3)
                {
                    int diff = (color1[0]-color2[0]);
                    bin.push_back(diff);
                    diff = (color1[1]-color2[1]);
                    bin.push_back(diff);
                    diff = (color1[2]-color2[2]);
                    bin.push_back(diff);
                }
                //LAB WITH OUT L
                else if(type == 4)
                {
                    if(color1[0]>color2[0]) bin.push_back(1);
                    else bin.push_back(0);
                    if(color1[1]>color2[1]) bin.push_back(1);
                    else bin.push_back(0);
                }
            }
        }
    }

    // BRIEF calculation of the difference between the drone descriptor and the patch descriptor
    double result = 0;
    double sumNdviWeightsPatch = 0;
    int pairCount = 0;
    int droneImageUndefined = 0;

    int diff = 0;

    for(int i = 0; i < droneDescriptor.size(); i++)
    {
        if(droneDescriptor[i] == HEURISTIC_UNDEFINED_INT
                || bin[i] == HEURISTIC_UNDEFINED_INT
                || bin[i] != droneDescriptor[i])
        {
            diff++;
            sumNdviWeightsPatch += pairWeight[pairCount];
        }
        if(droneDescriptor[i] == HEURISTIC_UNDEFINED_INT)
        {
            droneImageUndefined++;
            sumNdviWeightsPatch -= pairWeight[pairCount];
        }
        if(i % 2 != 0)
            pairCount++;
    }

    diff -= droneImageUndefined;

    sumNdviWeightsPatch = (this->sumNdviWeightsDrone * bitsPerPair) - sumNdviWeightsPatch;

    result = 1.0 - ((float)sumNdviWeightsPatch / ((float)this->sumNdviWeightsDrone * (float)bitsPerPair));

    tmpExecPxCalc += (clock() - tempo) / (double)CLOCKS_PER_SEC;
    qtdChamadasPxCalc++;
    return result;
}

//Without multispectral camera
double BriefHeuristic::calculateValue2(Pose3d p, cv::Mat *map)
{
    vector<int> bin;

    int bitsPerPair;
    switch (type)
    {
        case 0:
            bitsPerPair = 1;
            break;
        case 1:
            bitsPerPair = 3;
            break;
        case 2:
            bitsPerPair = 6;
            break;
        case 4:
            bitsPerPair = 2;
            break;
    }

    double scale = p.z;
    float theta = p.yaw*(-1)-1.5708;
    int xPad = width/2 - 0.5;
    int yPad = height/2 - 0.5;

    for(int c=0;c<pairs.size();c++)
    {
        // Projection of the pairs of points of the drone descriptor, in the "patch" of the map.
        // Rotation and scale
        // Pixel 1
        cv::Point point1 = cv::Point(((pairs[c][0].x-xPad)*scale*cos(theta)+(pairs[c][0].y-yPad)*scale*sin(theta))+p.x,
                                    ((pairs[c][0].x-xPad)*scale*-sin(theta)+(pairs[c][0].y-yPad)*scale*cos(theta))+p.y);

        if(point1.x < 0 || point1.x >= map->cols || point1.y < 0 || point1.y >= map->rows)
        {
            bin.push_back(HEURISTIC_UNDEFINED_INT);
            bin.push_back(HEURISTIC_UNDEFINED_INT);
            bin.push_back(HEURISTIC_UNDEFINED_INT);
        }
        else
        {
            //Pixel 2
            cv::Point point2 = cv::Point(((pairs[c][1].x-xPad)*scale*cos(theta)+(pairs[c][1].y-yPad)*scale*sin(theta))+p.x,
                    ((pairs[c][1].x-xPad)*scale*-sin(theta)+(pairs[c][1].y-yPad)*scale*cos(theta))+p.y);

            if(point2.x < 0 || point2.x >= map->cols || point2.y < 0 || point2.y >= map->rows){
                bin.push_back(HEURISTIC_UNDEFINED_INT);
                bin.push_back(HEURISTIC_UNDEFINED_INT);
                bin.push_back(HEURISTIC_UNDEFINED_INT);
            }
            else{
                cv::Vec3b color1 = map->at<cv::Vec3b>(point1.y, point1.x);
                cv::Vec3b color2 = map->at<cv::Vec3b>(point2.y, point2.x);

                //Lab
                if(type == 1)
                {
                    if(color1[0]>color2[0]) bin.push_back(1);
                    else bin.push_back(0);
                    if(color1[1]>color2[1]) bin.push_back(1);
                    else bin.push_back(0);
                    if(color1[2]>color2[2]) bin.push_back(1);
                    else bin.push_back(0);
                }


                else if(type == 2)
                {
                    if(color1[0] > color2[0])
                    {
                        bin.push_back(1);
                        bin.push_back(0);
                    }else if(color1[0] < color2[0])
                    {
                        bin.push_back(0);
                        bin.push_back(1);
                    }else
                    {
                        bin.push_back(0);
                        bin.push_back(0);
                    }
                    if(color1[1] > color2[1])
                    {
                        bin.push_back(1);
                        bin.push_back(0);
                    }
                    else if(color1[1] < color2[1])
                    {
                        bin.push_back(0);
                        bin.push_back(1);
                    }else
                    {
                        bin.push_back(0);
                        bin.push_back(0);
                    }
                    if(color1[2] > color2[2])
                    {
                        bin.push_back(1);
                        bin.push_back(0);
                    }
                    else if(color1[2] < color2[2])
                    {
                        bin.push_back(0);
                        bin.push_back(1);
                    }
                    else
                    {
                        bin.push_back(0);
                        bin.push_back(0);
                    }
                }

                //GRAY
                else if(type == 0)
                {
                    float r = color1.val[0] * 0.114;//B
                    float g = color1.val[1] * 0.587;//G
                    float b = color1.val[2] * 0.299;//R

                    int sum1 = r + g + b;

                    r = color2.val[0] * 0.114;//B
                    g = color2.val[1] * 0.587;//G
                    b = color2.val[2] * 0.299;//R

                    int sum2 = r + g + b;

                    if(sum1 > sum2) bin.push_back(1);
                    else bin.push_back(0);
                }

                else if(type == 3)
                {
                    int diff = (color1[0]-color2[0]);
                    bin.push_back(diff);
                    diff = (color1[1]-color2[1]);
                    bin.push_back(diff);
                    diff = (color1[2]-color2[2]);
                    bin.push_back(diff);
                }
                //LAB WITH OUT L
                else if(type == 4)
                {
                    if(color1[0]>color2[0]) bin.push_back(1);
                    else bin.push_back(0);
                    if(color1[1]>color2[1]) bin.push_back(1);
                    else bin.push_back(0);
                }
            }
        }
    }
    // abBRIEF calculating the difference between the drone descriptor and the patch descriptor
    if(type == 3)
    {
        int diff = 0;
        double prob = 1;
        float saturation = 25;
        float varProb = 10;
        for(int i= 0; i < droneDescriptor.size(); i++)
        {
            if(droneDescriptor[i] == HEURISTIC_UNDEFINED_INT || bin[i] == HEURISTIC_UNDEFINED_INT)
                diff+=saturation;
            else
            {
                int diff = abs(droneDescriptor[i]-bin[i]);
                if(diff>saturation) diff=saturation;
            }

            double auxD = exp(-0.5*(pow(diff,2.0)/varProb));
            prob *= auxD;
        }

        return prob;
    }
    // BRIEF calculation of the difference between the drone descriptor and the patch descriptor
    else
    {
        int diff = 0;
        int droneImageUndefined = 0;
        double result = 0;
        for(int i= 0; i < droneDescriptor.size(); i++){
            if(droneDescriptor[i] == HEURISTIC_UNDEFINED_INT || bin[i] == HEURISTIC_UNDEFINED_INT || bin[i] != droneDescriptor[i]) diff++;
            if(droneDescriptor[i] == HEURISTIC_UNDEFINED_INT) droneImageUndefined++;
        }

        diff -= droneImageUndefined;
        if(color_difference != INTENSITYC)
            result = 1.0 - ((float)((totalPairs*bitsPerPair)-diff)/(float)(totalPairs*bitsPerPair));
        else
            result = 1.0 - ((float)((totalPairs)-diff)/(float)(totalPairs));
        return result;
    }
}

cv::Point BriefHeuristic::transform(cv::Point pt, cv::Mat rot, cv::Point trans, int max_x, int max_y)
{
    cv::Mat res(1,2,CV_64F);

    res.at<double>(0,0)=pt.x;
    res.at<double>(0,1)=pt.y;

    cv::Mat dst = res*rot;
    cv::Point point = cv::Point(dst.at<double>(0,0)+trans.x,dst.at<double>(0,1)+trans.y);

    if(point.x < 0) point.x = 0;
    else if(point.x > max_x) point.x = max_x;
    if(point.y < 0) point.y = 0;
    else if(point.y > max_y) point.y = max_y;

    return point;
}

double BriefHeuristic::calculateValue(int x, int y, cv::Mat *image, cv::Mat* map)
{
    return 0;
}

bool BriefHeuristic::pointIn(cv::Point points, cv::Mat image)
{
    if(points.x < image.cols && points.x >= 0 && points.y < image.rows && points.y >= 0)
        return true;
    return false;
}
