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

#ifndef VEGETATIONINDEX_H
#define VEGETATIONINDEX_H

#include<iostream>
#include<stdio.h>
#include<malloc.h>
#include <stdlib.h>
#include <stdio.h>
#include<opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <fstream>
#include <sstream>

#define BLUE_CHANNEL 0
#define GREEN_CHANNEL 1
#define RED_CHANNEL 2

class VegetationIndex
{
public:
    //Public Attributes
    int radianceFix;//Defines whether radiance and reflectance will be calculated.

    // These parameters of radiance, reflectance and solar elevation are usually obtained from the satellite documentation.
    // In the case of Parrot Sequoia, these values will not be necessary if the camera calibration process is carried out.
    // If this data is needed for any other reason, it will be saved in the TIFF image tags that Sequoia saves.
    double radianceMultRed;
    double radianceAddRed;
    double radianceMultNir;
    double radianceAddNir;
    double radianceMultRedEdge;
    double radianceAddRedEdge;
    double radianceMultGreen;
    double radianceAddGreen;
    double reflectanceMultRed;
    double reflectanceAddRed;
    double reflectanceMultNir;
    double reflectanceAddNir;
    double reflectanceMultRedEdge;
    double reflectanceAddRedEdge;
    double reflectanceMultGreen;
    double reflectanceAddGreen;
    double thetaSe;             //Sun elevation/Solar zenith angle

    // Attributes related to images
    cv::Mat shortWaveMat;//NIR
    cv::Mat redVisibleMat;
    cv::Mat greenVisibleMat;
    cv::Mat redEdgeMat;
    int width = 0;
    int height = 0;
    double paramA = 0.4; //(0 - 1) The value of parameter “a ” represents the proportion of red reflectance, and the value of (1 –a ) represents the proportion of red-edge reflectance (Vegetation Indices Combining the Red and Red-Edge Spectral Information for Leaf Area Index Retrieval)

    //Public Methods
    VegetationIndex();
    void setColor(unsigned char *image, int pos, int r, int g, int b);
    void setReflectance(unsigned char *data, int const &band);
    void setRadiance(unsigned char *data, int const &band);
    double getReflectance(int pixel, int band);
    double getRadiance(int grayPixel, int band);

    //Virtual Methods
    /* Apply vegetation index and save the image with the colors already changed. */
    virtual void setIndexSaveImg(unsigned char *shortWave, unsigned char *redVisible, unsigned char *redEdge, unsigned char *greenVisible, unsigned char *result) = 0;
    /* Replaces the pixel colors of the image with a representation linked to the index in question. */
    virtual void paint(double const &min, double const &max, double *image, unsigned char *out) = 0;
    /* Calculate the index to 1 pixel */
    virtual float setIndex(double nir, double redEdge, double red, double green) = 0;
    virtual double setIndexD(double nir, double redEdge, double red, double green) = 0;
};
#endif // VEGETATIONINDEX_H
