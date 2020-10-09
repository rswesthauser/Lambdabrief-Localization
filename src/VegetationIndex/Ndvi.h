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

#ifndef NDVI_H
#define NDVI_H

#include "src/VegetationIndex/VegetationIndex.h"
/*
#define T_NDVI          1   //NDVI
#define T_RE_NDVI       2   //Red-edge NDVI
#define T_RED_RE_NDVI   3   //Red e Red-edge NDVI*/

class Ndvi : public VegetationIndex
{
public:
    //Public Attributes
    enum NdviType { ndvi=1, re_ndvi=2, red_re_ndvi=3 };
    int ndviType;

        //Public Methods
        Ndvi(int _radianceFix);
        Ndvi(int _radianceFix, int _width, int _height);
        Ndvi(int _radianceFix, enum NdviType _type);
        Ndvi(int _radianceFix, int _type);
        /*Computes the Red Edge NDVI for one pixel*/
        float setIndexPixelReNdvi(double shortWave, double redEdge);
        double setIndexPixelReNdviD(double shortWave, double redEdge);
        float setIndexPixelRedReNdvi(double shortWave, double redEdge, double redVisible);
        double setIndexPixelRedReNdviD(double shortWave, double redEdge, double redVisible);
        cv::Mat setIndexImage(cv::Mat &nirMap,cv::Mat &redMap, cv::Mat &regMap, cv::Mat &greMap);
        //Overridden Methods
        void setIndexSaveImg(unsigned char *shortWave, unsigned char *redVisible, unsigned char *redEdge, unsigned char *greenVisible, unsigned char *result);
        void paint(double const &min, double const &max, double *image, unsigned char *out);
        /*Computes the NDVI for one pixel*/
        float setIndexPixel(double shortWave, double redVisible);
        double setIndexPixelD(double shortWave, double redVisible);
        float setIndex(double nir, double redEdge, double red, double green);
        double setIndexD(double nir, double redEdge, double red, double green);
};

#endif // NDVI_H
