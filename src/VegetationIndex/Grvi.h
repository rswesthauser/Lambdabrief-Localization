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

#ifndef GRVI_H
#define GRVI_H

#include "src/VegetationIndex/VegetationIndex.h"

class Grvi : public VegetationIndex
{
public:    
    //Public Attributes
    enum GrviType { grvi=1 };
    int grviType;
    //Public Methods
        Grvi();
        Grvi(enum GrviType _type);
        Grvi(int _type);
    //Overridden Methods
        void setIndexSaveImg(unsigned char *shortWave, unsigned char *redVisible, unsigned char *redEdge, unsigned char *greenVisible, unsigned char *result);
        void paint(double const &min, double const &max, double *image, unsigned char *out);
        float setIndexPixel(double green, double red);
        double setIndexPixelD(double green, double red);
        float setIndex(double nir, double redEdge, double red, double green);
        double setIndexD(double nir, double redEdge, double red, double green);
        cv::Mat setIndexImage(cv::Mat &nirMap,cv::Mat &redMap, cv::Mat &regMap, cv::Mat &greMap);
};

#endif // GRVI_H
