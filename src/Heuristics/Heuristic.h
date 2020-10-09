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

#ifndef HEURISTIC_H
#define HEURISTIC_H

class Heuristic;

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include <climits>
#include <float.h>
#include "src/Utils/vec3.h"

#define HEURISTIC_UNDEFINED -DBL_MAX
#define HEURISTIC_UNDEFINED_INT INT_MIN
#define QUANTIZATION_LEVELS 255
#define IS_UNDEF(X) (X == HEURISTIC_UNDEFINED)

// Heuristics
enum STRATEGY{
    BRIEF,
    abBRIEF //and λ-BRIEF
};

class Heuristic
{
public:
    Heuristic(STRATEGY s, int id, double l, unsigned int cd);
    virtual double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map=NULL) = 0;
    double calculateGradientOrientation(int x, int y, cv::Mat *image, cv::Mat* map);
    double calculateGradientSobelOrientation(int x, int y, cv::Mat *image, cv::Mat* map);
    STRATEGY getType();
    double getLimiar();
    int getID();
    int getColorDifference();
    void setLimiar(double val);
    void setColorDifference(double val);
    vec3 getValuefromPixel(int x, int y, cv::Mat *image);
protected:
    STRATEGY type;
    int id;
    double limiar;
    unsigned int color_difference;
};

class KernelHeuristic : public Heuristic
{
public:
    KernelHeuristic(STRATEGY s, int id, double l, unsigned int cd, int rad, double* k, int kW, int kH);
    virtual double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map) = 0;
    double getRadius();

protected:
    int radius;
    double *kernel;
    int kWidth;
    int kHeight;
};
#endif
