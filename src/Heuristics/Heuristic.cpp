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

#include "Heuristic.h"

Heuristic::Heuristic(STRATEGY s, int id, double l, unsigned int cd):
    type(s), limiar(l), color_difference(cd), id(id)
{

}

// get Methods
STRATEGY Heuristic::getType()
{
    return type;
}

int Heuristic::getID()
{
    return id;
}

double Heuristic::getLimiar()
{
    return limiar;
}
int Heuristic::getColorDifference()
{
    return color_difference;
}

// set methods
void Heuristic::setLimiar(double val)
{
    limiar = val;
}
void Heuristic::setColorDifference(double val)
{
    color_difference = val;
}

vec3 Heuristic::getValuefromPixel(int x, int y, cv::Mat *image)
{
    if(image->type() != CV_32FC3 && image->type() != CV_32F)
    {
        cv::Vec3b color = image->at<cv::Vec3b>(y,x);
        double r = color[0];
        double g = color[1];
        double b = color[2];

        vec3 V(r, g, b);

        return V;
    }

    cv::Vec3f color = image->at<cv::Vec3f>(y,x);
    vec3 c(color[0], color[1], color[2]);
    return c;
}

// Apply mask
double Heuristic::calculateGradientOrientation(int xCenter, int yCenter, cv::Mat *image, cv::Mat *map)
{
    // The direction of the
    double l = calculateValue(xCenter-1, yCenter, image, map);
    double r = calculateValue(xCenter+1, yCenter, image, map);
    double u = calculateValue(xCenter, yCenter+1, image, map);
    double d = calculateValue(xCenter, yCenter-1, image, map);

    if(IS_UNDEF(l) || IS_UNDEF(r) || IS_UNDEF(u) || IS_UNDEF(d))
        return HEURISTIC_UNDEFINED;

    double dx = r-l;
    double dy = u-d;

    if(fabs(dy) < 0.00001 && fabs(dx) < 0.00001)
        return HEURISTIC_UNDEFINED;

    return  atan2(dy, dx); //radians
}

double Heuristic::calculateGradientSobelOrientation(int xCenter, int yCenter, cv::Mat *image, cv::Mat *map)
{
    // The direction of the
    double l  = calculateValue(xCenter-1, yCenter,   image, map);
    double r  = calculateValue(xCenter+1, yCenter,   image, map);
    double u  = calculateValue(xCenter,   yCenter+1, image, map);
    double d  = calculateValue(xCenter,   yCenter-1, image, map);
    double lu = calculateValue(xCenter-1, yCenter+1, image, map);
    double ru = calculateValue(xCenter+1, yCenter+1, image, map);
    double ld = calculateValue(xCenter-1, yCenter-1, image, map);
    double rd = calculateValue(xCenter+1, yCenter-1, image, map);

    if(IS_UNDEF(l) || IS_UNDEF(r) || IS_UNDEF(u) || IS_UNDEF(d) ||
       IS_UNDEF(lu) || IS_UNDEF(ru) || IS_UNDEF(ld) || IS_UNDEF(rd))
        return HEURISTIC_UNDEFINED;

    double dx = ru + 2*r + rd - lu - 2*l - ld;
    double dy = lu + 2*u + ru - ld - 2*d - rd;

    if(fabs(dy) < 0.00001 && fabs(dx) < 0.00001)
        return HEURISTIC_UNDEFINED;

    return atan2(dy, dx); //radians
}

KernelHeuristic::KernelHeuristic(STRATEGY s, int id, double l, unsigned int cd, int rad, double* k, int kW, int kH):
Heuristic(s, id, l, cd), radius(rad), kWidth(kW), kHeight(kH)
{
    this->kernel = new double[kWidth*kHeight];
    memcpy(this->kernel, k, sizeof(double)*kWidth*kHeight);
}

double KernelHeuristic::getRadius()
{
    return radius;
}
