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

#include "Grvi.h"
#include "config.h"

Grvi::Grvi()
{}

Grvi::Grvi(enum GrviType _type)
{
    this->grviType = _type;
}

Grvi::Grvi(int _type)
{
    this->grviType = _type;
}

float Grvi::setIndex(double nir, double redEdge, double red, double green)
{
    switch(this->grviType)
    {
        case(grvi):
            return setIndexPixel(green, red);
            break;
    }
}

double Grvi::setIndexD(double nir, double redEdge, double red, double green)
{
    switch(this->grviType)
    {
        case(grvi):
            return setIndexPixelD(green, red);
            break;
    }
}

double Grvi::setIndexPixelD(double green, double red)
{
    double r = 0;
    r = (green - red) / (green + red);
    return r;
}

float Grvi::setIndexPixel(double green, double red)
{
    float r = 0;
    r = (green - red) / (green + red);
    return r;
}

void Grvi::setIndexSaveImg(unsigned char *shortWave, unsigned char *redVisible, unsigned char *redEdge, unsigned char *greenVisible, unsigned char *result)
{
    int posGray;
    double minValue = -1.0, maxValue = 1.0;
    double value = 0.0;
    int size = sizeof(double) * this->width * this->height;
    double *temp;
    double red = 0, gre = 0;
    int i, posResult;
    temp = (double*)malloc(size);

    for (int row = 0; row < this->height; row++){
        for (int col = 0; col < width; col++)
        {
            posGray = (row * this->width) + col;

            gre = greenVisible[posGray];
            red = redVisible[posGray];

            if (gre + red != 0)
            {
                value = this->setIndexPixel(gre, red);
                temp[posGray] = (double)value;
            }
            else
                temp[posGray] = (double)redVisible[posGray];
        }
    }

    this->paint(-1, 1, temp, result);
    free(temp);
}

void Grvi::paint(double const &min, double const &max, double *image, unsigned char *out)
{
    int posGray, posResult;
    double value = 0.0;
    for (int row = 0; row < height; row++){
        for (int col = 0; col < width; col++){
            posGray = (row*width)+col;
            posResult = ((row*width)+col)*3;

            value = image[posGray];

            if (value > 0.941)
                setColor(out, posResult, 0,102,0);

            else if (value > 0.824 && value <= 0.941)
                setColor(out, posResult, 0,136,0);

            else if (value > 0.706 && value <= 0.824)
                setColor(out, posResult, 0,187,0);

            else if (value > 0.588 && value <= 0.706)
                setColor(out, posResult, 0,255,0);

            else if (value > 0.471 && value <= 0.588)
                setColor(out, posResult, 204,255,0);

            else if (value > 0.353 && value <= 0.471)
                setColor(out, posResult, 255,255,0);

            else if (value > 0.235 && value <= 0.353)
                setColor(out, posResult, 255,204,0);

            else if (value > 0.118 && value <= 0.235)
                setColor(out, posResult, 255,136,0);

            else if (value > 0.000 && value <= 0.118)
                setColor(out, posResult, 255,0,0);

            else if (value <= 0.000 && value > -0.118)
                setColor(out, posResult, 238,0,0);

            else if (value <= -0.118 && value > -0.235)
                setColor(out, posResult, 221,0,0);

            else if (value <= -0.235 && value > -0.353)
                setColor(out, posResult, 204,0,0);

            else if (value <= -0.353 && value > -0.471)
                setColor(out, posResult, 187,0,0);

            else if (value <= -0.471 && value > -0.588)
                setColor(out, posResult, 170,0,0);

            else if (value <= -0.588 && value > -0.706)
                setColor(out, posResult, 153,0,0);

            else if (value <= -0.706 && value > -0.824)
                setColor(out, posResult, 136,0,0);

            else if (value <= -0.824 && value > -0.941)
                setColor(out, posResult, 119,0,0);

            else if (value <= -0.941)
                setColor(out, posResult, 102,0,0);
        }
    }
}
