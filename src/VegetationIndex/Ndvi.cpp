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

#include "Ndvi.h"
#include "config.h"
#include "src/Utils/Utils.h"

Ndvi::Ndvi(int _radianceFix)
{
    this->radianceFix = _radianceFix;
    this->ndviType = ndvi;
}

Ndvi::Ndvi(int _radianceFix, int _width, int _height)
{
    this->radianceFix = _radianceFix;
    this->width = _width;
    this->height = _height;
    this->ndviType = ndvi;
}

Ndvi::Ndvi(int _radianceFix, enum NdviType _type)
{
    this->radianceFix = _radianceFix;
    this->ndviType = _type;
}

Ndvi::Ndvi(int _radianceFix, int _type)
{
    this->radianceFix = _radianceFix;
    this->ndviType = _type;
}

float Ndvi::setIndex(double nir, double redEdge, double red, double green)
{
    switch(this->ndviType)
    {
        case(ndvi):
            return setIndexPixel(nir, red);
        case(re_ndvi):
            return setIndexPixelReNdvi(nir, redEdge);
        case(red_re_ndvi):
            return setIndexPixelRedReNdvi(nir, redEdge, red);
    }
}

double Ndvi::setIndexD(double nir, double redEdge, double red, double green)
{
    switch(this->ndviType)
    {
        case(ndvi):
            return setIndexPixelD(nir, red);
        case(re_ndvi):
            return setIndexPixelReNdviD(nir, redEdge);
        case(red_re_ndvi):
            return setIndexPixelRedReNdviD(nir, redEdge, red);
    }
}

float Ndvi::setIndexPixel(double shortWave, double redVisible)
{
    return (shortWave - redVisible) / (shortWave + redVisible);
}

double Ndvi::setIndexPixelD(double shortWave, double redVisible)
{
    return (shortWave - redVisible) / (shortWave + redVisible);
}


float Ndvi::setIndexPixelReNdvi(double shortWave, double redEdge)
{
    return (shortWave - redEdge) / (shortWave + redEdge);
}

double Ndvi::setIndexPixelReNdviD(double shortWave, double redEdge)
{
    return (shortWave - redEdge) / (shortWave + redEdge);
}

float Ndvi::setIndexPixelRedReNdvi(double shortWave, double redEdge, double redVisible)
{
    return (shortWave - (paramA * redVisible + (1 - paramA) * redEdge))/
           (shortWave + (paramA * redVisible + (1 - paramA) * redEdge));
}

double Ndvi::setIndexPixelRedReNdviD(double shortWave, double redEdge, double redVisible)
{
    return (shortWave - (paramA * redVisible + (1 - paramA) * redEdge))/
           (shortWave + (paramA * redVisible + (1 - paramA) * redEdge));
}

void Ndvi::setIndexSaveImg(unsigned char *shortWave, unsigned char *redVisible, unsigned char *redEdge, unsigned char *greenVisible, unsigned char *result)
{
    int posGray;
    double minValue = -1.0, maxValue = 1.0;
    double value = 0.0;
    int size = sizeof(double) * this->width * this->height;
    double *temp;
    double nir = 0, red = 0, reg = 0, gre = 0;
    int i, posResult;
    temp = (double*)malloc(size);

    for (int row = 0; row < this->height; row++){
        for (int col = 0; col < width; col++)
        {
            posGray = (row * this->width) + col;

            if(radianceFix)
            {
                nir = getReflectance(shortWave[posGray], 4);
                red = getReflectance(redVisible[posGray], 3);
            }
            else
            {
                nir = shortWave[posGray];
                red = redVisible[posGray];
                reg = redEdge[posGray];
                gre = greenVisible[posGray];
            }

            if (nir + red + reg != 0)
            {
                value = setIndex(nir, reg, red, gre);
                temp[posGray] = (double)value;
            }
            else
            {
                temp[posGray] = (double)redVisible[posGray];
            }
        }
    }

    this->paint(-1, 1, temp, result);
    free(temp);
}


void Ndvi::paint(double const &min, double const &max, double *image, unsigned char *out)
{
    int posGray, posResult;
    double value = 0.0;
    for (int row = 0; row < height; row++){
        for (int col = 0; col < width; col++){
            posGray = (row*width)+col;
            posResult = ((row*width)+col)*3;

            value = image[posGray];

            setColor(out, posResult, Utils::Normalize(value, 0, 1, -1, 2)*255, 150,255 - 255*Utils::Normalize(value, 0, 1, -1, 2));
        }
    }
}
