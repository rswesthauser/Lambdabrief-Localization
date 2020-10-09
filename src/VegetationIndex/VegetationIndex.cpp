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

#include "VegetationIndex.h"

VegetationIndex::VegetationIndex()
{

}

void VegetationIndex::setColor(unsigned char *image, int pos, int r, int g, int b)
{
    image[pos + BLUE_CHANNEL] = b;
    image[pos + GREEN_CHANNEL] = g;
    image[pos + RED_CHANNEL] = r;
}

double VegetationIndex::getRadiance(int grayPixel, int band)
{
    if (grayPixel < 5)
        return 0;

    if (band == 3)
        return grayPixel * this->radianceMultRed + this->radianceAddRed;
    return grayPixel * this->radianceMultNir + this->radianceAddNir;
}

double VegetationIndex::getReflectance(int pixel, int band){
    if (pixel < 5)
        return 0;

    double theta = (this->thetaSe * M_PI) / 180;
    if (band == 3)
        return (this->reflectanceMultRed * pixel + this->reflectanceAddRed) / sin(theta);
    return (this->reflectanceMultNir * pixel + this->reflectanceAddNir) / sin(theta);
}

void VegetationIndex::setRadiance(unsigned char *data, int const &band)
{
    double value = 0.0;
    int pos;
    for (int row = 0; row < this->height; row++){
        for (int col = 0; col < this->width; col++){
            pos = row * this->width + col;
            value = getRadiance(data[pos], band);
            data[pos] = value;
        }
    }
}

void VegetationIndex::setReflectance(unsigned char *data, int const &band)
{
    double value = 0.0;
    int pos;
    for (int row = 0; row < this->height; row++){
        for (int col = 0; col < this->width; col++){
            pos = row * this->width + col;
            value = getReflectance((int)data[pos], band);
            data[pos] = (double)value;
        }
    }
}
