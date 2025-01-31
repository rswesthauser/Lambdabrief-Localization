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

#include <iostream>
#include <float.h>
#include <math.h>
#include "src/Utils/Utils.h"
#include "src/Map/MapGrid.h"

MapGrid::MapGrid(cv::Mat *image, cv::Mat* map, Heuristic *heuristic) : heuristic(heuristic)
{
    // initialize heuristic values
    width = image->cols;
    height = image->rows;
    heuristicValues = new double[width*height];

    // Initialize all cells with undefined values.
    for(int w=0; w<width; ++w)
         for(int h=0; h<height; ++h)
            heuristicValues[h*width + w] = HEURISTIC_UNDEFINED;
    
    // Calculate heuristic for all cells.
    for(int w=radius+1; w<width-radius-1; ++w)
        for(int h=radius+1; h<height-radius-1; ++h)
            heuristicValues[h*width + w] = heuristic->calculateValue(w, h, image, map);

    // Calculate bounds based on values.
    floorValue = DBL_MAX;
    ceilValue = -DBL_MAX;
    for(int i=0; i < width*height; ++i)
    {
        if(heuristicValues[i] > ceilValue)
            ceilValue = heuristicValues[i];
        if(heuristicValues[i] < floorValue)
            floorValue = heuristicValues[i];
    }
}


MapGrid::MapGrid(int width, int height, double floor, double ceil, Heuristic *heuristic)
    : width(width), height(height), floorValue(floor), ceilValue(ceil), heuristic(heuristic)
{
    cells = new occupancyValues[width*height];
    heuristicValues = new double[width*height];
    for(int cI=0; cI < width*height; cI++)
    {
        cells[cI] = CELL_UNKNOWN;
        heuristicValues[cI] = HEURISTIC_UNDEFINED;
    }
}

MapGrid::MapGrid(FILE *mapFile)
{
    // Read dimensions.
    char kernel[80];
    float fileThreshold=0.0;
    int fileColorDiff=0;
    int fileRadius=0;
    fscanf(mapFile, "%d %d %s %d %d %f", &width, &height, kernel, &fileRadius, &fileColorDiff, &fileThreshold);
    cout << "width: " << width << " height: " << height << endl;
    cout << "Densty data: " << kernel << " r " << fileRadius << " cdiff " <<  fileColorDiff <<
            " t " << fileThreshold << endl;

    cells = new occupancyValues[width*height];
    heuristicValues = new double[width*height];
    gradients = new double[width*height];
    reliableGradients = new bool[width*height];

    // Read grid and densities from file.
    char read;
    for(int hI=0; hI < height; hI++)
    {
        for(int wI=0; wI < width; wI++)
        {
            // density map is 0 or 1 for occupied + '/' + double number
            fscanf(mapFile,"%c/", &read);

           //storing density
            fscanf(mapFile,"%lf ", &(heuristicValues[hI*width + wI]));

            //storing occupied cells
            switch(read)
            {
                case '1':
                    cells[hI*width + wI] = CELL_OBSTACLE;
                    heuristicValues[hI*width + wI] = DBL_MAX;
                    break;
                case '0':
                    cells[hI*width + wI] = CELL_FREE;
                    break;
                case '-':
                    cells[hI*width + wI] = CELL_UNKNOWN;
                    heuristicValues[hI*width + wI] = -DBL_MAX;
                    break;
            }
        }
        fscanf(mapFile,"\n");
    }
    fclose(mapFile);

    // Calculate bounds based on values.
    floorValue = DBL_MAX;
    ceilValue = -DBL_MAX;
    for(int i=0; i < width*height; i++) {
        if(cells[i] == CELL_FREE){
            if(heuristicValues[i] > ceilValue)
                ceilValue = heuristicValues[i];
            if(heuristicValues[i] < floorValue)
                floorValue = heuristicValues[i];
        }
    }

    cout << "ceil  "<< ceilValue<< " floor " << floorValue << endl;

    // Calculate gradients
    for(int hI=1; hI < height-1; hI++){
        for(int wI=1; wI < width-1; wI++){
            gradients[hI*width + wI] = calculateSobelOrientation(wI, hI);
        }
    }

    // Find reliable gradients
    for(int hI=2; hI < height-2; hI++){
        for(int wI=2; wI < width-2; wI++){
            reliableGradients[hI*width + wI] = checkGrad(wI, hI);
        }
    }
}

double MapGrid::getFloorValue()
{
    return floorValue;
}

double MapGrid::getCeilValue()
{
    return ceilValue;
}

int MapGrid::getWidth()
{
    return width;
}

int MapGrid::getHeight()
{
    return height;   
}
        
void MapGrid::setObstacle(bool isObstacle, int x, int y)
{
    if(isObstacle){
        cells[y*width + x] = CELL_OBSTACLE;
    }else{
        cells[y*width + x] = CELL_FREE;
    }
}

void MapGrid::clearCell(int x, int y)
{
    cells[y*width + x] = CELL_UNKNOWN;
    heuristicValues[y*width + x] = HEURISTIC_UNDEFINED;
}

bool MapGrid::isObstacle(int x, int y)
{
    if(x<0 || x>=width || y<0 || y>=height)
        return true;
    return (cells[y*width + x]==CELL_OBSTACLE);
}

bool MapGrid::isKnown(int x, int y)
{
    if(x<0 || x>=width || y<0 || y>=height)
        return false;
    return (cells[y*width + x]!=CELL_UNKNOWN);
}
    
int MapGrid::getHeuristicValue(int x, int y)
{
    if(x<0 || x>=width || y<0 || y>=height)
        return HEURISTIC_UNDEFINED;
    if(fabs(heuristicValues[y*width + x]-HEURISTIC_UNDEFINED) < 0.01) // Undefined
        return HEURISTIC_UNDEFINED_INT;
    else
        return int( (heuristicValues[y*width + x] - floorValue) /
                    (ceilValue - floorValue) * QUANTIZATION_LEVELS
                );
}
int MapGrid::convertToMapGridDiscrete(double val)
{
    return int((val - floorValue)/(ceilValue - floorValue) * QUANTIZATION_LEVELS);
}

double MapGrid::getPureHeuristicValue(int x, int y)
{
    return heuristicValues[y*width + x];
}

double MapGrid::getOrientation(int x, int y)
{
    return gradients[y*width + x];
}

bool MapGrid::isGradientReliable(int x, int y)
{
    return reliableGradients[y*width + x];
}

void MapGrid::calculateHeuristicFor(int x, int y, cv::Mat* image, cv::Mat* map)
{
    heuristicValues[y*width + x] = heuristic->calculateValue(x,y,image,map);
}

double MapGrid::calculateGradientFor(int x, int y, cv::Mat* image, cv::Mat* map)
{
    return heuristic->calculateGradientOrientation(x,y,image,map);
}

double MapGrid::calculateOrientation(int x, int y)
{
    if(IS_UNDEF(heuristicValues[(y+1)*width + x]) ||
       IS_UNDEF(heuristicValues[(y-1)*width + x]) ||
       IS_UNDEF(heuristicValues[y*width + x+1])   ||
       IS_UNDEF(heuristicValues[y*width + x-1]))
        return HEURISTIC_UNDEFINED;

    double dy = getPureHeuristicValue(x,y+1) - getPureHeuristicValue(x,y-1);
    double dx = getPureHeuristicValue(x+1,y) - getPureHeuristicValue(x-1,y);
    if(fabs(dy) < 0.00001 && fabs(dx) < 0.00001)
        return HEURISTIC_UNDEFINED;
    return atan2(dy, dx);
}

double MapGrid::calculateSobelOrientation(int x, int y)
{
    if(IS_UNDEF(heuristicValues[(y+1)*width + x+1]) ||
       IS_UNDEF(heuristicValues[(y+1)*width + x-1]) ||
       IS_UNDEF(heuristicValues[(y+1)*width + x  ]) ||
       IS_UNDEF(heuristicValues[(y-1)*width + x+1]) ||
       IS_UNDEF(heuristicValues[(y-1)*width + x-1]) ||
       IS_UNDEF(heuristicValues[(y-1)*width + x  ]) ||
       IS_UNDEF(heuristicValues[(y  )*width + x+1]) ||
       IS_UNDEF(heuristicValues[(y  )*width + x-1]))
        return HEURISTIC_UNDEFINED;

    double dx =       getPureHeuristicValue(x+1,y-1)
                + 2.0*getPureHeuristicValue(x+1,y)
                +     getPureHeuristicValue(x+1,y+1)
                -     getPureHeuristicValue(x-1,y-1)
                - 2.0*getPureHeuristicValue(x-1,y)
                -     getPureHeuristicValue(x-1,y+1);

    double dy =       getPureHeuristicValue(x-1,y+1)
                + 2.0*getPureHeuristicValue(x  ,y+1)
                +     getPureHeuristicValue(x+1,y+1)
                -     getPureHeuristicValue(x-1,y-1)
                - 2.0*getPureHeuristicValue(x  ,y-1)
                -     getPureHeuristicValue(x+1,y+1);

    if(fabs(dy) < 0.00001 && fabs(dx) < 0.00001)
        return HEURISTIC_UNDEFINED;

    return atan2(dy, dx);
}

bool MapGrid::checkGrad(int x, int y)
{
    int offset[8][2] = { {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}};

    double gC = gradients[y*width + x];

    for(int n=0;n<8;++n)
    {
        int nx = x + offset[n][0];
        int ny = y + offset[n][1];
        double gN = gradients[ny*width + nx];

        if(IS_UNDEF(gN))
            return false;

        if(acos(cos(gC)*cos(gN) + sin(gC)*sin(gN)) > DEG2RAD(45.0))
            return false;
    }

    return true;
}

void MapGrid::draw()
{
    cv::Mat a(this->getHeight(), this->getWidth(),CV_8UC1,cv::Scalar(0));
    for(int hI=0; hI < this->getWidth(); ++hI)
        for(int wI=0; wI < this->getHeight(); wI++)
            a.at<char>(wI, hI) = this->getHeuristicValue(hI,wI);

    resize(a,a,cv::Size(0,0),0.2,0.2);
    cv::namedWindow("Map Grid Values",CV_WINDOW_KEEPRATIO);
    cv::waitKey(0);
}


void MapGrid::drawLine()
{
    cv::Mat a(this->getHeight(), this->getWidth(), CV_8UC1,cv::Scalar(0));
    for(int hI=0; hI < this->getWidth(); ++hI)
        for(int wI=0; wI < this->getHeight(); wI++)
            a.at<char>(wI, hI) = (this->getHeuristicValue(hI,wI)>100?this->getHeuristicValue(hI,wI):0);

    resize(a,a,cv::Size(0,0),0.2,0.2);
    cv::namedWindow("Map Grid Values",CV_WINDOW_KEEPRATIO);
    cv::waitKey(0);
}
