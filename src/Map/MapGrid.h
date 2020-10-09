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

#ifndef MAP_GRID_H
#define MAP_GRID_H

class MapGrid;

#include <stdio.h>
#include "src/Heuristics/Heuristic.h"

using namespace std;
class Heuristic;
enum occupancyValues { CELL_FREE, CELL_OBSTACLE, CELL_UNKNOWN };

class MapGrid {
    public:
        // Start grid from file info, calculating densities using the heuristic.
        MapGrid(cv::Mat *image, cv::Mat *map, Heuristic *heuristic);
        // Start empty grid. 
        MapGrid(int width, int height, double floor, double ceil, Heuristic *heuristic);
        // Start grid from file with density info.
        MapGrid(FILE *mapFile);
       
        int getWidth();
        int getHeight();
        void draw();
        void drawLine();        
        double getFloorValue();
        double getCeilValue();
        int convertToMapGridDiscrete(double val);
        void setObstacle(bool isObstacle, int x, int y);
        bool isObstacle(int x, int y);
        bool isKnown(int x, int y);
        void clearCell(int x, int y);    
        int    getHeuristicValue(int x, int y);
        double getPureHeuristicValue(int x, int y);
        double getOrientation(int x, int y);
        bool   isGradientReliable(int x, int y);
        void   calculateHeuristicFor(int x, int y, cv::Mat* image, cv::Mat* map);
        double calculateGradientFor(int x, int y, cv::Mat *image, cv::Mat* map);
        double calculateOrientation(int x, int y); //get orientation from precalculated map (true)
        double calculateSobelOrientation(int x, int y); //get orientation from precalculated map (true)
        bool checkGrad(int x, int y);
        Heuristic *heuristic;
    private:
        occupancyValues *cells;
        double *heuristicValues;
        double *gradients;
        bool *reliableGradients;
        int width;
        int height;
        double floorValue;
        double ceilValue;
        double radius;
};

#endif
