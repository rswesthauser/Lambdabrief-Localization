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

#ifndef __GRID_H__
#define __GRID_H__

class Cell
{
    public:
        int x,y;        
        bool visited, isObstacle;
        int himm, himm_count, laser_count;
        double distWalls, dirX, dirY;
};

class Grid
{
    public:
        Grid();
        Cell* getCell(int x, int y);

        int getMapScale();
        int getMapWidth();
        int getMapHeight();

        void draw(int xi, int yi, int xf, int yf);

        int numViewModes;
        int viewMode;
        bool showValues;
        bool showArrows;

        int himm_count;

    private:
        int mapScale_; // Number of cells per meter
        int mapWidth_, mapHeight_; // in cells
        int numCellsInRow_, halfNumCellsInRow_;
        Cell* cells_;
        void drawCell(unsigned int i);
        void drawVector(unsigned int i);
        void drawText(unsigned int n);
};

#endif // __GRID_H__
