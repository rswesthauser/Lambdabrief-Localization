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

#ifndef MCL_H
#define MCL_H

class MCL;

#include <GL/glew.h>
#include "src/Robot/Robot.h"
#include "src/Map/MapGrid.h"
#include "src/Utils/GlutClass.h"
#include <GL/glut.h>
#include <opencv2/core/core.hpp>
#include "src/Heuristics/BriefHeuristic.h"
#include <omp.h>
#include <limits>

typedef struct{
    Pose3d p;
    double w; //WEIGHT
    double s; //SCALE
} MCLparticle;

class MCL
{
    public:
        MCL(vector<Heuristic*>& hVector, vector<MapGrid *> &cMaps, vector<cv::Mat> &gMaps, Pose3d &initial, string &lName, int &numParticles, vector<Pose3d>droneAngles, vector<Pose3d>CorrectedGT);
        ~MCL();
        bool run(Pose3d &u, bool is_u_reliable, cv::Mat &z, double time, Pose3d &real, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal);
        bool initialRun(Pose3d &u, bool is_u_reliable, cv::Mat &z, double time, Pose3d &real, double lastTotalElapsed, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal, Pose3d orientacao);
        void writeErrorLogFile3d(double trueX, double trueY, double trueTh);
        void writeLogFile3d();
        void draw(int x_aux, int y_aux, int halfWindowSize);
        void restart(Pose3d &initial, string &lName);
        void setNumParticles(int _numParticles);

        // Required to draw
        vector<MCLparticle> particles;
        vector<cv::Mat>& globalMaps;
        GLuint imageTex;

    private:
        fstream particleLog;
        int numParticles;
        int resamplingThreshold;
        double maxRange;
        double neff;
        double minscale;
        double maxscale;
        float sumW;
        Pose3d lastOdometry;
        STRATEGY locTechnique;
        Pose3d realPose;
        Pose3d odomPose;
        vector<Pose3d> realPath;
        vector<Pose3d> odomPath;
        vector<Pose3d>droneAngles;
        vector<Pose3d>CorrectedGT;
        vector<Heuristic*>& heuristics;
        vector<MapGrid*>& cachedMaps;
        vector<double> heuristicValues;
        vector<double> heuristicGradients;
        vector<cv::Mat> frameColorConverted;
        cv::Mat binaryFrameMask;

        bool starting;
        Timer timer;
        vector<vector<bool> > bins;
        void sampling(Pose3d &u, bool reliable);
        void weighting(cv::Mat& z_robot, Pose3d &u, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal);
        void prepareWeighting(cv::Mat &z);
        void prepareWeighting(cv::Mat &z, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal, Pose3d orientacao);
        void resampling();
        double computeNeff();
        double computeError(double trueX, double trueY,double particleX, double particleY);
        double computeError3d(double trueX, double trueY,double trueZ,double particleX, double particleY,double particleZ);
        double computeAngleError(double trueTh, double particleTh);
        double sumAngles(double a, double b);
        void createColorVersions(cv::Mat& imageRGB);
        void normalizingParticles();
        void setingEmptyBins();
        bool isEmpty(MCLparticle part);
        void updateOdomPath(Pose3d &u);
        bool verifyParticlePosition(MCLparticle part);
};

#endif // MCL_H
