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

#ifndef ROBOT_H
#define ROBOT_H

class Robot;

#include <vector>
using namespace std;

#include "src/Map/Grid.h"
#include "src/Utils/Utils.h"
#include "src/Mcl/Mcl.h"

enum MotionMode {MANUAL, WANDER, FOLLOWWALL, ENDING};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT};
enum ConnectionMode {SIMULATION, SERIAL, WIFI};

class Robot
{
public:
    Robot();
    ~Robot();

    virtual void initialize(ConnectionMode cmode, LogMode lmode, string fname, int numParticles);
    virtual void run();
    virtual void move(MovingDirection dir);
    virtual void draw(double xRobot, double yRobot, double angRobot);
    const Pose& getTruePose();
    const Pose& getPoseEstimate();
    const Pose3d& getOdometry();
    void drawPath();
    bool isReady();
    bool isRunning();
    Grid* grid;
    MotionMode motionMode_;
    MCL* mcLocalization;

protected:
    Pose3d odometry_;
    Pose poseEstimate_;
    Pose truePose_;
    bool ready_;
    bool running_;
    vector<Pose> path_;
    LogFile* logFile_;
    LogMode logMode_;
    bool runQuiet;
    streambuf* orig_buf;
    void disableCout();
    void enableCout();
};

#endif // ROBOT_H
