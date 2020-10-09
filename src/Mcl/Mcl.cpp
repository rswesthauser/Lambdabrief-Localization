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

#include <random>
#include <chrono>
#include <limits>
#include "Mcl.h"
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <set>
#include <GL/glut.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "src/Robot/DroneRobot.h"
#include "src/Utils/RadiusVolumeTransferFunctions.h"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include "config.h"

MCL::MCL(vector<Heuristic*> &hVector, vector<MapGrid *> &cMaps, vector<cv::Mat> &gMaps, Pose3d &initial, string &lName, int &nParticles, vector<Pose3d>droneAngles, vector<Pose3d>CorrectedGT):
    heuristics(hVector),
    heuristicValues(heuristics.size(),0.0),
    heuristicGradients(heuristics.size(),0.0),
    frameColorConverted(3),
    binaryFrameMask(),
    cachedMaps(cMaps),
    globalMaps(gMaps)
{
    this->sumW = 0;
    this->droneAngles = droneAngles;
    this->CorrectedGT = CorrectedGT;
    numParticles = nParticles;
    cout << "PARTICLES:" << numParticles<< endl;
    resamplingThreshold = numParticles/8;
    lastOdometry.x=0.0;
    lastOdometry.y=0.0;
    lastOdometry.yaw=0.0;
    starting=false;
    particles.resize(numParticles);
    realPose = initial;
    odomPose = initial;
    minscale = minScale;
    maxscale = maxScale;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> randomX(0.0,globalMaps[0].cols);
    std::uniform_real_distribution<double> randomY(0.0,globalMaps[0].rows);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);
    std::uniform_real_distribution<double> randomS(minscale,maxscale);//SCALE FACTOR
    std::uniform_real_distribution<double> randomRollPitch(-45*(M_PI/180), 45*(M_PI/180));

    // generate initial set
    for(int i=0; i<particles.size(); i++)
    {
        bool valid = false;
        do{            
            // sample particle pose
            particles[i].p.x = randomX(generator);
            particles[i].p.y = randomY(generator);
            particles[i].p.roll = randomRollPitch(generator);
            particles[i].p.pitch = randomRollPitch(generator);
            particles[i].p.yaw = randomTh(generator);
            particles[i].p.z = randomS(generator); //SCALE FACTOR
                valid = true;

        }while(!valid);
    }

    /******************** Prepare log file *****************************/
    if(lName.empty()){
        time_t t = time(0);
        struct tm *now = localtime(&t);
        stringstream logName;
        logName << "../LambdaBrief-Localization/Logs/mcl-" << -100+now->tm_year
                        << setfill('0') << setw(2) << 1+now->tm_mon
                        << setfill('0') << setw(2) << now->tm_mday << '-'
                        << setfill('0') << setw(2) << now->tm_hour
                        << setfill('0') << setw(2) << now->tm_min
                        << setfill('0') << setw(2) << now->tm_sec << ".txt";
        cout << logName.str() << endl; cout.flush();
        particleLog.open(logName.str().c_str(), std::fstream::out);
    }else{
        particleLog.open(lName.c_str(), std::fstream::out);
    }
}

void MCL::restart(Pose3d &initial, string &lName)
{
    lastOdometry.x=0.0;
    lastOdometry.y=0.0;
    lastOdometry.yaw=0.0;
    realPose = initial;
    odomPose = initial;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> randomX(0.0,globalMaps[0].cols);
    std::uniform_real_distribution<double> randomY(0.0,globalMaps[0].rows);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);
    std::uniform_real_distribution<double> randomS(minscale, maxscale); //SCALE FACTOR
    std::uniform_real_distribution<double> randomRollPitch(-rollPitchAngRange*(M_PI/180), rollPitchAngRange*(M_PI/180));

    // generate initial set
    for(int i=0; i<particles.size(); i++)
    {
        bool valid = false;
        do
        {
            // sample particle pose
            particles[i].p.x = randomX(generator);
            particles[i].p.y = randomY(generator);
            particles[i].p.roll = randomRollPitch(generator);
            particles[i].p.pitch = randomRollPitch(generator);
            particles[i].p.yaw = randomTh(generator);
            particles[i].p.z = randomS(generator);
            valid = true;

        }while(!valid);
    }

    realPath.clear();
    odomPath.clear();

    if(particleLog.is_open())
        particleLog.close();

    /******************** Prepare log file *****************************/
    if(lName.empty())
    {
        time_t t = time(0);
        struct tm *now = localtime(&t);
        stringstream logName;
        logName << "../LambdaBrief-Localization/Logs/mcl-" << -100+now->tm_year
                        << setfill('0') << setw(2) << 1+now->tm_mon
                        << setfill('0') << setw(2) << now->tm_mday << '-'
                        << setfill('0') << setw(2) << now->tm_hour
                        << setfill('0') << setw(2) << now->tm_min
                        << setfill('0') << setw(2) << now->tm_sec << ".txt";
        cout << logName.str() << endl; cout.flush();
        particleLog.open(logName.str().c_str(), std::fstream::out);
    }else{
        particleLog.open(lName.c_str(), std::fstream::out);
    }
      particleLog << "meanPX meanPY meanPZ varPX varPY varPZ meanAngle angleStdev NEFF elapsedTime\n";
}

MCL::~MCL()
{
    particleLog.close();
}

void MCL::draw(int x_aux, int y_aux, int halfWindowSize)
{
    int h = globalMaps[0].rows;
    int w = globalMaps[0].cols;



    // Atualiza a região da janela
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    if(w > h){
        glOrtho (0 + x_aux + halfWindowSize,
                 w + x_aux - halfWindowSize,
                 h+(w-h)/2 + y_aux - halfWindowSize,
                 -(w-h)/2 + y_aux + halfWindowSize,
                 -1, 50);
    }else{
        glOrtho (-(h-w)/2 + x_aux + halfWindowSize,
                 w+(h-w)/2 + x_aux - halfWindowSize,
                 h + y_aux - halfWindowSize,
                 0 + y_aux + halfWindowSize,
                 -1, 50);
    }
    glMatrixMode (GL_MODELVIEW);

    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear (GL_COLOR_BUFFER_BIT);

    glColor3f(1.0f, 1.0f, 1.0f);

    glEnable(GL_TEXTURE_2D);
    // Draw map
    // Note: Window co-ordinates origin is top left, texture co-ordinate origin is bottom left.
    glBindTexture(GL_TEXTURE_2D, imageTex);
    glBegin(GL_QUADS);
        glTexCoord2f(1, 1);
        glVertex2f(0,  h);
        glTexCoord2f(0, 1);
        glVertex2f( w,  h);
        glTexCoord2f(0, 0);
        glVertex2f(w, 0);
        glTexCoord2f(1, 0);
        glVertex2f(0, 0);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // Draw real path
    if(realPath.size() > 1)
    {
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            glColor3f(0.0,1.0,0.0);
            for(unsigned int i=0;i<realPath.size()-1; i++){
                glVertex2f(realPath[i].x, realPath[i].y);
                glVertex2f(realPath[i+1].x, realPath[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
    }

    // Draw odom path
    if(odomPath.size() > 1)
    {
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            glColor3f(1.0,1.0,0.0);
            for(unsigned int i=0;i<odomPath.size()-1; i++)
            {
                glVertex2f(odomPath[i].x, odomPath[i].y);
                glVertex2f(odomPath[i+1].x, odomPath[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
    }

    // Draw real pose
    double x=realPose.x;
    double y=realPose.y;
    double th=realPose.yaw;
    glColor3f(0.0,1.0,0.0);
    glPointSize(12);
    glBegin( GL_POINTS ); // point
    {
        glVertex2f(x, y);
    }
    glEnd();
    glColor3f(0.0, 0.0, 0.0);
    glLineWidth(2);
    glBegin( GL_LINES ); // direction
    {
        glVertex2f(x, y);
        glVertex2f(x+cos(th)*150, y+sin(th)*150);
    }
    glEnd();
    glLineWidth(1);

    // Draw odom pose
    x=odomPose.x;
    y=odomPose.y;
    th=odomPose.yaw;
    glColor3f(1.0,1.0,0.0);
    glPointSize(12);
    glBegin( GL_POINTS ); // point
    {
        glVertex2f(x, y);
    }
    glEnd();
    glColor3f(0.0, 0.0, 0.0);
    glLineWidth(2);
    glBegin( GL_LINES ); // direction
    {
        glVertex2f(x, y);
        glVertex2f(x+cos(th)*150, y+sin(th)*150);
    }
    glEnd();
    glLineWidth(1);

    if(starting)
    {
        glutSwapBuffers();
        glutPostRedisplay();
        return;
    }

    // Draw particles
    for(int p=0;p<particles.size();p++)
    {
        double x=particles[p].p.x;
        double y=particles[p].p.y;
        double th=particles[p].p.yaw;

        glColor3f(1.0,0.0,0.0);

        glPointSize(6);
        glBegin( GL_POINTS );
        {
            glVertex2f(x, y);
        }
        glEnd();

        // Draw direction
        glColor3f(0.0, 0.0, 1.0);
        glLineWidth(2);
        glBegin( GL_LINES );
        {
            glVertex2f(x, y);
            glVertex2f(x+cos(th)*50, y+sin(th)*50);
        }
        glEnd();
        glLineWidth(1);

    }

    glutSwapBuffers();
    glutPostRedisplay();
}

bool MCL::initialRun(Pose3d &u, bool is_u_reliable, cv::Mat &z, double time, Pose3d &real, double lastTotalElapsed, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal, Pose3d orientacao)
{
    // Create different versions of the input image
    createColorVersions(z);

    realPose = real;
    realPath.push_back(realPose);

    double wElapsedTime, totalElapsedTime=0.0;
    struct timeval tstart, tend, tstartW, tendW;

    // Start counting the elapsed time of this iteration
    gettimeofday(&tstart, NULL);
    sampling(u,is_u_reliable);
    gettimeofday(&tstartW, NULL);

    if(multispectralCam)
        prepareWeighting(z, redMap, nirMap, regMap, greMap, currentMapOriginal, orientacao);
    else
        prepareWeighting(z);

    weighting(z, u, redMap, nirMap, regMap, greMap, currentMapOriginal);
    gettimeofday(&tendW, NULL);

    double sumWeights = 0.0;
    for(int i=0; i<particles.size(); i++)
        sumWeights += particles[i].w;

    if(sumWeights!=0)
        resampling();

    lastOdometry = u;

    // Stop counting the elapsed time of this SLAM iteration
    gettimeofday(&tend, NULL);

    // Compute and print the elapsed time
    if (tstart.tv_usec > tend.tv_usec)
    {
        tend.tv_usec += 1000000;
        tend.tv_sec--;
    }
    if (tstartW.tv_usec > tendW.tv_usec)
    {
        tendW.tv_usec += 1000000;
        tendW.tv_sec--;
    }

    wElapsedTime = ((double)tendW.tv_sec - (double)tstartW.tv_sec) + ((double)tendW.tv_usec - (double)tstartW.tv_usec)/1000000.0;
    totalElapsedTime = ((double)tend.tv_sec - (double)tstart.tv_sec) + ((double)tend.tv_usec - (double)tstart.tv_usec)/1000000.0;
    cout << "PARTICLES:" << numParticles
         << " total elapsed time MCL: " << totalElapsedTime << " weighting elapsed time MCL: " << wElapsedTime
         << " ratio:" << wElapsedTime/totalElapsedTime << endl;

    float time_limit = 0.33333333;
    if(wElapsedTime > time_limit )
    {
        numParticles = (time_limit-lastTotalElapsed)/(totalElapsedTime-lastTotalElapsed)*(numParticles/2) + numParticles/2;

        particles.resize(numParticles);
        cout << "FINAL NUMPARTICLES " << numParticles << endl;

        particleLog << "# NumParticles " << numParticles << endl;
        particleLog << "# trueX trueY meanPX3333 meanPY closestx cloesty closestTh closestw meanParticleErrorOk meanParticleStdev meanErrorOk stdevError trueTh meanAngle angleStdev angleError stdevAngleError NEFF elapsedTime\n";

        std::default_random_engine generator;
        std::uniform_real_distribution<double> randomX(0.0,globalMaps[0].cols);
        std::uniform_real_distribution<double> randomY(0.0,globalMaps[0].rows);
        std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);
        std::uniform_real_distribution<double> randomS(minscale, maxscale); //SCALE FACTOR
        std::uniform_real_distribution<double> randomRollPitch(-rollPitchAngRange*(M_PI/180), rollPitchAngRange*(M_PI/180));

        // generate initial set
        for(int i=0; i<particles.size(); i++)
        {
            bool valid = false;
            do{
                // sample particle pose
                particles[i].p.x = randomX(generator);
                particles[i].p.y = randomY(generator);
                particles[i].p.roll = randomRollPitch(generator);
                particles[i].p.pitch = randomRollPitch(generator);
                particles[i].p.yaw = randomTh(generator);
                particles[i].p.z = randomS(generator);
                valid = true;

            }while(!valid);
        }

        starting=false;
    }
    else
    {
        numParticles*=2;
        particles.resize(numParticles);

        std::default_random_engine generator;
        std::uniform_real_distribution<double> randomX(0.0,globalMaps[0].cols);
        std::uniform_real_distribution<double> randomY(0.0,globalMaps[0].rows);
        std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);
        std::uniform_real_distribution<double> randomS(minscale, maxscale);  //SCALE FACTOR
        std::uniform_real_distribution<double> randomRollPitch(-rollPitchAngRange*(M_PI/180), rollPitchAngRange*(M_PI/180));

        // generate initial set
        for(int i=0; i<particles.size(); i++)
        {

            bool valid = false;
            do{
                // sample particle pose
                particles[i].p.x = randomX(generator);
                particles[i].p.y = randomY(generator);
                particles[i].p.roll = randomRollPitch(generator);
                particles[i].p.pitch = randomRollPitch(generator);
                particles[i].p.yaw = randomTh(generator);
                particles[i].p.z = randomS(generator); //SCALE FACTOR
                valid = true;

            }while(!valid);
        }
        initialRun(u,is_u_reliable,z,time,real,totalElapsedTime, redMap, nirMap, regMap, greMap, currentMapOriginal, droneAngles[currentImage]);
    }
    return true;
}

bool MCL::run(Pose3d &u, bool is_u_reliable, cv::Mat &z, double time, Pose3d& real, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal)//"z" é o mapa local atual, a imagem que a câmera do drone acaba de capturar.
{
    createColorVersions(z);
    realPose = real;
    realPath.push_back(realPose);
    sampling(u,is_u_reliable);

    if(multispectralCam)
        prepareWeighting(z, redMap, nirMap, regMap, greMap, currentMapOriginal, droneAngles[currentImage]);
    else
        prepareWeighting(z);

    weighting(z, u, redMap, nirMap, regMap, greMap, currentMapOriginal);

    double sumWeights = 0.0;
    for(int i=0; i<particles.size(); i++)
        sumWeights += particles[i].w;

    if(sumWeights!=0)
        resampling();

    lastOdometry = u;
    return true;
}

void MCL::writeLogFile3d()
{
    double elapsedTime=timer.getLapTime();

    /**************************************************************
     ******************  Position information  ********************
     **************************************************************/
    // For each particle compute the weighed mean position error
    double normalizer = 0.0;
    double meanPX = 0.0;
    double meanPY = 0.0;
    double meanPZ = 0.0;

    double varPX = 0.0;
    double varPY = 0.0;
    double varPZ = 0.0;

    double NEFF = 0.0;
    // Evaluate weighed mean particle
    for(int i=0; i<particles.size(); i++) {
        normalizer+=particles[i].w;
        NEFF+=particles[i].w*particles[i].w;
    }
    cout << "Normalizer data:" << normalizer << endl;
    NEFF = 1/NEFF;

    if(normalizer==0.0)
        for(int i=0; i<particles.size(); i++)
            particles[i].w=1.0/particles.size();

    // Evaluate weighed mean particle
    for(int i=0; i<particles.size(); i++)
    {
        meanPX += particles[i].p.x*particles[i].w;
        meanPY += particles[i].p.y*particles[i].w;
        meanPZ += particles[i].p.z*particles[i].w;
    }

    meanPX  /= normalizer;
    meanPY  /= normalizer;
    meanPZ  /= normalizer;

    // Evaluate weighed mean particle variance
    double meanParticleVar = 0.0;
    for(int i=0; i<particles.size(); i++)
    {
      //Variancia de X, Y e Z
      varPX = pow((particles[i].p.x-meanPX), 2.0)*particles[i].w;
      varPY = pow((particles[i].p.y-meanPY), 2.0)*particles[i].w;
      varPZ = pow((particles[i].p.z-meanPZ), 2.0)*particles[i].w;
    }

    /**************************************************************
     ********************  Angle information  *********************
     **************************************************************/
    // For each particle compute the weighed mean angle
    double meanAngle  = 0.0;
    double angle,sin_ang,cos_ang;
    sin_ang = 0;
    cos_ang = 0;
    for(int i=0; i<particles.size(); i++)
    {
        angle = particles[i].p.yaw;
        cos_ang += cos(angle)*particles[i].w;
        sin_ang += sin(angle)*particles[i].w;
    }
    cos_ang /= normalizer;
    sin_ang /= normalizer;
    meanAngle = atan2(sin_ang, cos_ang)*180/M_PI;

    // For each particle compute the mean particle angle var and stdev
    double stddev = sqrt(-1.0 * log(pow(sin_ang,2)+pow(cos_ang,2)));
    double angleVar = pow(stddev,2);

    particleLog  << meanPX << " " << meanPY << " " << meanPZ << " " << varPX<< " " << varPY << " " << varPZ << " "
                 << meanAngle << " " << angleVar << " "
                 << NEFF << " " << elapsedTime << endl;
    particleLog.flush();
    timer.startLap();

    try
    {
       if(meanPZ > maxScale)
       {
           throw 1;
       }
       else if(meanPZ < minScale)
       {
           throw 2;
       }
    }
    catch(int erro)
    {
       if (erro == 1)
           cout << "The average of the cloud scale is greater than the maximum scale" << endl;
       else if (erro == 2)
           cout << "The cloud scale average is less than the minimum scale" << endl;
    }
}

void MCL::writeErrorLogFile3d(double trueX, double trueY, double trueTh)
{
    double elapsedTime=timer.getLapTime();

    /**************************************************************
     ******************  Position information  ********************
     **************************************************************/
    // For each particle compute the weighed mean position error
    double normalizer = 0.0;
    double meanParticleError = 0.0;
    double meanPX = 0.0;
    double meanPY = 0.0;

    double NEFF = 0.0;
    // Evaluate weighed mean particle
    for(int i=0; i<particles.size(); i++)
    {
        normalizer+=particles[i].w;
        NEFF+=particles[i].w*particles[i].w;
    }
    cout << "Normalizer data:" << normalizer << endl;
    NEFF = 1/NEFF;

    if(normalizer==0.0)
        for(int i=0; i<particles.size(); i++)
            particles[i].w=1.0/particles.size();

    // Evaluate weighed mean particle
    for(int i=0; i<particles.size(); i++)
    {
        meanPX += particles[i].p.x*particles[i].w;
        meanPY += particles[i].p.y*particles[i].w;
    }

    meanPX  /= normalizer;
    meanPY  /= normalizer;
    meanParticleError = computeError(trueX, trueY, meanPX, meanPY);

    // Evaluate weighed mean particle variance
    double meanParticleVar = 0.0;
    for(int i=0; i<particles.size(); i++)
    {
        meanParticleVar +=
        pow(computeError(meanPX, meanPY, particles[i].p.x, particles[i].p.y)-meanParticleError, 2.0)*particles[i].w;
    }
    meanParticleVar/=normalizer;
    double meanParticleStdev = sqrt(meanParticleVar);

    // Evaluate mean error
    double meanError = 0.0;
    MCLparticle closest=particles[0];
    double bestError = DBL_MAX;
    for(int i=0; i<particles.size(); i++)
    {
        double x = particles[i].p.x;
        double y = particles[i].p.y;

        // get closest particle
        double error = computeError(trueX, trueY, x, y);
        if(error<bestError)
        {
            closest = particles[i];
            bestError = error;
        }
        // weighing the error
        error *= particles[i].w;
        meanError += error;
    }
    meanError/=normalizer;

    // Evaluate mean error variance
    double varError = 0.0;
    for(int i=0; i<particles.size(); i++)
    {
        double x = particles[i].p.x;
        double y = particles[i].p.y;
        varError += pow(computeError(trueX, trueY, x, y)-meanError, 2.0)*particles[i].w;
    }
    varError = varError/normalizer;
    double stdevError = sqrt(varError);

    /**************************************************************
     ********************  Angle information  *********************
     **************************************************************/
    // For each particle compute the weighed mean angle

    double meanAngle  = 0.0;
    double ponderada,sin_ang,cos_ang;
    sin_ang = 0;
    cos_ang = 0;
    for(int i=0; i<particles.size(); i++)
    {
        ponderada = particles[i].p.yaw;
        cos_ang += cos(ponderada)*particles[i].w;
        sin_ang += sin(ponderada)*particles[i].w;
    }
    cos_ang /= normalizer;
    sin_ang /= normalizer;
    meanAngle = atan2(sin_ang, cos_ang)*180/M_PI;

    // For each particle compute the mean particle angle var and stdev    
    double stddev = sqrt(-1.0 * log(pow(sin_ang,2)+pow(cos_ang,2)));
    double angleVar = pow(stddev,2);
    double angleStdev = sqrt(angleVar);

    // Compute the mean angle error
    double angleError = computeAngleError(trueTh, meanAngle);
    angleError/=normalizer;
    while (angleError > M_PI)
        angleError -= 2*M_PI;
    while (angleError < -M_PI)
        angleError += 2*M_PI;

    // Compute the angle var error
    double varAngleError = 0.0;
    for(int i=0; i<particles.size(); i++)
    {
        varAngleError   += pow((particles[i].p.yaw-trueTh)-angleError, 2.0)*particles[i].w;
    }
    varAngleError/=normalizer;
    double stdevAngleError = sqrt(varAngleError);
    while (stdevAngleError > M_PI)
        stdevAngleError -= 2*M_PI;
    while (stdevAngleError < -M_PI)
        stdevAngleError += 2*M_PI;
    varAngleError = stdevAngleError*stdevAngleError;
    particleLog  << trueX <<  " " << trueY << " " << " " << meanPX << " " << meanPY << " "
                 << closest.p.x << " " << closest.p.y << " " << closest.p.yaw << " " << closest.w << " "
                 << meanParticleError <<  " " << meanParticleStdev  << " " << meanError << " " << stdevError << " "
                 << trueTh << " " << meanAngle << " " << angleStdev << " " << angleError << " " << stdevAngleError << " "
                 << NEFF << " " << elapsedTime << endl;
    particleLog.flush();
    timer.startLap();
}

void MCL::setNumParticles(int _numParticles)
{
    this->numParticles = _numParticles;
}

void MCL::sampling(Pose3d &u, bool reliable)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> randomValue(-1.0,1.0);
    std::uniform_real_distribution<double> randomS(-0.02578,0.02578);
    std::uniform_real_distribution<double> randomRollPitch(-rollPitchAngRange*(M_PI/180), rollPitchAngRange*(M_PI/180));

    updateOdomPath(u);

    if(reliable)
    {
        #pragma omp parallel for num_threads(NUM_THREADS)
        for(int i=0; i<particles.size(); i++)
        {

            float newX = u.x * (particles[i].p.z);
            float newY = u.y * (particles[i].p.z);

            particles[i].p.x += cos(particles[i].p.yaw)*newX - sin(particles[i].p.yaw)*newY + randomValue(generator)*10.0;
            particles[i].p.y += sin(particles[i].p.yaw)*newX + cos(particles[i].p.yaw)*newY + randomValue(generator)*10.0;
            particles[i].p.yaw += u.yaw + randomValue(generator)*5*M_PI/180.0;
            particles[i].p.roll = randomRollPitch(generator);
            particles[i].p.pitch = randomRollPitch(generator);

            particles[i].p.z = (particles[i].p.z * u.z) + randomS(generator); //Z is a scale, not a displacement, so it is multiplied. If the height does not change in the images, it is equal to 1, and does not change the value

            while(particles[i].p.yaw > M_PI)
                particles[i].p.yaw -= 2*M_PI;
            while(particles[i].p.yaw < -M_PI)
                particles[i].p.yaw += 2*M_PI;
        }
    }
    else
    {
        cout << "!UNRELIABLE!!!" << endl;
        #pragma omp parallel for num_threads(NUM_THREADS)
        for(int i=0; i<particles.size(); i++)
        {
            float newX = u.x * (particles[i].p.z);
            float newY = u.y * (particles[i].p.z);

            particles[i].p.x += cos(particles[i].p.yaw)*newX - sin(particles[i].p.yaw)*newY + randomValue(generator)*10.0;
            particles[i].p.y += sin(particles[i].p.yaw)*newX + cos(particles[i].p.yaw)*newY + randomValue(generator)*10.0;
            particles[i].p.yaw += u.yaw + randomValue(generator)*30*M_PI/180.0;
            particles[i].p.roll = randomValue(generator)*M_PI/180.0;
            particles[i].p.pitch = randomValue(generator)*M_PI/180.0;

            particles[i].p.z = (particles[i].p.z * u.z) + randomS(generator); //Z is a scale, not a displacement, so it is multiplied. If the height does not change in the images, it is equal to 1, and does not change the value

            while(particles[i].p.yaw > M_PI)
                particles[i].p.yaw -= 2*M_PI;
            while(particles[i].p.yaw < -M_PI)
                particles[i].p.yaw += 2*M_PI;
        }
    }
}

void MCL::updateOdomPath(Pose3d &u){
    // Transformation matrix (Z is the scale)
    // Instead of building a matrix operation, the multiplication is a lot at once, making execution faster.
    // It is a rotation and a scale
    odomPose.x += cos(odomPose.yaw)*u.x*odomPose.z - sin(odomPose.yaw)*u.y*odomPose.z;
    odomPose.y += sin(odomPose.yaw)*u.x*odomPose.z + cos(odomPose.yaw)*u.y*odomPose.z;
    odomPose.z *= u.z;
    odomPose.yaw += u.yaw;
    while(odomPose.yaw > M_PI)
        odomPose.yaw -= 2*M_PI;
    while(odomPose.yaw < -M_PI)
        odomPose.yaw += 2*M_PI;

    odomPath.push_back(odomPose);
}

void MCL::weighting(cv::Mat& z_robot, Pose3d &u, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal)
{
    int count = 0;
    double t = (double)cv::getTickCount();

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0;i<particles.size();i++)
    {
        int x=round(particles[i].p.x);
        int y=round(particles[i].p.y);

        // check if particle is not valid (unknown or obstacle)
        if(heuristics[0]->getType() != BRIEF)
            if(!cachedMaps[0]->isKnown(x,y) || cachedMaps[0]->isObstacle(x,y))
            {
                particles[i].w = 0.0;
                count++;
                continue;
            }

        double varUnscented = pow(2.5, 2.0); // normalized gaussian
        double varColor = pow(0.5, 2.0); // normalized gaussian
        double varMI = pow(.15,2.0);
        double varBrief = pow(.15, 2.0);
        double prob=1.0;

        for(int l=0;l<heuristics.size();++l)
        {
            Heuristic* h = heuristics[l];
            int mapID = selectMapID(h->getColorDifference());
            switch(h->getType())
            {
                case BRIEF:
                {
                    double diff;

                    BriefHeuristic* bh = (BriefHeuristic*) heuristics[l];
                    if(x<0 || x>=globalMaps[mapID].cols || y<0 || y>=globalMaps[mapID].rows)
                    {
                        prob=0;
                    }
                    else
                    {
                        if(multispectralCam)
                        {
                            diff = bh->calculateValue2(particles[i].p, &globalMaps[mapID], redMap, nirMap, regMap, greMap, currentMapOriginal, droneAngles[currentImage]);                            
                        }
                        else
                        {
                            diff = bh->calculateValue2(particles[i].p, &globalMaps[mapID]);
                        }
                        prob *= (1.0/(sqrt(varBrief*2.0*M_PI))*exp(-0.5*(pow(diff,2.0)/varBrief)));
                    }
                    break;
                }
            }
        }
        particles[i].w = prob;
    }

    t = (double)cv::getTickCount() - t;
    cout << endl << "Particles calculated in: " << (t*1000./cv::getTickFrequency())/1000 << "s" << endl;
    cout << "I've killed: " << count << " particles." << endl;

    double sumWeights = 0.0;
    for(int i=0; i<particles.size(); i++)
    {
        sumWeights += particles[i].w;
    }
    cout << "SumWeights B " << sumWeights << endl;

    // Correct zero error
    if(sumWeights==0.0)
    {
        for(int i=0; i<particles.size(); i++)
        {
            if(heuristics[0]->getType() != BRIEF)
            {
                // check if particle is valid (known and not obstacle)
                if(!cachedMaps[0]->isKnown((int)particles[i].p.x,(int)particles[i].p.y) ||
                        cachedMaps[0]->isObstacle((int)particles[i].p.x,(int)particles[i].p.y)) {
                    particles[i].w = 0.0;
                }
                else {
                    particles[i].w = 1.0;
                    sumWeights+=1.0;
                }
            }
            else
            {
                if(particles[i].p.x<0 || particles[i].p.y<0 || particles[i].p.y>=this->globalMaps[0].rows
                        || particles[i].p.x>=this->globalMaps[0].cols )
                    particles[i].w = 0.0;
                else
                {
                    particles[i].w = 1.0;
                    sumWeights+=1.0;
                }
            }
        }
    }

    count = 0; //number of particles dead
    neff = 0;  //número de partículas efetivas
    sumW = sumWeights;
    //normalize particles
    if(sumWeights!=0.0)
        for(int i=0; i<particles.size(); i++)
        {
            particles[i].w /= sumWeights;

            if(particles[i].w == 0.0)
                count++;

            neff+=pow(particles[i].w,2.0);
        }
    else
    {
        for(int i=0; i<particles.size(); i++)
            particles[i].w = 1.0/numParticles;
        neff=numParticles;
    }
    cout << "Confirmed, I've killed: " << count << " particles." << endl;
    neff=1.0/neff;
    cout << "NEFF: " << neff << endl;

    if(!quiet)
    {
        cout << "\nAverage particle calculation time (time per particle): " << tmpExecPxCalc/qtdChamadasPxCalc << " s \n";
    }
}

void MCL::resampling()
{
    cout<<"RESAMPLING"<<endl;
    vector<int> children;
    children.resize(numParticles,0);

    // low variance sampler (table 4.4 in Thrun, 2005)
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> randomValue(0.0,1.0/((double)numParticles));

    int i=0;
    double r=randomValue(generator);
    double c=particles[i].w;

    float averageScale = 0;
    float averageRoll = 0;
    float averagePitch = 0;
    float averageYaw = 0;

    for(int m=0; m<particles.size(); m++)
    {
        double U = r + m*1.0/(double)numParticles;
        while(U>c)
        {
            i++;
            c+=particles[i].w;
        }
        children[i]++;
        averageScale += particles[i].p.z;
        averageRoll += particles[i].p.roll;
        averagePitch += particles[i].p.pitch;
        averageYaw += particles[i].p.yaw;
    }

    cout<<"                  AverageScale:"<<averageScale/particles.size()<<"                 "<<endl;
    cout<< std::fixed<< std::setprecision(5)<<"                  AverageRoll:"<<(averageRoll/particles.size()) * (180/M_PI)<<" deg                 "<<endl;
    cout<< std::fixed<< std::setprecision(5)<<"                  AveragePitch:"<<(averagePitch/particles.size()) * (180/M_PI)<<" deg                 "<<endl;
    cout<< std::fixed<< std::setprecision(5)<<"                  AverageYaw:"<<(averageYaw/particles.size())* (180/M_PI)<<" deg                 "<<endl;
    cout<<" ------------------------------------------------------"<<endl;

    // generate children from current particles
    vector<MCLparticle> nextGeneration;

            for(int m=0; m<particles.size(); m++)
            {
                for(int c=0; c<children[m]; c++)
                    nextGeneration.push_back(particles[m]);
            }
    particles = nextGeneration;
}

double MCL::computeNeff()
{
    double Neff=0.0;
    for(int i=0; i<particles.size(); i++)
        Neff += particles[i].w * particles[i].w;
    return 1.0/Neff;
}

double MCL::computeError(double trueX, double trueY,double particleX, double particleY)
{
    return sqrt(pow(trueX-particleX, 2.0) + pow(trueY-particleY, 2.0));
}

double MCL::computeError3d(double trueX, double trueY, double trueZ, double particleX, double particleY, double particleZ)
{
    return sqrt(pow(trueX-particleX, 2.0) + pow(trueY-particleY, 2.0) + pow(trueZ-particleZ, 2.0));
}

double MCL::computeAngleError(double trueTh, double particleTh)
{
    double angleError=particleTh-trueTh;

    while (angleError > M_PI)
        angleError -= 2*M_PI;
    while (angleError < -M_PI)
        angleError += 2*M_PI;

    // error in radians
    return angleError;

}

double MCL::sumAngles(double a, double b)
{
    double c = a + b;

    while (c > M_PI)
        c -= 2*M_PI;
    while (c < -M_PI)
        c += 2*M_PI;

    // error in radians
    return c;
}

void MCL::prepareWeighting(cv::Mat &z, cv::Mat &redMap, cv::Mat &nirMap, cv::Mat &regMap, cv::Mat &greMap, cv::Mat &currentMapOriginal, Pose3d orientacao)
{
    // precomputing halfRows e halfCows
    int halfRows = frameColorConverted[0].rows/2;
    int halfCols = frameColorConverted[0].cols/2;

    // Set mask if not set yet
    if(!binaryFrameMask.data)
        binaryFrameMask = cv::Mat(frameColorConverted[0].cols,
                              frameColorConverted[0].rows,
                              CV_8SC3,cv::Scalar(0,0,0));

    // Compute value at the center of the frame using
    // appropriate color space
    for(int c = 0; c<heuristics.size();++c)
    {
        Heuristic* h = heuristics[c];        
        // get proper color space
        int mapID = selectMapID(h->getColorDifference());
        cout << "Color Difference: "<< h->getColorDifference() << "Color Map: " << mapID << endl;

        double val = 0.0;
        double grad = val;

        BriefHeuristic* bh = (BriefHeuristic*) heuristics[c];
        bh->updateDroneDescriptor(frameColorConverted[mapID], redMap, nirMap, regMap, greMap, currentMapOriginal, orientacao, realPose);
    }
}

void MCL::prepareWeighting(cv::Mat &z)
{
    // precomputing halfRows e halfCows
    int halfRows = frameColorConverted[0].rows/2;
    int halfCols = frameColorConverted[0].cols/2;

    // Set mask if not set yet
    if(!binaryFrameMask.data)
        binaryFrameMask = cv::Mat(frameColorConverted[0].cols,
                              frameColorConverted[0].rows,
                              CV_8SC3,cv::Scalar(0,0,0));

    // Compute value at the center of the frame using
    // appropriate color space    
    for(int c = 0; c<heuristics.size();++c)
    {
        Heuristic* h = heuristics[c];

        // get proper color space
        int mapID = selectMapID(h->getColorDifference());
        cout << "Color Difference: "<< h->getColorDifference() << "Color Map: " << mapID << endl;

        double val = 0.0;
        double grad = val;

        switch(h->getType())
        {
            case BRIEF:
            {

                BriefHeuristic* bh = (BriefHeuristic*) heuristics[c];
                bh->updateDroneDescriptor(frameColorConverted[mapID]);
                break;
            }
        }
    }
}

void MCL::createColorVersions(cv::Mat& imageRGB)
{
    // RGB
    frameColorConverted[0]=imageRGB.clone();

    // INTENSITYC:
    cvtColor(imageRGB, frameColorConverted[1],CV_BGR2GRAY);
    cvtColor(frameColorConverted[1], frameColorConverted[1],CV_GRAY2BGR);

    // CIELAB1976 || CIELAB1994 || CMCLAB1984 || CIELAB2000 || CIELAB1994MIX || CIELAB2000MIX
    cvtColor(imageRGB,frameColorConverted[2], CV_BGR2Lab);
}

void MCL::normalizingParticles(){
    double sumWeights = 0.0;

    for(int i=0; i < particles.size(); i++)
        sumWeights += particles[i].w;

    for(int i=0; i < particles.size(); i++)
        particles[i].w /= sumWeights;
}

void MCL::setingEmptyBins(){
    bins.clear();

    for(int i = 0; i < globalMaps[0].cols;i++)
    {
        vector<bool> temp;
        for(int j = 0; j < globalMaps[0].rows;j++)
        {
            temp.push_back(true);
        }
        bins.push_back(temp);
    }
}

bool MCL::isEmpty(MCLparticle part){
    if(bins[(int)part.p.x][(int)part.p.y]) //THE bin(x,y) IS EMPTY
    {
        bins[(int)part.p.x][(int)part.p.y] = false;
        return true;
    }
    else //THE bin(x,y) IS NON-EMPTY
    {
        return false;
    }
}

bool MCL::verifyParticlePosition(MCLparticle part)
{
    if(part.p.x >= 0 && part.p.x <= globalMaps[0].cols && part.p.y >= 0 && part.p.y <= globalMaps[0].rows)
    {
        return false;
    }
    else
    {
        return true;
    }
}
