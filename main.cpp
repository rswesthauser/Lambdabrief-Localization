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

#include <pthread.h>
#include <iostream>
#include <iostream>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include "config.h"
#include "src/Robot/Robot.h"
#include "src/Robot/DroneRobot.h"
#include "src/Utils/GlutClass.h"
#include <unistd.h>
#include "src/Utils/RadiusVolumeTransferFunctions.h"

using std::cout;
using std::endl;
using std::cerr;
using namespace std;

ConnectionMode connectionMode;
LogMode logMode;
string filename;
pthread_mutex_t* mutex;
bool quiet=false;
int numParticles = 0;
bool multispectralCam=true;             // By default I use multispectral, you don't need to enter the parameter.
int ndvi_type = 1;                      //Default: NDVI
int grvi_type = 0;
double minScale = 0;
double maxScale = 0;
int msImgPreProc = 0;                   //Default: without pre-processing
double gbSigma = 0;
double gbKernel = 0;
int bfD = 0;
double bfSigmaColor = 0;
double bfSigmaSpace = 0;
int mapPreProc = 1;                     //Default: quantization
int pixelDistMethod = 1;
int poolSelFit = 0;
int imgIniDs = 0;
int imgIniExec = 0;
int imgFimDs = 0;
int imgFimExec = 0;
float multiGaussianMask = 0;
float alphaVegetationMask = 0;
float betaHammingWeight = 0;            // Used to calculate the weight of the pixel pairs, based on the NDVI vegetation mask.
int currentImage = 0;                   // Initialized at 0 (currentImage + iniexec + 1 = current image number)
double tmpExecPxDist = 0;               // Counts the total execution time of the calls to the pixel distribution.
int PxDistCall = 0;              // Number of times the particle distribution was called.
double tmpExecPxCalc = 0;               // Counts the total execution time of the calls to the calculation of the pixel positions.
int qtdChamadasPxCalc = 0;              // Number of times the pixel position calculation was called
cv::Mat originalMap;                    // Original global map, with no processing
int rollPitchAngRange;                  // Range in which roll and pitch can vary, in particle estimation
double percentualPxVegImg = 0;

void* startRobotThread (void* ref)
{
    Robot* robot=(Robot*) ref;
    int k = 0;

    robot->initialize(connectionMode, logMode, filename, numParticles);
    while(robot->isRunning()){
        cout<<"                                            K = "<<k++<<endl;
        if(k > 6)
            sleep(1);
        robot->run();
    }
    return NULL;
}

void* startGlutThread (void* ref)
{
    sleep(2);
    GlutClass* glut=GlutClass::getInstance();
    glut->setRobot((Robot*) ref);

    glut->initialize();

    glut->process();

    return NULL;
}

// Standard Input Error Message
bool errorMessage(int position=-1, std::string message="")
{
    // Detailed message
    if(message.size()!=0)
        cerr << message << endl;

    // Print character position if available
    if(position>0)
        cerr << "Failure at input element "<< position << endl;

    // Standard error message
    cerr << "Usage: LambdaBrief-Localization -s <diff-intensity|diff-rgb|diff-cie1976|diff-cmc1984|diff-cie1994|diff-cie2000|diff-cie1994mix|diff-cie2000mix> <threshold> <gaussian|circular|inverted> <radius>" << endl;

    return false;
}

bool config(int argc, char* argv[], vector< heuristicType* > &heuristicTypes, std::string& mp, std::string& tp, std::string& op, int& start, int& finish)
{
    //data to collect
    cv::Mat image;
    cv::Mat map;
    std::string outputName;
    int p=1;

    quiet=false;    //show the graphical interface (false = show, true = no show)

    while(p<argc)
    {
        // print help and exit
        if(!strncmp(argv[p], "-h", 2) || !strncmp(argv[p], "-H", 2) || !strncmp(argv[p], "--help", 6))
        {
            cout << "Usage: LambdaBrief-Localization -e <path/> -o <path/outputDir/> -s <diff-intensity|diff-rgb|diff-cie1976|diff-cmc1984|diff-cie1994|diff-cie2000|diff-cie1994mix|diff-cie2000mix> <double> <gaussian|circular|inverted> <int>" << endl;
            exit(0);
        }
        else if(!strncmp(argv[p], "-quiet", 6))
        {
            quiet=true;
            p++;
        }
        // Check if this is the filename part -- step 1. The output configuration parameters.
        else if(!strncmp(argv[p], "-o", 2) || !strncmp(argv[p], "-O", 2))
        {
            // check if there is a file name
            if(argc>p+3)
            {
                op=argv[p+1];
                start=stoi(argv[p+2]);   //initial number of the output file
                finish=stoi(argv[p+3]);  //final number of the output file
                p+=4;
            }
            else
            {
                return errorMessage(p,"Failed to load output, missing argument");
            }
        }
        // Check if this is the filename part -- step 1. The globalmap path.
        else if(!strncmp(argv[p], "-e", 2) || !strncmp(argv[p], "-E", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                mp=argv[p+1];
                p+=2;
            }
            else
            {
                return errorMessage(p,"Failed to load map, missing argument");
            }
        }
        // Check the trajectory files.
        else if(!strncmp(argv[p], "-t", 2) || !strncmp(argv[p], "-T", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                tp=argv[p+1];
                p+=2;
            }
            else
            {
                return errorMessage(p,"Failed to load trajectory, missing argument");
            }
        }
        // Check the strategies files and parameters.
        else if(!strncmp(argv[p], "-s", 2) || !strncmp(argv[p], "-S", 2))
        {
            // initialize heuristic config
            heuristicType* ht = new heuristicType();

            // check if there is an appropriate strategy type
            if(argc>=p+3)
            {
                // store strategy
                std::string s(argv[p+1]);

                if(s.compare("BRIEF")==0 || s.compare("brief")==0)
                    ht->strategy=BRIEF;
                else
                    return errorMessage(p+1, "Invalid strategy: " + s);
            } else
                return errorMessage(p+1, "Insuficient strategy information:");

            // Validate and store color difference type
            std::string color_diff(argv[p+2]);
            if(color_diff.compare("diff-intensity")==0 || color_diff.compare("DIFF-INTENSITY")==0)
                ht->colorDifference = INTENSITYC;
            if (color_diff.compare("diff-rgb")==0 || color_diff.compare("DIFF-RGB")==0)
                ht->colorDifference = RGBNORMA;
            if(color_diff.compare("diff-cie1976")==0   || color_diff.compare("DIFF-CIE1976")==0)
                ht->colorDifference = CIELAB1976;
            if(color_diff.compare("diff-cmc1984")==0   || color_diff.compare("DIFF-CMC1984")==0)
                ht->colorDifference = CMCLAB1984;
            if(color_diff.compare("diff-cie1994")==0   || color_diff.compare("DIFF-CIE1994")==0)
                ht->colorDifference = CIELAB1994;
            if(color_diff.compare("diff-cie2000")==0   || color_diff.compare("DIFF-CIE2000")==0)
                ht->colorDifference = CIELAB2000;
            if(color_diff.compare("diff-cie1994mix")==0|| color_diff.compare("DIFF-CIE1994MIX")==0)
                ht->colorDifference = CIELAB1994MIX;
            if(color_diff.compare("diff-cie2000mix")==0|| color_diff.compare("DIFF-CIE2000MIX")==0)
                ht->colorDifference = CIELAB2000MIX;

            //Check for correct initialization
            if(ht->colorDifference == -1)
                return errorMessage(p+2, "Invalid color difference: " + color_diff);

            // Check if this is the threshold for color difference is valid
            try {
                ht->threshold = std::stof(argv[p+3]);
            }
            catch (const std::invalid_argument& ia) {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+3, "Color threshold is a double, example: -t 2.3\n");
            }

            // Catching silly color threshold
            if(ht->threshold<0)
                return errorMessage(p+3, "Color limiar cannot be negative: " + to_string(ht->threshold));

            //increasing the string position
            p+=4;

            // Store heuristic
            heuristicTypes.push_back(ht);
        }
        else if(!strncmp(argv[p], "-bp", 3) || !strncmp(argv[p], "-BP", 3))
        {
            heuristicType* ht = heuristicTypes[heuristicTypes.size()-1]; //The last heuristic added
            try {
                ht->numberPairs = std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia) {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Number of pairs is a integer, example: -bp 1000\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-blt", 4) || !strncmp(argv[p], "-BLT", 4))
        {
            heuristicType* ht = heuristicTypes[heuristicTypes.size()-1];//The last heuristic added
            try {
                ht->lowThreshold = std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia) {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Low threshold is a float between 0 and 0.99, example: -blt 0.55\n");
            }
            if (ht->lowThreshold >= 1 || ht->lowThreshold < 0){
                return errorMessage(p+1, "Low threshold is a float between 0 and 0.99, example: -blt 0.55\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-bmt", 4) || !strncmp(argv[p], "-BMT", 4))
        {
            heuristicType* ht = heuristicTypes[heuristicTypes.size()-1];//The last heuristic added
            try {
                ht->multiplierThreshold = std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia) {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Multiplier threshold is a float, example: -bmt 10.5\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-bm", 3) || !strncmp(argv[p], "-BM", 3))
        {
            heuristicType* ht = heuristicTypes[heuristicTypes.size()-1];
            try {
                ht->margin = std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia) {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Margin is a integer, example: -bmt 10\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-unp", 4) || !strncmp(argv[p], "-UNP", 4))
        {
            heuristicType* ht = heuristicTypes[heuristicTypes.size()-1];//The last heuristic added
            try {
                ht->unscentedNumberPoints  = std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia) {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Number of Points is a integer, example: -np 100\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-par", 4) || !strncmp(argv[p], "-PAR", 4))
        {
            try
            {
                numParticles =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Number of particles is a integer, example: -par 50000\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-msc", 4) || !strncmp(argv[p], "-MSC", 4))
        {
            try
            {
                multispectralCam =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "The use of a multispectal camera is a boolean, example: -msc 1\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-ndvit", 6) || !strncmp(argv[p], "-NDVIT", 6))
        {
            try
            {
                ndvi_type =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "NDVI Type is a integer, example: -ndvit 1\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-grvit", 6) || !strncmp(argv[p], "-GRVIT", 6))
        {
            try
            {
                grvi_type =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "GRVI Type is a integer, example: -grvit 1\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-minsc", 6) || !strncmp(argv[p], "-MINSC", 6))
        {
            try
            {
                minScale =  std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Min Scale  Type is a double, example: -minsc 1.9\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-maxsc", 6) || !strncmp(argv[p], "-MAXSC", 6))
        {
            try
            {
                maxScale =  std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Max Scale  Type is a double, example: -maxsc 1.9\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-msimp", 6) || !strncmp(argv[p], "-MSIMP", 6))
        {
            try
            {
                msImgPreProc =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Multispectral image pre-processing is a int:\n "
                                         "0: without pre-processing\n "
                                         "1: gaussian filter\n "
                                         "2: bilateral filtering.\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-gbs", 6) || !strncmp(argv[p], "-GBS", 6))
        {
            try
            {
                gbSigma =  std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Gaussian blur sigma.\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-gbk", 6) || !strncmp(argv[p], "-GBK", 6))
        {
            try
            {
                gbKernel =  std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Gaussian blur kernel size.\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-bfd", 6) || !strncmp(argv[p], "-BFD", 6))
        {
            try
            {
                bfD =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Bilateral Filter D\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-bfsc", 6) || !strncmp(argv[p], "-BFSC", 6))
        {
            try
            {
                bfSigmaColor =  std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Bilateral Filter sigma color\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-bfss", 6) || !strncmp(argv[p], "-BFSS", 6))
        {
            try
            {
                bfSigmaSpace =  std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Bilateral Filter sigma space\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-mappp", 6) || !strncmp(argv[p], "-MAPPP", 6))
        {
            try
            {
                mapPreProc =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Local and global map pre-processing is a int:\n "
                                         "1: quantization\n "
                                         "2: bilateral filtering.\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-pdm", 4) || !strncmp(argv[p], "-PDM", 4))
        {
            try
            {
                pixelDistMethod =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Pixel Distribution Method is a int:\n "
                                         "1:  Gaussian\n"
                                         "11: Old Gaussian\n"
                                         "2:  Roulette-wheel selection via stochastic acceptance.\n "
                                         "3:  Roulette-wheel\n"
                                         "31: Roulette-wheel + Gaussian\n"
                                         "4:  Tournament\n"
                                         "41: Tournament + Gaussian\n");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-fit", 4) || !strncmp(argv[p], "-FIT", 4))
        {
            try
            {
                poolSelFit = std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Pool Selection Fitness is a int:\n "
                                         "0: defaut operation \ n"
                                         "1: disregard all vegetation \ n"
                                         "2: disregard the common vegetation, working only with the most dense. \ N"
                                         "3: Accept all pixels that are not vegetation");
            }
            p++;
        }        
        else if(!strncmp(argv[p], "-inids", 6) || !strncmp(argv[p], "-INIDS", 6))
        {
            try
            {
                imgIniDs = std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Initial dataset image.");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-iniexec", 8) || !strncmp(argv[p], "-INIEXEC", 8))
        {
            try
            {
                imgIniExec = std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Initial execution image.");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-fimds", 6) || !strncmp(argv[p], "-FIMDS", 6))
        {
            try
            {
                imgFimDs = std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Final dataset image.");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-fiexec", 7) || !strncmp(argv[p], "-FIEXEC", 7))
        {
            try
            {
                imgFimExec = std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Final execution image.");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-multgaussmsk", 13) || !strncmp(argv[p], "-MULTGAUSSMSK", 13))
        {
            try
            {
                multiGaussianMask = std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Multiplier of the Gaussian mask used in conjunction with roulette by VI. Ex: with 0.5, the Gaussian mask will only exert 50% influence.");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-alfavismsk", 11) || !strncmp(argv[p], "-ALFAVIMSK", 11))
        {
            try
            {
                alphaVegetationMask = std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Alpha of the vegetation mask used in conjunction with roulette by VI. Ex: with 0.5, the Gaussian mask will only exert 50% influence.");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-betaweight", 11) || !strncmp(argv[p], "-BETAWEIGHT", 11))
        {
            try
            {
                betaHammingWeight = std::stof(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Used to calculate the weight of the pixel pairs, based on the NDVI vegetation mask.");
            }
            p++;
        }
        else if(!strncmp(argv[p], "-rpang", 6) || !strncmp(argv[p], "-RPANG", 6))
        {
            try
            {
                rollPitchAngRange =  std::stoi(argv[p+1]);
            }
            catch (const std::invalid_argument& ia)
            {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+1, "Troll and pitch ranges is a integer, example: -rpang 5\n");
            }
            p++;
        }
        else
            p++;
    }
    return true;
}

int main(int argc, char* argv[])
{

    // Global variables
    connectionMode = SIMULATION;    //Robot.h
    logMode = NONE;                 //Utils.h
    filename = "";

    // load config from command line
    std::string mapPath, trajPath, outputPath;
    int start=-1;
    int finish=-1;

    vector< heuristicType* > heuristicTypes;
    if(!config(argc, argv, heuristicTypes, mapPath, trajPath, outputPath, start, finish))
        exit(1);

    Robot* r;
    std::cout<<"CONFIGURATED"<<std::endl;
    r = new DroneRobot(mapPath,trajPath,heuristicTypes,quiet,outputPath,start,finish);

    if(quiet){
        startRobotThread((void*)r);        
    }else{
        pthread_t robotThread, glutThread;
        mutex = new pthread_mutex_t;
        pthread_mutex_unlock(mutex);

        pthread_create(&(robotThread),NULL,startRobotThread,(void*)r);
        pthread_create(&(glutThread),NULL,startGlutThread,(void*)r);

        pthread_join(robotThread, 0);
        pthread_join(glutThread, 0);
    }
    return 0;
}
