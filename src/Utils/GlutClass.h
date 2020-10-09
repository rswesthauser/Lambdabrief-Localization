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

#ifndef __GLUTCLASS_H__
#define __GLUTCLASS_H__

class GlutClass;

#include "src/Robot/Robot.h"
#include "src/Utils/Utils.h"
#include <GL/glew.h>
#include <GL/glut.h>
#include <opencv2/core/core.hpp>

class GlutClass
{
    public:
        static GlutClass* getInstance();

        void initialize();
        void process();
        void terminate();
        void screenshot(string name = "");
        void setRobot(Robot* r);
        bool drawRobotPath;
        int glutWindowSize;
        int frame;
        int halfWindowSize;
        int x_aux, y_aux;

    private:
        GlutClass ();
        static GlutClass* instance;

        Robot* robot_;
        Grid* grid_;
        Timer timer;

        int halfWindowSizeX_, halfWindowSizeY_;
        bool lockCameraOnRobot;
        int id_;
	    void render();

        static void display();
        static void reshape(int w, int h);
        static void keyboard(unsigned char key, int x, int y);
        static void specialKeys(int key, int x, int y);
};

GLuint matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);

#endif /* __GLUT_H__ */
