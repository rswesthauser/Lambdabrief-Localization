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

#ifndef __COLORCPU__
#define __COLORCPU__

enum COLOR_NAME{
    WHITE,
    BLACK,
    RED,
    GREEN,
    BLUE,
};

#include <vector>
#include "src/Utils/mat3x3.h"

#define RGBTOGRAYSCALE 0
#define RGBTOCIELAB 1
#define RGBTOCIELCH 2
#define RGBTORGB 3

/* reference white in XYZ coordinates

Constants used to Lab conversion taken from
http://www.brucelindbloom.com/index.html?Eqn_Spect_to_XYZ.html

*/
//#define A  	1.09850f,  	1.00000f,  	0.35585f
//#define B 	0.99072f, 	1.00000f, 	0.85223f
//#define C 	0.98074f, 	1.00000f, 	1.18232f
//#define D50 0.96422f, 	1.00000f, 	0.82521f
//#define D55 0.95682f, 	1.00000f, 	0.92149f
#define D65 0.95047f, 	1.00000f, 	1.08883f
//#define D75 0.94972f, 	1.00000f, 	1.22638f
//#define E 	1.00000f, 	1.00000f, 	1.00000f
//#define F2 	0.99186f, 	1.00000f, 	0.67393f
//#define F7 	0.95041f, 	1.00000f, 	1.08747f
//#define F11 1.00962f, 	1.00000f, 	0.64350f

#define gammasRGB 2.2f


#define eZero .0001f	
static double isZero(double value){
	return (sqrt(pow(value, 2)) >= eZero) ? value : 0.0f;
}

class CPUColorConverter
{
public:
	CPUColorConverter(void);
	~CPUColorConverter(void);

	//CPU
//	double MIN(double a, double b, double c);
//	double MAX(double a, double b, double c);
	//RGB to Normalized RGB and backward
	static vec3 RGB255toNormalizedRGB(const vec3& colorRGB255);
	// Conversion from sRGB to rgb
	static vec3 sRGBtorgb(const vec3& colorRGB);
	// rgb to XYZ
	static vec3 rgbXYZ(const vec3& rgbColor);
	//XYZ to RGB
	static vec3 XYZCIELAB(const vec3& colorXYZ, vec3 illuminant=vec3(D65));
	// sRGB to L*a*b
	static vec3 sRGBCIELAB(const vec3& colorRGB);
	static vec3 RGB255toCIELAB(const vec3& colorRGB255);
	static vec3 CIELABLCH(const vec3& labcolor);
	static vec3 RGB255toCIELCH(const vec3& colorRGB255);
	static vec3 RGB255toGrayscale(const vec3& colorRGB255);

	void cpuRGBtoCIELABConversion(const unsigned char *frame, std::vector<vec3> &output);
	void cpuRGBtoCIELCHConversion(const unsigned char *frame, std::vector<vec3> &output);
	void cpuRGBtoGrayscaleConversion(const unsigned char *frame, std::vector<vec3> &output);
	void cpuRGBtoRGB(const unsigned char *frame, std::vector<vec3> &output);
	void cpuColorConversion(const unsigned char *frame, std::vector<vec3> &output, int conversion = RGBTOCIELAB);
	static double DeltaECIE1976(vec3 colorLab1, vec3 colorLab2);
	static double DeltaECIE1994(vec3 colorLab1, vec3 colorLab2);
	static double DeltaECMC1984(vec3 colorLab1, vec3 colorLab2, double l=1.0f, double c=1.0f);
	static double DeltaEMixCIE1994(vec3 colorLab1, vec3 colorLab2);

private:
	double m_WhitePoint;
	double m_Gamma;
};

#endif
