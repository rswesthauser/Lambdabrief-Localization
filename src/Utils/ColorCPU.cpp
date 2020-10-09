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

#include "ColorCPU.h"

CPUColorConverter::CPUColorConverter(void)
{
    this->m_WhitePoint = 0;
    this->m_Gamma = 0;
}

CPUColorConverter::~CPUColorConverter(void)
{}

vec3 CPUColorConverter::RGB255toNormalizedRGB(const vec3& colorRGB255)
{
	return vec3(colorRGB255/255.0);
}

vec3 CPUColorConverter::sRGBtorgb(const vec3& colorRGB)
{
	//sRGB factor, gamma power and threshold
	const double factor=.055f;
	const double gammaPower=2.4f;
	const double threshold=.04045f;
	//sRGB has two separated functions
	vec3 result;
	if(colorRGB.r > threshold)
		result.r=pow((colorRGB.r+factor)/1.055f, gammaPower);
	else
		result.r=colorRGB.r/12.92f;
	if(colorRGB.g > threshold)
		result.g=pow((colorRGB.g+factor)/1.055f, gammaPower);
	else
		result.g=colorRGB.g/12.92f;
	if(colorRGB.b > threshold)
		result.b=pow((colorRGB.b+factor)/1.055f, gammaPower);
	else
		result.b=colorRGB.b/12.92f;

	//return color1 if mixBoolean is false and color2 if mix boolean is true
	return result;
}
vec3 CPUColorConverter::rgbXYZ(const vec3& rgbColor)
{
	//Conversion Matrix from rgb to XYZ D65 illuminant
	mat3x3 rgbtoXYZ(   
		vec3(0.4124564f,0.2126729f,0.0193339f), 
		vec3(0.3575761f,0.7151522f,0.1191920f), 
		vec3(0.1804375f,0.0721750f,0.9503041f));

	return vec3(rgbtoXYZ*rgbColor);
}

vec3 CPUColorConverter::XYZCIELAB(const vec3& colorXYZ, vec3 illuminant)
{
	double Ka = 24389.0f/27.0f;
	double Epsilon = 216.0f/24389.0f;

	//making XYZ relative to the illuminant D65
	vec3 oldcolorXYZ = colorXYZ/illuminant;
	//XYZ factors, cubic root and threshold
	const double Kfactor=Ka;
	const double cubicRoot=1.0f/3.0f;
	const double threshold=Epsilon;

	//conversion has two separated functions
	vec3 result;
	if(oldcolorXYZ.r > threshold)
		result.r=pow(oldcolorXYZ.r,cubicRoot);
	else
		result.r=(oldcolorXYZ.r*Kfactor+16.0f)/116.0f;
	if(oldcolorXYZ.g > threshold)
		result.g=pow(oldcolorXYZ.g,cubicRoot);
	else
		result.g=(oldcolorXYZ.g*Kfactor+16.0f)/116.0f;
	if(oldcolorXYZ.b > threshold)
		result.b=pow(oldcolorXYZ.b,cubicRoot);
	else
		result.b=(oldcolorXYZ.b*Kfactor+16.0f)/116.0f;

	double L = (result.y*116.0f-16.0f);
	double a = 500.0f*(result.x-result.y);
	double b = 200.0f*(result.y-result.z);
	L = isZero(L);
	a = isZero(a);
	b = isZero(b);

	//Calculating the Lab component values
	return vec3(L, a, b); 
}

// sRGB to L*a*b
vec3 CPUColorConverter::sRGBCIELAB(const vec3& colorRGB)
{
	// it is not scaled to one
	return XYZCIELAB(rgbXYZ(colorRGB));
}

vec3 CPUColorConverter::RGB255toCIELAB(const vec3& colorRGB255)
{
	return XYZCIELAB(rgbXYZ(sRGBtorgb(RGB255toNormalizedRGB(colorRGB255))));
}

vec3 CPUColorConverter::CIELABLCH(const vec3 &labcolor)
{	
	double H = degrees(atan2(labcolor.z, labcolor.y));
	return vec3(
		labcolor.x,
		sqrt(pow(labcolor.y, 2)+pow(labcolor.z, 2)),
		H>=0 ? H : H+360.0f);
}

vec3 CPUColorConverter::RGB255toCIELCH(const vec3& colorRGB255)
{
	return CIELABLCH(XYZCIELAB(rgbXYZ(RGB255toNormalizedRGB(colorRGB255))));
}

vec3 CPUColorConverter::RGB255toGrayscale(const vec3& colorRGB255)
{
	vec3 grayFactors = vec3(0.2989, 0.5870, 0.1140);
	vec3 color = RGB255toNormalizedRGB(colorRGB255);
    double gray = vec3::dot(color, grayFactors);
	return vec3(gray, gray, gray); //works
}

void CPUColorConverter::cpuRGBtoCIELABConversion(const unsigned char *frame, std::vector<vec3> &output)
{
	size_t size = output.size()*3;
	for(size_t i=0; i < size; i=i+3)
	{
		vec3 color(frame[i], frame[i+1], frame[i+2]);
		color =  RGB255toCIELAB(color);
		memcpy(&output[i/3], &color, sizeof(double)*3);
	}

}

void CPUColorConverter::cpuRGBtoCIELCHConversion(const unsigned char *frame, std::vector<vec3> &output)
{
	size_t size = output.size()*3;
	for(size_t i=0; i < size; i=i+3)
	{
		unsigned char tempData[3];
		memcpy(&tempData[0], &frame[i], sizeof(unsigned char)*3);
		vec3 color(tempData[0], tempData[1], tempData[2]);
		color =  RGB255toCIELCH(color);	
		output[i/3]=color;
	}
}

void CPUColorConverter::cpuRGBtoGrayscaleConversion(const unsigned char *frame, std::vector<vec3> &output)
{
	size_t size = output.size()*3;
	for(unsigned int i=0; i < size; i=i+3)
	{
		unsigned char tempData[3];
		memcpy(&tempData[0], &frame[i], sizeof(unsigned char)*3);
		vec3 color(tempData[0], tempData[1], tempData[2]);
		color =  RGB255toGrayscale(color);	
		output[i/3]=color;
	}
}

void CPUColorConverter::cpuRGBtoRGB(const unsigned char *frame, std::vector<vec3> &output)
{
	size_t size = output.size()*3;
	for(unsigned int i=0; i < size; i=i+3)
	{
		unsigned char tempData[3];
		memcpy(&tempData[0], &frame[i], sizeof(unsigned char)*3);
		vec3 color(tempData[0], tempData[1], tempData[2]);
		color =  RGB255toNormalizedRGB(color);
		output[i/3]=color;
	}
}

void CPUColorConverter::cpuColorConversion(const unsigned char *frame, std::vector<vec3> &output, int conversion)
{
	switch(conversion)
	{
	case RGBTOCIELCH:
		cpuRGBtoCIELCHConversion(frame, output);
		break;
	case RGBTOCIELAB:
		cpuRGBtoCIELABConversion(frame, output);
		break;
	case RGBTOGRAYSCALE:
		cpuRGBtoGrayscaleConversion(frame, output);
		break;
	case RGBTORGB:
		cpuRGBtoRGB(frame, output);
		break;
	default:
        //cout << "unknown color conversion\n";
		break;
	};
}

double CPUColorConverter::DeltaECIE1976(vec3 colorLab1, vec3 colorLab2)
{
	return vec3::distance(colorLab1, colorLab2);
}

double CPUColorConverter::DeltaECIE1994(vec3 colorLab1, vec3 colorLab2)
{	
	//for Textiles KL = 2.0f
	double K1 = 0.045f, K2 = 0.015f;

	vec3 colorLCH1 = CIELABLCH(colorLab1);
	vec3 colorLCH2 = CIELABLCH(colorLab2);
	//Calculating DeltaL
   	double deltaL = colorLab1.x - colorLab2.x;
	double deltaA = colorLab1.y - colorLab2.y;
	double deltaB = colorLab1.z - colorLab2.z;
	double deltaC = colorLCH1.y - colorLCH2.y;
	double deltaH2 = deltaA*deltaA+deltaB*deltaB-deltaC*deltaC;

	double SL =1.0f;
	double SC =1.0f+K1*colorLCH1.y;
	double SH =1.0f+K2*colorLCH1.y;

	//Checking if the square root will be negative
	if(deltaH2>=0)
	{
        //Graphic Arts
        double KL = 1.0f, KC = 1.0f, KH = 1.0f;
		double deltaH = sqrt(deltaH2);
		return sqrt(pow(deltaL/(KL*SL),2) + pow(deltaC/(KC*SC),2) + pow(deltaH/(KH*SH), 2));
	}
	else
		return sqrt(deltaL*deltaL+deltaA*deltaA+deltaB*deltaB);
}

double CPUColorConverter::DeltaECMC1984(vec3 colorLab1, vec3 colorLab2, double l, double c)
{
	vec3 colorLCH1 = CIELABLCH(colorLab1);
   	vec3 colorLCH2 = CIELABLCH(colorLab2);
	
	//Calculating DeltaL
   	double deltaL = colorLab1.x - colorLab2.x;
	double deltaA = colorLab1.y - colorLab2.y;
	double deltaB = colorLab1.z - colorLab2.z;
	double deltaC = colorLCH1.y - colorLCH2.y;
    double dh = deltaA*deltaA+deltaB*deltaB-deltaC*deltaC;

    // avoid negative square-root
    if(dh >=0)
    {
        double deltaH = sqrt(dh);

        //Calculating SL
        double SL = (colorLab1.x < 16.0f)
                ? 0.511f
                : 0.040975f*colorLab1.x/(1.0f+0.01765f*colorLab1.x);
        //Calculating SC
        double SC = .0638f*colorLCH1.y/(1.0f+0.0131f*colorLCH1.y)+0.638f;
        /******* reach SH **********/
        //Calculate T
        double T = (164.0f <= colorLCH1.z && colorLCH1.z < 345.0f)
                ? (0.56f + abs(0.2f*cos(radians(colorLCH1.z+168.0f))))
                : (0.36f + abs(0.4f*cos(radians(colorLCH1.z+ 35.0f))));
        // Calculating C1^4 and F
        double Cab4 = pow(colorLCH1.y, 4);
        double F = sqrt(Cab4/(Cab4+1900.0f));
        //finally SH
        double SH = SC*(F*T+1.0f-F);

        //Checking if the square root will be negative
        double squaredValue = pow(deltaL/(l*SL), 2) + pow(deltaC/(c*SC), 2) + pow(deltaH/SH, 2);
        return sqrt(squaredValue);

    }
	else
        return sqrt(deltaL*deltaL+deltaA*deltaA+deltaB*deltaB);
}

double CPUColorConverter::DeltaEMixCIE1994(vec3 colorLab1, vec3 colorLab2)
{
		double kL = 1.0; double k1 = .045; double k2 = .015;
		colorLab2 =  colorLab1*0.25+colorLab2*0.75;
		vec3 colorLCH1 = CIELABLCH(colorLab1);
		vec3 colorLCH2 = CIELABLCH(colorLab2);
		//Calculating DeltaL
		double deltaL = colorLCH1.x - colorLCH2.x;
		//calculating DeltaC
		double deltaC = sqrt(pow(colorLCH2.y, 2.0)+pow(colorLCH2.z, 2.0))-sqrt(pow(colorLCH1.y, 2.0)+pow(colorLCH1.z, 2.0));
		//Calculating deltaH
		double DEab1976 = sqrt(pow(colorLab1.x-colorLab2.x, 2.0) + pow(colorLab1.y-colorLab2.y, 2.0) + pow(colorLab1.z-colorLab2.z, 2.0));
		double deltaH = sqrt(pow(DEab1976, 2.0)-pow(deltaL, 2.0)-pow(deltaC, 2.0));
		double DEab1994 =	pow((deltaL)/kL, 2.0)+
							pow((deltaC)/(1.0+k1*colorLCH1.y), 2.0)+
							pow((deltaH)/(1.0+k2*colorLCH1.y), 2.0);
		return (DEab1976 < 5.0 && deltaH >= 0.0) ? DEab1994 : DEab1976;
}
