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

#ifndef __MAT3X3_H__
#define __MAT3X3_H__

#include "src/Utils/vec3.h"

class mat3x3
{
public:

	mat3x3(void){}
	mat3x3(double b11,double b12,double b13,double b21,double b22,double b23,double b31,double b32,double b33)
	{
		a11=b11; a12=b12; a13=b13; 
		a21=b21; a22=b22; a23=b23; 
		a31=b31; a32=b32; a33=b33;
	}
	mat3x3(const vec3& v1, const vec3& v2, const vec3& v3)
	{
		a11=v1.x; a12=v1.y; a13=v1.z; 
		a21=v2.x; a22=v2.y; a23=v2.z; 
		a31=v3.x; a32=v3.y; a33=v3.z;
	}
	
	vec3 operator * (const vec3& vector) const
	{
		return vec3(
				vector.x*a11 + vector.y*a21 + vector.z*a31,
				vector.x*a12 + vector.y*a22 + vector.z*a32,
				vector.x*a13 + vector.y*a23 + vector.z*a33);
	}

	mat3x3 operator * (const mat3x3& other) const
	{
		return mat3x3(
			a11*other.a11 + a12*other.a21 + a13*other.a31,
			a11*other.a12 + a12*other.a22 + a13*other.a32,
			a11*other.a13 + a12*other.a23 + a13*other.a33,
			a21*other.a11 + a22*other.a21 + a23*other.a31,
			a21*other.a12 + a22*other.a22 + a23*other.a32,
			a21*other.a13 + a22*other.a23 + a23*other.a33,
			a31*other.a11 + a32*other.a21 + a33*other.a31,
			a31*other.a12 + a32*other.a22 + a33*other.a32,
			a31*other.a13 + a32*other.a23 + a33*other.a33);
	}
	mat3x3 operator * (double x) const
	{
		return mat3x3(
		a11*x, a12*x, a13*x, 
		a21*x, a22*x, a23*x,
		a31*x, a32*x, a33*x);
	}

	~mat3x3(void){}

	double a11, a12, a13, a21, a22, a23, a31, a32, a33;

};
#endif
