/*
QuatRotLib v0.1  3/8/2012
A minimalist class to represent quaternion for use in rotations

Assumes a right-handed co-ordinate system, as in OpenGL, unlike DirectX which takes a left-hand. 
If enough people bug me (barrett@technarian.com), I'll whip up some functionality for left-handed via a #define

This library was largely written from memory, but I did double check the matrix conversion 
by comparing to http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation

If you're planning on using this exclusively with OpenGL, I would suggest doing a find replace for float with GLfloat,
though this is not required.

This library is licensed under the FreeBSD license, which is as follows:

*******************************************************************************
Copyright (c) 2012, Christian Barrett
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*******************************************************************************
 */


#ifndef __QUAT_H__
#define __QUAT_H__

#define USE_STL_CONTAINER //Commenting this removes stl support for vectors.


#include <memory.h>
#include <math.h>

const float TOLERANCE = 0.00001f; //used to determine when quaternions need to be normalized due to precision errors. Adjust this to adjust how frequently you normalize
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef USE_STL_CONTAINER
#include <vector>
using namespace std;
#endif


class Quat
{
	float q[4]; //internal quaternion representation
public:
	//Constructors:
	Quat(void){memset(q,0,sizeof(float)*4);q[3] = 1.f;}

	Quat(float x, float y, float z, float w){q[0]=x;q[1]=y;q[2]=z;q[3]=w;}

	Quat(const float *newq){memcpy(q,newq,sizeof(float)*4);}

	Quat(const Quat &newq){memcpy(q,newq.q,sizeof(float)*4);}

	~Quat(){};

	float x() const {return q[0];}	
	float y() const {return q[1];}	
	float z() const {return q[2];}
	float w() const {return q[3];}

	void Identity(void){q[0]=0.f;q[1]=0.f;q[2]=0.f;q[3]=1.f;} //Multiplication identity quaternion

	//Normalize the quaternion
	void Normalize(void)
	{
		float magSquared = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
		if (magSquared > (1.f - TOLERANCE) && magSquared < (1.f + TOLERANCE))
			return;
		float mag = sqrt(magSquared);
		q[0] /= mag;q[1] /= mag;q[2] /= mag;q[3] /= mag;
	}


	void FromAxisAngle(float x, float y, float z, float angle)
	{
		float newAngle = angle / 2.f;
		float normVal = (float)sqrt((x*x)+(y*y)+(z*z));
		x *= 1.f/normVal;
		y *= 1.f/normVal;
		z *= 1.f/normVal;
		q[0] = (x * sin(newAngle));
		q[1] = (y * sin(newAngle));
		q[2] = (z * sin(newAngle));
		q[3] = cos(newAngle);
		Normalize();
	}

#ifdef USE_STL_CONTAINER
	//If you want a 4x4 matrix represented as a vector, for some reason...
	void ToRotationMatrix(vector <float> *mat)
	{
		float x2 = q[0] * q[0];
		float y2 = q[1] * q[1];
		float z2 = q[2] * q[2];
		float xy = q[0] * q[1];
		float xz = q[0] * q[2];
		float yz = q[1] * q[2];

		//First row
		mat->push_back(1-2*(y2+z2));
		mat->push_back(2*(xy-q[3]*q[2]));
		mat->push_back(2*(xz+q[3]*q[1]));
		mat->push_back(0);
		//Second row 
		mat->push_back(2*(xy+q[3]*q[2]));
		mat->push_back(1-2*(x2+z2));
		mat->push_back(2*(yz-q[3]*q[0]));
		mat->push_back(0);
		//Third row
		mat->push_back(2*(xz-q[3]*q[1]));
		mat->push_back(2*(yz+q[3]*q[0]));
		mat->push_back(1-2*(x2+y2));
		mat->push_back(0);
		//Fourth row
		mat->push_back(0);
		mat->push_back(0);
		mat->push_back(0);
		mat->push_back(1);
	}
#endif

	//Give the rotation matrix as a 4x4 array of floats
	void ToRotationMatrix(float mat[][4],int arrlength, int arrwidth)
	{
		
		if (arrlength!=arrwidth)
			return; //If the array isn't equal, it's definitely not right
		if (arrlength != 4 || arrwidth != 4)
			return; //Needs to be a 4x4 array
		int totalSize = sizeof(mat) / (sizeof(float)*4);
		if (totalSize != 16)
			return; //Array is not 4x4
		//If a user passes arrlenght = 4 and arrwidth = 4 but doesn't follow that with their arrays, bad things happen
		//At this point said user has brought this upon themselves
		float x2 = q[0] * q[0];
		float y2 = q[1] * q[1];
		float z2 = q[2] * q[2];
		float xy = q[0] * q[1];
		float xz = q[0] * q[2];
		float yz = q[1] * q[2];
		//First row
		mat[0][0] = 1-2*(y2+z2);
		mat[0][1] = 2*(xy-q[3]*q[2]);
		mat[0][2] = 2*(yz-q[3]*q[0]);
		mat[0][3] = 0;
		//Second row
		mat[1][0] = 2*(xy+q[3]*q[2]);
		mat[1][1] = 1-2*(x2+z2);
		mat[1][2] = 2*(yz-q[3]*q[0]);
		mat[1][3] = 0;
		//First row
		mat[0][0] = 2*(xz-q[3]*q[1]);
		mat[0][1] = 2*(yz+q[3]*q[0]);
		mat[0][2] = 1-2*(x2+y2);
		mat[0][3] = 0;
		//First row
		mat[0][0] = 0;
		mat[0][1] = 0;
		mat[0][2] = 0;
		mat[0][3] = 1;
	}

	//Operators
	inline Quat & operator=(const Quat & rhs)
	{
		q[0] = rhs.q[0];
		q[1] = rhs.q[1];
		q[2] = rhs.q[2];
		q[3] = rhs.q[3];
		return *this;
	}

	Quat operator*(const Quat & rhs) const
	{return Quat(rhs.q[3]*q[0] + rhs.q[0]*q[3] + rhs.q[1]*q[2] - rhs.q[2]*q[1],
				 rhs.q[3]*q[1] + rhs.q[1]*q[3] + rhs.q[2]*q[0] - rhs.q[0]*q[2],
				 rhs.q[3]*q[2] + rhs.q[2]*q[3] + rhs.q[0]*q[1] - rhs.q[1]*q[0],
				 rhs.q[3]*q[3] - rhs.q[0]*q[0] - rhs.q[1]*q[1] - rhs.q[2]*q[2]);} 
	Quat & operator*=(const Quat & rhs) 
	{
		*this = rhs * *this;
		return *this;
	}

	bool operator==(const Quat & rhs) const {return (q[0]==rhs.q[0] && q[1]==rhs.q[1] && q[2]==rhs.q[2] && q[3]==rhs.q[3]);}
	bool operator!=(const Quat & rhs) const {return !((*this)==rhs);}

	
};

#endif __QUAT_H__