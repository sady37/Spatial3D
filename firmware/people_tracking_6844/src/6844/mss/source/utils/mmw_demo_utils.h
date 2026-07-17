/*
 * Copyright (C) 2022-24 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MMW_DEMO_UTILS_H
#define MMW_DEMO_UTILS_H

#define PI 3.14159265358979323846f
#define RAD2DEG (180.f/PI)

/*The function reads the FRAME_REF_TIMER that runs free at 40MHz*/
uint32_t Cycleprofiler_getTimeStamp(void);

/* Boundary box structure
 *  The structure defines the box element used to describe the scenery
 */
typedef union
{
    float a[6];
    struct
    {
        float x1;    /* Left boundary, m */
        float x2;    /* Right boundary, m */
        float y1;    /* Near boundary, m */
        float y2;    /* Far boundary, m */
        float z1;    /* Bottom boundary, m */
        float z2;    /* Top boundary, m */
    };
} mmwDemo_boundaryBox;

/* The structure defines a position in cartesian coordinates */
typedef union
{
    float a[3];
    struct
	{
        float posX;     /* X dimension (left-right), m */
        float posY;     /* Y dimension (near-far), m */
        float posZ;     /* Z dimension (height), m */
    };
} mmwDemo_cartesian_position;


/* Sensor position structure
 *  Application can configure algorithm with sensor position. Position is in cartesian space relative to the [3-dimentional] world.
 */
typedef union {
    float a[3];
    struct
	{
        float x;     /* X dimension (left-right), m */
        float y;     /* Y dimension (near-far), m */
        float z;     /* Z dimension (height), m */
    };
} mmwDemo_sensorPosition;

/* Sensor orientation structure
 *  Application can configure algorithm with sensor orientation. Orientation is defined as boresight angular tilts.
 */
typedef union {
    float a[3];
    struct
	{
        float xTilt;  /* Tilt around X axis (i.e., elevation tilt), in radians */
        float yTilt;  /* Tilt around Y axis (i.e., yaw tilt), in radians */
        float zTilt;  /* Tilt around Z axis (i.e., azimuth tilt), in radians */
    };
} mmwDemo_sensorOrientation; 

typedef union {
	float a[9];
	struct
	{
		float e11; float e12; float e13;
		float e21; float e22; float e23;
		float e31; float e32; float e33;
	};
} mmwDemo_MATRIX3x3; 

/* The structure defines the transformation from sensor space to world coordinates in 3D configurations */
typedef struct {
    mmwDemo_sensorOrientation tilt;      /* structure to hold tilting angle parameters */
    mmwDemo_MATRIX3x3 rotW;              /* structure to hold overall rotation matrix */
    mmwDemo_sensorPosition offset;   /* structure to hold sensor offset parameters */
} mmwDemo_worldTransformParams;

void MmwDemo_world2sensor(mmwDemo_cartesian_position *c_in, mmwDemo_worldTransformParams *wt, mmwDemo_cartesian_position *c_out);
void MmwDemo_computeInvRotationMatrix(const mmwDemo_sensorOrientation *tilt, float *rotW);
float MmwDemo_distanceToCuboid(mmwDemo_cartesian_position *A, mmwDemo_boundaryBox *box);
float MmwDemo_farthestDistanceToCuboid(mmwDemo_cartesian_position *A, mmwDemo_boundaryBox *box);

#endif
