/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <math.h>

/* mmwave SDK files */
#include <source/utils/mmw_demo_utils.h>

/*The function reads the FRAME_REF_TIMER that runs free at 40MHz*/
uint32_t Cycleprofiler_getTimeStamp(void)
{
    volatile uint32_t *frameRefTimer;
    frameRefTimer = (volatile uint32_t *) 0x5B000020;
    return *frameRefTimer;
}

/* Function Definitions */
/*
 This function is used to multiply two matrices.
 Matrices are all real, single precision floating point.
 Matrices are in row-major order
 * Arguments    : uint16_t rows, Outer dimension, number of rows
                  uint16_t m, Inner dimension
                  uint16_t cols, Outer dimension, number of cols
                  float *A
                  float *B
                  float *C, C(rows,cols) = A(rows,m) X B(cols,m)T
 * Return Type  : void
 */
void MmwDemo_matrixMultiply(uint16_t rows, uint16_t m, uint16_t cols, float *A, float *B, float *C)
{
	/* C(rows*cols) = A(rows*m)*B(m*cols) */
	uint16_t i,j, k;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols; j++)
		{			
			C[i*cols + j] = 0;
			for (k = 0; k < m; k++)
			{
				C[i*cols+j] += A[i*m+k] * B[k*cols + j];
			}
		}
	}
}

/* Function Definitions */
/* This function is used to transorm cartesian coordinates from sensor-centric to world space
 *
 * Arguments    : IDETECT_cartesian_position *c_in, Pointer to cartesian coordinate before transformation
                  IDETECT_worldTransformParams *wt, Parameters for transformation to world coordinate
                  IDETECT_cartesian_position *c_out, Pointer to cartesian coordinate after transformation
 * Return Type  : void
 */
void MmwDemo_world2sensor(mmwDemo_cartesian_position *c_in, mmwDemo_worldTransformParams *wt, mmwDemo_cartesian_position *c_out)
{
 
  /* Offset */
  c_in->posX -= wt->offset.x;
  c_in->posY -= wt->offset.y;
  c_in->posZ -= wt->offset.z;

  /* Rotation */
  MmwDemo_matrixMultiply(3U, 3U, 1U, wt->rotW.a, c_in->a, c_out->a);

}

void MmwDemo_computeInvRotationMatrix(const mmwDemo_sensorOrientation *tilt, float *rotW)
{
  /* Used the same rotations: https://www.mathworks.com/help/phased/ref/rotx.html */
  float sinRotx, cosRotx;
  float sinRoty, cosRoty;
  float sinRotz, cosRotz;

  sinRotx = sinf(-1*tilt->xTilt);
  cosRotx = cosf(-1*tilt->xTilt);
  sinRoty = sinf(-1*tilt->yTilt);
  cosRoty = cosf(-1*tilt->yTilt);
  sinRotz = sinf(-1*tilt->zTilt);
  cosRotz = cosf(-1*tilt->zTilt);

  /* rotX_W = [1 0 0; 0 cos(rotx_tw) -sin(rotx_tw); 0 sin(rotx_tw) cos(rotx_tw)] */
  float rotX_W[9] = {
    1.f,  0.f,      0.f,
    0.f,  cosRotx,  -1*sinRotx,
    0.f,  sinRotx,  cosRotx
  };

  /* rotY_W = [cos(roty_tw) 0 sin(roty_tw); 0 1 0; -sin(roty_tw) 0 cos(roty_tw)] */
  float rotY_W[9] = {
    cosRoty,    0.f,  sinRoty,
    0.f,        1.f,  0.f,
    -1*sinRoty, 0.f,  cosRoty
  };

  /* rotZ_W = [cos(rotz_tw) -sin(rotz_tw) 0; sin(rotz_tw) cos(rotz_tw) 0; 0 0 1] */
  float rotZ_W[9] = {
    cosRotz,  -1*sinRotz, 0.f,
    sinRotz,  cosRotz,    0.f,
    0.f,      0.f,        1.f
  };

  /* rotW  = rotZ_W * rotY_W * rotX_W */
  float rotXY_W[9];
  MmwDemo_matrixMultiply(3U, 3U, 3U, rotX_W, rotY_W, rotXY_W);
  MmwDemo_matrixMultiply(3U, 3U, 3U, rotXY_W, rotZ_W, rotW);
}
