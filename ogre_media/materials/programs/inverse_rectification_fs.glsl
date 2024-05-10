#version 120

// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

uniform int undistortionIterations;
uniform float cameraMatrixVec[9];
uniform float distCoeffs[14];
uniform float rectificationRotationVec[9];
uniform float newCameraMatrixVec[9];
uniform vec2 size;

// The distort method implementation is largely based on the OpenCV cvUndistortPointsInternal
// https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/undistort.dispatch.cpp#L385
// and related functions like initInverseRectificationMap.
// Here's the copyright notice of the source files that served as inspiration:
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

mat3 vecToMat(float[9] v)
{
  return mat3(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
}

mat3 matmul(mat3 N, mat3 M)
{
  mat3 res;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      res[i][j] = M[i][0] * N[0][j] + M[i][1] * N[1][j] + M[i][2] * N[2][j];
  return res;
}

vec3 matvecmul(mat3 N, vec3 v)
{
  return vec3(
    v[0] * N[0][0] + v[1] * N[1][0] + v[2] * N[2][0],
    v[0] * N[0][1] + v[1] * N[1][1] + v[2] * N[2][1],
    v[0] * N[0][2] + v[1] * N[1][2] + v[2] * N[2][2]
  );
}

mat3 computeTiltProjectionMatrix(float tauX, float tauY)
{
  float cTauX = cos(tauX);
  float sTauX = sin(tauX);
  float cTauY = cos(tauY);
  float sTauY = sin(tauY);
  mat3 matRotX = transpose(mat3(1.0, 0.0, 0.0, 0.0, cTauX, sTauX, 0.0, -sTauX, cTauX));
  mat3 matRotY = transpose(mat3(cTauY, 0.0, -sTauY, 0.0, 1.0, 0.0, sTauY, 0.0, cTauY));
  mat3 matRotXY = matmul(matRotY, matRotX);
  float inv = 1. / matRotXY[2][2];
  mat3 invMatProjZ = transpose(mat3(inv, 0.0, inv * matRotXY[2][0], 0, inv, inv * matRotXY[2][1], 0.0, 0.0, 1.0));
  return matmul(transpose(matRotXY), invMatProjZ);
}

vec2 undistortPoints(vec2 pos, mat3 K, float[14] k, mat3 R, mat3 P)
{
  float fx = K[0][0];
  float fy = K[1][1];
  float ifx = 1.0 / fx;
  float ify = 1.0 / fy;
  float cx = K[2][0];
  float cy = K[2][1];

  float x = pos.x;
  float y = pos.y;
  float u = x;
  float v = y;

  x = (x - cx) * ifx;
  y = (y - cy) * ify;

  if (k[0] != 0.0)
  {
    if (k[12] != 0.0 || k[13] != 0.0)
    {
      mat3 invMatTilt = computeTiltProjectionMatrix(k[12], k[13]);
      vec3 vecUntilt = matvecmul(invMatTilt, vec3(x, y, 1.0));
      float invProj = vecUntilt.z != 0.0 ? 1.0 / vecUntilt.z : 1.0;
      x = invProj * vecUntilt.x;
      y = invProj * vecUntilt.y;
    }

    float x0 = x;
    float y0 = y;

    float r2;
    float icdist;
    float deltaX;
    float deltaY;

    for (int j = 0; j < undistortionIterations; ++j)
    {
      r2 = x * x + y * y;
      icdist = (1.0 + ((k[7] * r2 + k[6]) * r2 + k[5]) * r2) / (1.0 + ((k[4] * r2 + k[1]) * r2 + k[0]) * r2);
      if (icdist < 0.0)
      {
        x = (u - cx) * ifx;
        y = (v - cy) * ify;
        break;
      }
      deltaX = 2.0 * k[2] * x * y + k[3] * (r2 + 2.0 * x * x)+ k[8] * r2 + k[9] * r2 * r2;
      deltaY = k[2] * (r2 + 2.0 * y * y) + 2.0 * k[3] * x * y+ k[10] * r2 + k[11] * r2 * r2;
      x = (x0 - deltaX) * icdist;
      y = (y0 - deltaY) * icdist;
    }
  }

  R = matmul(P, R);
  vec3 pos2 = matvecmul(R, vec3(x, y, 1.0));
  x = pos2.x / pos2.z;
  y = pos2.y / pos2.z;

  return vec2(x, y);
}

vec2 perspectiveTransform(vec2 pos, mat3 R)
{
  float x = pos.x;
  float y = pos.y;
  float w = x * R[0][2] + y * R[1][2] + R[2][2];

  if(abs(w) > 1e-9)
  {
    w = 1.0 / w;
    return vec2(
      (x * R[0][0] + y * R[1][0] + R[2][0]) * w,
      (x * R[0][1] + y * R[1][1] + R[2][1]) * w
    );
  }
  else
  {
    return vec2(0.0);
  }
}

vec2 inverseRectify(vec2 pos)
{
  float[distCoeffs.length()] emptyCoeffs;
  for (int i = 0; i < emptyCoeffs.length(); ++i)
    emptyCoeffs[i] = 0.0;

  mat3 cameraMatrix = vecToMat(cameraMatrixVec);
  mat3 rectificationRotation = vecToMat(rectificationRotationVec);
  mat3 newCameraMatrix = vecToMat(newCameraMatrixVec);

  // HACK: Prevent some compilers (AMD) to optimize out some dimensions of cameraMatrixVec array.
  pos *= (cameraMatrix[2][2] / cameraMatrix[2][2]);

  pos *= size;  // convert from relative texture coords to absolute image coords

  // explanation in OpenCV initInverseRectificationMap
  pos = undistortPoints(pos, cameraMatrix, distCoeffs, mat3(1.0), mat3(1.0));
  pos = perspectiveTransform(pos, rectificationRotation);
  pos = undistortPoints(pos, mat3(1.0), emptyCoeffs, mat3(1.0), newCameraMatrix);

  return pos / size;  // convert back to relative texture coords
}

void main()
{
  vec2 uvRect = gl_TexCoord[0].xy;  // We start in a rectified image rendered by OGRE camera.
  vec2 uvRaw = inverseRectify(uvRect);  // And we want to get the raw (distorted) coordinates.

  if (uvRaw.x < 0.0 || uvRaw.y < 0.0)
    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
  else
    gl_FragColor = texture2D(RT, uvRaw);
}

