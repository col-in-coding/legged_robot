
#include <stdio.h>
#include <stdarg.h> 
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "easyMat.h"

//-----------------------------------------------------------function
/*
创建矩阵
*/
void easyMat_create(matTypeDef* mat, uint16_t row, uint16_t col)
{
  mat->row = row;
  mat->col = col;
  mat->data = (double **)malloc(row*sizeof(double *));    //指针的指针
  for (int i = 0; i < row; i++)
  {
    mat->data[i] = (double *)malloc(col*sizeof(double));  //每行数据的指针
  }
}
/*
释放矩阵内存
*/
void easyMat_free(matTypeDef* mat)
{
  for (uint16_t i = 0; i < mat->row; i++)
  {
    free(mat->data[i]);
  }
  free(mat->data);
}
/*
矩阵赋值
将矩阵from_mat中的值赋给to_mat
*/
void easyMat_assign(matTypeDef* to_mat, matTypeDef* from_mat)
{
  if (to_mat->row < from_mat->row || to_mat->col < from_mat->col) {
    // 输入错误
    return;
  }

  for (uint16_t i = 0; i < from_mat->row; i++)
  {
    for (uint16_t j = 0; j < from_mat->col; j++)
    {
      to_mat->data[i][j] = from_mat->data[i][j];
    }
  }
}
/*
所有元素清零
*/
void easyMat_clear(matTypeDef* mat)
{
  for (uint16_t i = 0; i < mat->row; i++)
  {
    for (uint16_t j = 0; j < mat->col; j++)
    {
      mat->data[i][j] = 0;
    }
  }
}
/*
将方阵初始化为单位阵
*/
void easyMat_eye(matTypeDef* mat)
{
  for (uint16_t i = 0; i < mat->row; i++)
  {
    for (uint16_t j = 0; j < mat->col; j++)
    {
      if(i == j)    mat->data[i][j] = 1;
      else          mat->data[i][j] = 0;
    }
  }
}
/*
矩阵加法
outMat = mat1 + mat2
*/
void easyMat_add(matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)
{
  for (uint16_t i = 0; i < outMat->row; i++)
  {
    for (uint16_t j = 0; j < outMat->col; j++)
    {
      outMat->data[i][j] = mat1->data[i][j] + mat2->data[i][j];
    }
  }
}
/*
矩阵减法
outMat = mat1 - mat2
*/
void easyMat_sub(matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)
{
  for (uint16_t i = 0; i < outMat->row; i++)
  {
    for (uint16_t j = 0; j < outMat->col; j++)
    {
      outMat->data[i][j] = mat1->data[i][j] - mat2->data[i][j];
    }
  }
}
/*
矩阵乘法
outMat = mat1 * mat2
*/
void easyMat_mult(matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)
{
  for (uint16_t i = 0; i < mat1->row; i++)
  {
    for (uint16_t j = 0; j < mat2->col; j++)
    {
      outMat->data[i][j] = 0;
      for (uint16_t m = 0; m < mat1->col; m++)
      {
        outMat->data[i][j] += mat1->data[i][m] * mat2->data[m][j];
      }
    }
  }
}
/*
矩阵转置
dstMat = srcMat'
*/
void easyMat_trans(matTypeDef* dstMat, matTypeDef* srcMat)
{
  for (uint16_t i = 0; i < dstMat->row; i++)
  {
    for (uint16_t j = 0; j < dstMat->col; j++)
    {
      dstMat->data[i][j] = srcMat->data[j][i];
    }
  }
}
/*
矩阵复制
dstMat = srcMat
*/
void easyMat_copy(matTypeDef* dstMat, matTypeDef* srcMat)
{
  for (uint16_t i = 0; i < dstMat->row; i++)
  {
    for (uint16_t j = 0; j < dstMat->col; j++)
    {
      dstMat->data[i][j] = srcMat->data[i][j];
    }
  }
}
/*
绕X轴旋转的3*3旋转矩阵方阵
Mat(0:2, 0:2) = Rot(X, angle)
angle: 采用角度制
*/
void easyMat_rotX(matTypeDef *Mat, double angle)
{
  if (Mat->row < 3 || Mat->col < 3)
    // Error Input
    return;

  //转成弧度制
  double a = angle * 3.141592654f / 180.0f;
  Mat->data[1][1] = cos(a);
  Mat->data[1][2] = -sin(a);
  Mat->data[2][1] = sin(a);
  Mat->data[2][2] = cos(a);
}
/*
绕Y轴旋转的3*3旋转矩阵方阵
Mat(0:2, 0:2) = Rot(Y, angle)
angle: 采用角度制
*/
void easyMat_rotY(matTypeDef *Mat, double angle)
{
  if (Mat->row < 3 || Mat->col < 3)
    // Error Input
    return;

  //转成弧度制
  double a = angle * 3.141592654f / 180.0f;  
  Mat->data[0][0] = cos(a);
  Mat->data[0][2] = sin(a);
  Mat->data[2][0] = -sin(a);
  Mat->data[2][2] = cos(a);
}
/*
绕Z轴旋转的3*3旋转矩阵方阵
Mat(0:2, 0:2) = Rot(Z, angle)
angle: 采用角度制
*/
void easyMat_rotZ(matTypeDef *Mat, double angle)
{
  if (Mat->row < 3 || Mat->col < 3)
    // Error Input
    return;

  //转成弧度制
  double a = angle * 3.141592654f / 180.0f;
  Mat->data[0][0] = cos(a);
  Mat->data[0][1] = -sin(a);
  Mat->data[1][0] = sin(a);
  Mat->data[1][1] = cos(a);
}
/*
往向量（px, py, pz）方向平移
*/
void easyMat_translate(matTypeDef *T, double p[])
{
  if (T->row < 4 || T->col < 4)
    // Error Input
    return;
  T->data[0][3] = p[0];
  T->data[1][3] = p[1];
  T->data[2][3] = p[2];
}

/*
求复合旋转矩阵
outRot=RotY(yaw)*RotZ(pitch)*RotX(roll);
*/
void easyMat_RPY(matTypeDef* outRot, double roll, double pitch, double yaw)
{
  matTypeDef RotX, RotY, RotZ, temp;
  
  easyMat_create(&RotX, 3, 3);
  easyMat_create(&RotY, 3, 3);
  easyMat_create(&RotZ, 3, 3);
  easyMat_create(&temp, 3, 3);

  easyMat_eye(&RotX);
  easyMat_eye(&RotY);
  easyMat_eye(&RotZ);
  easyMat_rotX(&RotX, roll );
  easyMat_rotY(&RotY, yaw  );
  easyMat_rotZ(&RotZ, pitch);
  
  easyMat_mult(&temp , &RotZ, &RotX);
  easyMat_mult(outRot, &RotY, &temp);
  
  easyMat_free(&RotX);
  easyMat_free(&RotY);
  easyMat_free(&RotZ);
  easyMat_free(&temp);
}
/*
矩阵求逆
dstMat = srcMat^(-1)
*/
void easyMat_inv(matTypeDef* dstMat, matTypeDef* srcMat)
{
  //code
}

