#ifndef MATRIXMN_H
#define MATRIXMN_H

#include "VectorN.h"
// we will use two dimension array represent matrix
struct MatrixMN
{
  int M;         // M rows
  int N;         // N columns
  VectorN *rows; // each row has N elements

  MatrixMN();
  MatrixMN(int M, int N);
  MatrixMN(const MatrixMN &other);
  ~MatrixMN();

  // m = m1
  void operator=(const MatrixMN &other);

  VectorN &operator[](int index);

  MatrixMN Transpose() const;

  // m*v and v is column vector
  VectorN operator*(const VectorN &v) const;

  // m*m1
  MatrixMN operator*(const MatrixMN &m) const;

  // solve linear equation "Ax=b" return x
  static VectorN SolveGaussSeidel(const MatrixMN &A, const VectorN &b, int iterations = -1);
};

#endif