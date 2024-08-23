#include "MatrixMN.h"
#include <stddef.h>
#include <assert.h>

/**
 * more efficient way is to use malloc/free and data layout included shape, strides, offset
 * and the you can use a continuous memory to represent tensor not just matrix
 **/

MatrixMN::MatrixMN() : M(0), N(0), rows(nullptr) {};

MatrixMN::MatrixMN(int M, int N) : M(M), N(N)
{
  rows = new VectorN[M];
  for (size_t i = 0; i < M; i++)
  {
    rows[i] = VectorN(N);
  }
};

MatrixMN::MatrixMN(const MatrixMN &other)
{
  M = other.M;
  N = other.N;
  rows = new VectorN[M];
  for (size_t i = 0; i < M; i++)
  {
    rows[i] = other.rows[i];
  }
};

MatrixMN::~MatrixMN()
{
  delete[] rows;
};

void MatrixMN::operator=(const MatrixMN &other)
{
  if (rows != nullptr)
    delete[] rows;
  M = other.M;
  N = other.N;
  rows = new VectorN[M];
  for (size_t i = 0; i < M; i++)
  {
    rows[i] = other.rows[i];
  }
};

MatrixMN MatrixMN::Transpose() const
{
  MatrixMN result = MatrixMN(N, M);
  for (size_t i = 0; i < N; i++)
  {
    for (size_t j = 0; j < M; j++)
    {
      result.rows[i][j] = rows[j][i];
    }
  }
  return result;
};

VectorN MatrixMN::operator*(const VectorN &v) const
{
  // MxN Nx1
  assert(N == v.N);
  VectorN result(M);
  for (size_t i = 0; i < M; i++)
  {
    result[i] = rows[i].dot(v);
  }
  return result;
};

MatrixMN MatrixMN::operator*(const MatrixMN &m) const
{
  // MxN NxP
  assert(N == m.M);
  float P = m.N;
  MatrixMN result(M, P);
  MatrixMN mTransposed = m.Transpose(); // PxN
  for (size_t i = 0; i < M; i++)
  {
    for (size_t j = 0; j < P; j++)
    {
      result.rows[i][j] = rows[i].dot(mTransposed.rows[j]);
    }
  }
  return result;
};

VectorN MatrixMN::SolveGaussSeidel(const MatrixMN &A, const VectorN &b, int iterations)
{
  // MxN(A) Nx1(x) = Mx1(b)
  assert(A.M == b.N && A.M <= A.N);
  int N = A.N;
  VectorN X(N);
  if (iterations <= 0)
    iterations = N;
  // iterate times
  for (size_t it = 0; it < iterations; it++)
  {
    for (size_t i = 0; i < N; i++)
    {
      float dx = (b[i] - A.rows[i].dot(X)) / A.rows[i][i];
      if (dx == dx) // 排除 nan
      {
        X[i] += dx;
      }
    }
  }

  return X;
};

VectorN &MatrixMN::operator[](int index)
{
  return rows[index];
};