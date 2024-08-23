#include "VectorN.h"
#include <stddef.h>
#include <assert.h>

VectorN::VectorN() : N(0), data(nullptr) {}
VectorN::VectorN(int N) : N(N)
{
  data = new float[N];
  for (size_t i = 0; i < N; i++)
  {
    data[i] = 0.f;
  }
};

VectorN::VectorN(const VectorN &other)
{
  N = other.N;
  data = new float[N];
  for (size_t i = 0; i < N; i++)
  {
    data[i] = other[i];
  }
};

VectorN::~VectorN()
{
  delete[] data;
};

float VectorN::dot(const VectorN &v) const
{
  assert(v.N == N);
  float sum = 0.f;
  for (size_t i = 0; i < N; i++)
  {
    sum += data[i] * v[i];
  }
  return sum;
};

float &VectorN::operator[](int index)
{
  assert(index < N);
  return data[index];
};

float VectorN::operator[](int index) const
{
  assert(index < N);
  return data[index];
};

void VectorN::operator=(const VectorN &other)
{
  if (data != nullptr)
    delete[] data;
  N = other.N;
  data = new float[N];
  for (size_t i = 0; i < N; i++)
  {
    data[i] = other[i];
  }
};

VectorN operator/(float scalar, const VectorN &v)
{
  VectorN result = VectorN(v.N);
  for (size_t i = 0, n = v.N; i < n; i++)
  {
    result[i] = scalar / v[i];
  }
  return result;
};
VectorN operator*(float scalar, const VectorN &v)
{
  VectorN result = VectorN(v.N);
  for (size_t i = 0, n = v.N; i < n; i++)
  {
    result[i] = scalar * v[i];
  }
  return result;
};
VectorN operator+(float scalar, const VectorN &v)
{
  VectorN result = VectorN(v.N);
  for (size_t i = 0, n = v.N; i < n; i++)
  {
    result[i] = scalar + v[i];
  }
  return result;
};
VectorN operator-(float scalar, const VectorN &v)
{
  VectorN result = VectorN(v.N);
  for (size_t i = 0, n = v.N; i < n; i++)
  {
    result[i] = scalar - v[i];
  }
  return result;
};

VectorN VectorN::operator+(const VectorN &v) const
{
  VectorN result = *this;
  result += v;
  return result;
};
VectorN VectorN::operator+(float v) const
{
  VectorN result = *this;
  result += v;
  return result;
};
VectorN VectorN::operator-() const
{
  VectorN result = *this;
  for (size_t i = 0; i < N; i++)
  {
    result[i] = -result[i];
  }
  return result;
};
VectorN VectorN::operator-(const VectorN &v) const
{
  VectorN result = *this;
  result -= v;
  return result;
};
VectorN VectorN::operator-(float v) const
{
  VectorN result = *this;
  result -= v;
  return result;
};
VectorN VectorN::operator*(const VectorN &v) const
{
  VectorN result = *this;
  result *= v;
  return result;
};
VectorN VectorN::operator*(float v) const
{
  VectorN result = *this;
  result *= v;
  return result;
};
VectorN VectorN::operator/(const VectorN &v) const
{
  VectorN result = *this;
  result /= v;
  return result;
};
VectorN VectorN::operator/(float v) const
{
  VectorN result = *this;
  result /= v;
  return result;
};

void VectorN::operator+=(const VectorN &v)
{
  assert(N == v.N);
  for (size_t i = 0; i < N; i++)
  {
    data[i] += v[i];
  }
};
void VectorN::operator+=(float v)
{
  for (size_t i = 0; i < N; i++)
  {
    data[i] += v;
  }
};
void VectorN::operator-=(const VectorN &v)
{
  assert(N == v.N);
  for (size_t i = 0; i < N; i++)
  {
    data[i] -= v[i];
  }
};
void VectorN::operator-=(float v)
{
  for (size_t i = 0; i < N; i++)
  {
    data[i] -= v;
  }
};
void VectorN::operator*=(const VectorN &v)
{
  assert(N == v.N);
  for (size_t i = 0; i < N; i++)
  {
    data[i] *= v[i];
  }
};
void VectorN::operator*=(float v)
{
  for (size_t i = 0; i < N; i++)
  {
    data[i] *= v;
  }
};
void VectorN::operator/=(const VectorN &v)
{
  assert(N == v.N);
  for (size_t i = 0; i < N; i++)
  {
    data[i] /= v[i];
  }
};
void VectorN::operator/=(float v)
{
  for (size_t i = 0; i < N; i++)
  {
    data[i] /= v;
  }
};