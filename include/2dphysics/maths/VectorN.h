#ifndef VECTORN_H
#define VECTORN_H

/**
 * VectorN 中存放了一段堆中的数据内存，注意要释放内存
 * 特别是做一些运算时，可能容易忘记释放中间变量，容易造成内存泄漏，例如
 *
 * VectorN n = VectorN(10);
 * VectorN t = 2*n + 1;  // 计算出了 t，但是中间变量 2*n 你没有释放，中间变量并不需要了，除非你要保留整个计算图
 *
 * 解决办法：
 *    API内部不要使用 new 来返回一个新对象，那么中间变量就是栈上的局部变量，不再使用后，会自动调用析构函数释放其中的堆内存
 *    API 接口改成 Parameter out 的形式，内部不会创建新的，由外部统一创建，函数内部赋值，这样可以共用一些中间运算变量
 */
struct VectorN
{
  int N;
  float *data;

  VectorN();
  VectorN(int N);
  // 拷贝构造函数也承担了初始化
  VectorN(const VectorN &other);
  ~VectorN();

  float dot(const VectorN &v) const;

  float operator[](int index) const;
  float &operator[](int index);
  // 必须显式对一个已经初始化的进行 = 赋值才会调用，否则默认都是走拷贝构造函数，例如函数返回值，函数值参数等
  void operator=(const VectorN &other);

  // 友元并不是类成员
  friend VectorN operator/(float scalar, const VectorN &v);
  friend VectorN operator*(float scalar, const VectorN &v);
  friend VectorN operator+(float scalar, const VectorN &v);
  friend VectorN operator-(float scalar, const VectorN &v);

  VectorN operator+(const VectorN &v) const;
  VectorN operator+(float v) const;
  VectorN operator-() const;
  VectorN operator-(const VectorN &v) const;
  VectorN operator-(float v) const;
  VectorN operator*(const VectorN &v) const;
  VectorN operator*(float v) const;
  VectorN operator/(const VectorN &v) const;
  VectorN operator/(float v) const;

  void operator+=(const VectorN &v);
  void operator+=(float v);
  void operator-=(const VectorN &v);
  void operator-=(float v);
  void operator*=(const VectorN &v);
  void operator*=(float v);
  void operator/=(const VectorN &v);
  void operator/=(float v);
};

#endif