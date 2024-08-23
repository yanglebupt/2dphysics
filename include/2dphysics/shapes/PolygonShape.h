#ifndef POLYGONSHAPE_H
#define POLYGONSHAPE_H

#include "Shape.h"
#include "Vector2.h"
#include <vector>

struct PolygonShape : Shape
{
  // local vertices is based on origin 0,0，and the order of vertices should be anticlockwise
  std::vector<Vector2> localVertices;
  /**
   * world vertices is based on particle position and body transform (rotation)
   * in 3D, we do this in vertex shader and it do not return the world position
   * but In physics simulation (rigid body) and particle simulation (soft body) we need to get
   * the world position for each vertices.
   * so we will calculate the world position in each frame
   */
  std::vector<Vector2> worldVertices;

  PolygonShape();
  PolygonShape(const std::vector<Vector2> vertices);
  virtual ~PolygonShape();
  virtual float GetMomentOfInertia() const;
  virtual ShapeType GetType() const;
  virtual Shape *New() const;
  // update the world position after physics simulation
  virtual void UpdateVertices(float angle, const Vector2 &position);

  /**
   * 找到最接近的渗透，包括点/法线（和倾入边垂直，三维空间是倾入面）
   *
   * Method:
   * 对自身 (this) 每条边的法线，计算 other 每个顶点到法线的最小投影长度（代表侵入长度<0）
   * 然后对所有的最小侵入长度取最大，此时如果存在一个最小侵入长度大于零，说明 other 的全部顶点在和法向指向的一侧
   * 而法向指向为多边形外，说明 other 的全部顶点在多边形外，则代表没有碰撞
   * 否则，全部侵入长度都小于0，则取最大的，说明是靠着这条边侵入的，（侵入的绝对值最小），也就给出了侵入法线
   *
   * @param other the other polygon shape
   * @param normal 和倾入边垂直的侵入法线
   * @param point other 上侵入到自身 (this) 内部的点
   * @return {float} 侵入长度，大于0，则代表没有碰撞，小于等于0，则代表碰撞
   **/
  float FindClosestPenetration(const PolygonShape &other, Vector2 &normal, Vector2 &point) const;

  /**
   * 返回顶点对应的边向量
   * @param index 顶点索引
   * @return {Vector2} 边向量
   */
  Vector2 GetEdge(int index) const;

  /**
   * 返回顶点对应的边的法向量
   * @param index 顶点索引
   * @return {Vector2} 边对应的法向量
   */
  Vector2 GetEdgeNormal(int index) const;
};

#endif