/*
 * @Author: LHospitalLKY 1141165506@qq.com
 * @Date: 2024-10-03 10:00:43
 * @LastEditors: LHospitalLKY 1141165506@qq.com
 * @LastEditTime: 2024-10-05 13:08:53
 * @FilePath: /ray_tracing_asteroid_simulator/include/BVH.h
 * @Description: BVH射线求交算法
 */


#ifndef BVH_H

#include <cstddef>
#include <fstream>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "common.h"

#define DOUBLE_MAX std::numeric_limits<double>::max()
#define DOUBLE_MIN std::numeric_limits<double>::min()

// 交点
struct Intersect {
  VertexPosition p;
  double t;
  bool hitted;

  Intersect();
};

// 包围盒
class AABB {
public:
  AABB();
  /**
   * @description: 输入两个点来AABB构造函数
   * @param {VertexPosition} &p0
   * @param {VertexPosition} &p1
   * @return {*}
   */
  AABB(const VertexPosition &p0, const VertexPosition &p1);
  /**
   * @description: 复制构造函数
   * @param {AABB} &aabb
   * @return {*}
   */
  AABB(const AABB &aabb);

public:
  /**
   * @description: 向AABB中加入一个点
   * @param {VertexPosition} &p 待加入的点
   * @return {*}
   */
  void grow(const VertexPosition &p);

  /**
   * @description: 融合两个AABB包围盒
   * @param {AABB} &aabb, 待融合的AABB盒
   * @return {*}
   */
  void grow(const AABB &aabb);

  /**
   * @description: 求两个AABB包围盒的交集
   * @param {AABB} &aabb, 待求的AABB盒
   * @return {*}
   */
  void intersect(const AABB &aabb);

  /**
   * @description: 计算aabb的面积
   * @return {*}
   */
  double area() const;

  /**
   * @description: 判断当前AABB是否有效
   * @return {bool} true:有效 false:无效
   */
  bool vaild_aabb() const;

  /**
   * @description: 计算AABB盒的中点
   * @return {VertexPosition} 中心点
   */
  VertexPosition centroid() const;

  /**
   * @description: 返回AABB最长的方向
   * @return {*} 0: x轴最长, 1: y轴最长, 2: z轴最长
   */
  int maxExtent() const;

  /**
   * @description: 计算一条射线与AABB的交点
   * @param {VertexPosition} &origin 射线起点
   * @param {ViewVector} &direction 射线方向
   * @param {array<int, 3>} &dirIsNeg TODO: 还不确定是什么呢
   * @return {*}
   */
  bool intersectP(const VertexPosition &origin, const ViewVector &direction,
                  const std::array<int, 3> &dirIsNeg) const;

  // 返回min点与max点
  VertexPosition getMin() const;
  VertexPosition getMax() const;

private:
  VertexPosition m_mn; // AABB包围盒的min坐标
  VertexPosition m_mx; // AABB包围盒的max坐标
};

class Triangle {
public:
  /**
   * @description: 创建三角面片
   * @param {VertexPosition} &v0, 顶点0
   * @param {VertexPosition} &v1, 顶点1
   * @param {VertexPosition} &v2, 顶点2
   * @param {ViewVector} &normal, 法向量
   * @return {*}
   */
  Triangle(const VertexPosition &v0, const VertexPosition &v1,
           const VertexPosition &v2, const ViewVector &normal);

public:
  /**
   * @description: 返回顶点列表
   * @return {VerticeList}
   */
  VerticeList getVertex();

  /**
   * @description: 计算射线与三角面的交点
   * @param {VertexPosition} &origin, 射线起点
   * @param {ViewVector} &direction, 射线方向
   * @return {Intersect}
   */
  Intersect intersect(const VertexPosition &origin,
                      const ViewVector &direction) const;

  /**
   * @description: 返回当前三角面片的AABB包围盒
   * @return {AABB}
   */
  AABB getAABB() const;

private:
  // 三角形三个顶点
  const VertexPosition &_p0;
  const VertexPosition &_p1;
  const VertexPosition &_p2;
  // 法向量
  const ViewVector &_normal;
};

class BVHBuildNode {
public:
  BVHBuildNode();

public:
  AABB m_bounds;
  BVHBuildNode *m_left;
  BVHBuildNode *m_right;
  Triangle *m_tri_list;

  int splitAxis = 0; // 切分轴
  int firstPrimOffset = 0;
  int nPrimitives = 0;
};

/**
 * @description: 递归地创建BVH树
 * @param {vector<Triangle*>} triangles列表
 * @return {BVHBuildNode*} BVH树根节点
 */
BVHBuildNode *recursiveBuild(std::vector<Triangle *> triangles);

/**
 * @description: 计算射线与BVH树的交点
 * @param {BVHBuildNode*} node
 * @param {VertexPosition} &origin
 * @param {ViewVector} &direction
 * @param {double} &t
 * @param {VertexPosition} &intersection
 * @return {*}
 */
Intersect getIntersect(BVHBuildNode *node, const VertexPosition &origin,
                       const ViewVector &direction);

/**
 * @description: 计算两个AABB的并
 * @param {AABB} &box_1, aabb_1
 * @param {AABB} &box_2, aabb_2
 * @return {*}
 */
AABB surroundingBox(const AABB &box_1, const AABB &box_2);

// 可视化函数
void showAABB(const AABB &box);
void showAABBwithShape(const VerticeList &verticeList, const AABB &box);
void showAABBwithShape(const std::vector<Triangle *> tri_list, const AABB &box);
// 保存BVH树
void saveBVH(BVHBuildNode *node, std::ofstream &f_vertices,
             std::ofstream &aabb_vertices);
// 可视化BVH树
void showBVHAABB(const std::string &aabb_path);

void wait_for_key();

#endif // !BVH_H
