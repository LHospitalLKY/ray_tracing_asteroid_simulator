/*
 * @Author: LHospitalLKY 1141165506@qq.com
 * @Date: 2024-10-03 10:01:14
 * @LastEditors: LHospitalLKY 1141165506@qq.com
 * @LastEditTime: 2024-10-05 13:09:39
 * @FilePath: /ray_tracing_asteroid_simulator/src/BVH.cpp
 * @Description: BVH.h的实现
 */

#include "../include/BVH.h"
#include "gnuplot_i.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

/*Intersect结构体*/
Intersect::Intersect() {
  p = {0.0, 0.0, 0.0};
  t = std::numeric_limits<double>::max();
  hitted = false;
}

/*AABB类*/
AABB::AABB() {
  m_mn = {DOUBLE_MAX, DOUBLE_MAX, DOUBLE_MAX};
  m_mx = {DOUBLE_MIN, DOUBLE_MIN, DOUBLE_MIN};
}

AABB::AABB(const VertexPosition &p0, const VertexPosition &p1) {
  m_mn = {std::fmin(p0[0], p1[0]), std::fmin(p0[1], p1[1]),
          std::fmin(p0[2], p1[2])};
  m_mx = {std::fmax(p0[0], p1[0]), std::fmax(p0[1], p1[1]),
          std::fmax(p0[2], p1[2])};
}

AABB::AABB(const AABB &aabb) {
  m_mn = aabb.getMin();
  m_mx = aabb.getMax();
}

void AABB::grow(const VertexPosition &p) {
  m_mn = {std::fmin(m_mn[0], p[0]), std::fmin(m_mn[1], p[1]),
          std::fmin(m_mn[2], p[2])};
  m_mx = {std::fmax(m_mx[0], p[0]), std::fmax(m_mx[1], p[1]),
          std::fmax(m_mx[2], p[2])};
}

void AABB::grow(const AABB &aabb) {
  this->grow(aabb.getMin());
  this->grow(aabb.getMax());
}

void AABB::intersect(const AABB &aabb) {
  m_mn = {std::fmax(m_mn[0], (aabb.getMin())[0]),
          std::fmax(m_mn[1], (aabb.getMin())[1]),
          std::fmax(m_mn[2], (aabb.getMin())[2])};
  m_mn = {std::fmin(m_mn[0], (aabb.getMax())[0]),
          std::fmin(m_mn[1], (aabb.getMax())[1]),
          std::fmin(m_mn[2], (aabb.getMax())[2])};
}

double AABB::area() const {
  if (!vaild_aabb()) {
    return 0.0;
  }
  return (m_mx[0] - m_mn[0]) * (m_mx[1] - m_mn[1]) * (m_mx[2] - m_mn[2]);
}

bool AABB::vaild_aabb() const {
  return (m_mn[0] <= m_mx[0]) && (m_mn[1] <= m_mx[1]) && (m_mn[2] <= m_mx[2]);
}
VertexPosition AABB::centroid() const {
  return {(m_mn[0] + m_mx[0]) / 2.0, (m_mn[1] + m_mx[1]) / 2.0,
          (m_mn[2] + m_mx[2]) / 2.0};
}

int AABB::maxExtent() const {
  VertexPosition d = m_mx - m_mn;
  // Vector3f d = Diagonal();
  if (d[0] > d[1] && d[0] > d[2])
    return 0;
  else if (d[1] > d[2])
    return 1;
  else
    return 2;
}

bool AABB::intersectP(const VertexPosition &origin, const ViewVector &direction,
                      const std::array<int, 3> &dirIsNeg) const {
  double tEnter = -std::numeric_limits<double>::infinity();
  double tExit = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < 3; i++) {
    double min = (m_mn[i] - origin[i]) / direction[i];
    double max = (m_mx[i] - origin[i]) / direction[i];
    if (!dirIsNeg[i]) {
      std::swap(min, max);
    }
    tEnter = std::max(min, tEnter);
    tExit = std::min(max, tExit);
  }

  return tEnter < tExit && tExit >= 0.0;
}

VertexPosition AABB::getMin() const { return m_mn; }
VertexPosition AABB::getMax() const { return m_mx; }

AABB surroundingBox(const AABB &box_1, const AABB &box_2) {
  AABB new_box(box_1);
  new_box.grow(box_2);

  return new_box;
}

/*Triangle类*/
Triangle::Triangle(const VertexPosition &v0, const VertexPosition &v1,
                   const VertexPosition &v2, const ViewVector &normal)
    : _p0(v0), _p1(v1), _p2(v2), _normal(normal) {}
VerticeList Triangle::getVertex() {
  VerticeList tri_vertex_list;
  tri_vertex_list.push_back(VertexPosition{_p0[0], _p0[1], _p0[2]});
  tri_vertex_list.push_back(VertexPosition{_p1[0], _p1[1], _p1[2]});
  tri_vertex_list.push_back(VertexPosition{_p2[0], _p2[1], _p2[2]});

  return tri_vertex_list;
}
Intersect Triangle::intersect(const VertexPosition &origin,
                              const ViewVector &direction) const {
  // 得到三角形顶点坐标
  // 三角形边
  ViewVector AB = _p1 - _p0;
  ViewVector AC = _p2 - _p0;

  // 计算距离
  ViewVector h = cross(direction, AC);
  double a = dot(AB, h);

  if (a > 0.0 || a < 0.0) {
    double f = 1.0 / a;
    ViewVector s = origin - _p0;
    double u = f * dot(s, h);

    if (u < 0.0 || u > 1.0) {
      return Intersect();
    }

    ViewVector q = cross(s, AB);
    double v = f * dot(direction, q);

    if (v < 0.0 || u + v > 1.0) {
      return Intersect();
    }

    // 输出o+td的t值与交点坐标
    double t = f * dot(AC, q);
    if (t < 0.) {
      return Intersect();
    }
    Intersect intersection;
    intersection.p = origin + direction * t;
    intersection.t = t;
    intersection.hitted = true;
    // std::cout << intersection[0] << " ";
    // std::cout << intersection[1] << " ";
    // std::cout << intersection[2] << std::endl;
    return intersection;
  }
  return Intersect();
}

AABB Triangle::getAABB() const {
  AABB aabb = AABB(_p0, _p1);
  aabb.grow(_p2);
  return aabb;
}

/*BVHBuildNode类*/
BVHBuildNode::BVHBuildNode() {
  m_bounds = AABB();
  m_left = nullptr;
  m_right = nullptr;
  m_tri_list = nullptr;
}

BVHBuildNode *recursiveBuild(std::vector<Triangle *> triangles) {
  BVHBuildNode *node = new BVHBuildNode();

  // 计算所有的三角形包围盒
  AABB root_box;
  for (size_t i = 0; i < triangles.size(); i++) {
    VerticeList ver_list = triangles[i]->getVertex();
    for (size_t j = 0; j < 3; j++) {
      root_box.grow(ver_list[j]);
    }
  }
  // std::cout << root_box.centroid();

  // 判断当前三角形队列中有多少三角形
  if (triangles.size() == 1) {
    // 三角形队列中仅剩1个三角形, 直接作为叶子节点
    node->m_bounds = triangles[0]->getAABB();
    node->m_tri_list = triangles[0];
    node->m_left = nullptr;
    node->m_right = nullptr;
    return node;
  }
  if (triangles.size() == 2) {
    // 三角形队列中仅剩2个三角形, 作为父节点, 创建两个叶节点
    node->m_left = recursiveBuild(std::vector<Triangle *>{triangles[0]});
    node->m_right = recursiveBuild(std::vector<Triangle *>{triangles[1]});
    AABB left_aabb = node->m_left->m_bounds;
    left_aabb.grow(triangles[1]->getAABB());
    node->m_bounds =
        surroundingBox(node->m_left->m_bounds, node->m_right->m_bounds);
    return node;
  }

  AABB centroidAABB;
  for (size_t i = 0; i < triangles.size(); i++) {
    centroidAABB.grow(triangles[i]->getAABB().centroid());
  }
  int dim = centroidAABB.maxExtent();

  // 按照最长边对三角形集合进行重新排序
  switch (dim) {
  case 0:
    std::sort(
        triangles.begin(), triangles.end(), [](Triangle *f1, Triangle *f2) {
          return (f1->getAABB()).centroid()[0] < (f2->getAABB()).centroid()[0];
        });
    break;
  case 1:
    std::sort(
        triangles.begin(), triangles.end(), [](Triangle *f1, Triangle *f2) {
          return (f1->getAABB()).centroid()[1] < (f2->getAABB()).centroid()[1];
        });
    break;
  case 2:
    std::sort(
        triangles.begin(), triangles.end(), [](Triangle *f1, Triangle *f2) {
          return (f1->getAABB()).centroid()[2] < (f2->getAABB()).centroid()[2];
        });
    break;
  }

  // 分割三角形列表
  std::vector<Triangle *>::iterator beginning = triangles.begin();
  std::vector<Triangle *>::iterator middling =
      triangles.begin() + (triangles.size() / 2);
  std::vector<Triangle *>::iterator endding = triangles.end();

  std::vector<Triangle *> leftShapes =
      std::vector<Triangle *>(beginning, middling);
  std::vector<Triangle *> rightShapes =
      std::vector<Triangle *>(middling, endding);

  assert(triangles.size() == (leftShapes.size() + rightShapes.size()));

  node->m_left = recursiveBuild(leftShapes);
  node->m_right = recursiveBuild(rightShapes);
  node->m_bounds =
      surroundingBox(node->m_left->m_bounds, node->m_right->m_bounds);

  return node;
  // showAABBwithShape(triangles, centroidAABB);
  // Gnuplot g;
  // showAABBwithShape(triangles, root_box);
}

Intersect getIntersect(BVHBuildNode *node, const VertexPosition &origin,
                       const ViewVector &direction) {
  Intersect intersec;

  std::array<int, 3> dirIsNeg;
  dirIsNeg[0] = int(direction[0] >= 0.0);
  dirIsNeg[1] = int(direction[1] >= 0.0);
  dirIsNeg[2] = int(direction[2] >= 0.0);

  //
  if (!node->m_bounds.intersectP(origin, direction, dirIsNeg)) {
    return intersec;
  }

  // 若当前节点是叶节点
  if (node->m_left == nullptr && node->m_right == nullptr) {
    intersec = node->m_tri_list->intersect(origin, direction);
    return intersec;
  }

  // 若当前节点并不是叶节点
  Intersect hit_1 = getIntersect(node->m_left, origin, direction);
  Intersect hit_2 = getIntersect(node->m_right, origin, direction);

  if (hit_1.t < hit_2.t) {
    return hit_1;
  } else {
    return hit_2;
  }
}

void showAABB(const AABB &box) {
  VertexPosition min = box.getMin();
  VertexPosition max = box.getMax();
  VertexPosition p0 = {min[0], min[1], min[2]};
  VertexPosition p1 = {min[0], max[1], min[2]};
  VertexPosition p2 = {max[0], min[1], min[2]};
  VertexPosition p3 = {max[0], max[1], min[2]};
  VertexPosition p4 = {min[0], min[1], max[2]};
  VertexPosition p5 = {min[0], max[1], max[2]};
  VertexPosition p6 = {max[0], min[1], max[2]};
  VertexPosition p7 = {max[0], max[1], max[2]};

  std::vector<double> x = {p0[0], p1[0], p0[0], p2[0], p1[0], p3[0],
                           p2[0], p3[0], p0[0], p4[0], p1[0], p5[0],
                           p2[0], p6[0], p3[0], p7[0], p4[0], p5[0],
                           p4[0], p6[0], p5[0], p7[0], p6[0], p7[0]};
  std::vector<double> y = {p0[1], p1[1], p0[1], p2[1], p1[1], p3[1],
                           p2[1], p3[1], p0[1], p4[1], p1[1], p5[1],
                           p2[1], p6[1], p3[1], p7[1], p4[1], p5[1],
                           p4[1], p6[1], p5[1], p7[1], p6[1], p7[1]};
  std::vector<double> z = {p0[2], p1[2], p0[2], p2[2], p1[2], p3[2],
                           p2[2], p3[2], p0[2], p4[2], p1[2], p5[2],
                           p2[2], p6[2], p3[2], p7[2], p4[2], p5[2],
                           p4[2], p6[2], p5[2], p7[2], p6[2], p7[2]};

  Gnuplot g1("lines");
  g1.reset_all();
  g1.set_style("linespoints").plot_xyz(x, y, z);

  wait_for_key();
}

void showAABBwithShape(const VerticeList &verticeList, const AABB &box) {
  // 创建包围盒的点
  VertexPosition min = box.getMin();
  VertexPosition max = box.getMax();
  VertexPosition p0 = {min[0], min[1], min[2]};
  VertexPosition p1 = {min[0], max[1], min[2]};
  VertexPosition p2 = {max[0], min[1], min[2]};
  VertexPosition p3 = {max[0], max[1], min[2]};
  VertexPosition p4 = {min[0], min[1], max[2]};
  VertexPosition p5 = {min[0], max[1], max[2]};
  VertexPosition p6 = {max[0], min[1], max[2]};
  VertexPosition p7 = {max[0], max[1], max[2]};

  std::vector<double> aabb_x = {p0[0], p1[0], p0[0], p2[0], p1[0], p3[0],
                                p2[0], p3[0], p0[0], p4[0], p1[0], p5[0],
                                p2[0], p6[0], p3[0], p7[0], p4[0], p5[0],
                                p4[0], p6[0], p5[0], p7[0], p6[0], p7[0]};
  std::vector<double> aabb_y = {p0[1], p1[1], p0[1], p2[1], p1[1], p3[1],
                                p2[1], p3[1], p0[1], p4[1], p1[1], p5[1],
                                p2[1], p6[1], p3[1], p7[1], p4[1], p5[1],
                                p4[1], p6[1], p5[1], p7[1], p6[1], p7[1]};
  std::vector<double> aabb_z = {p0[2], p1[2], p0[2], p2[2], p1[2], p3[2],
                                p2[2], p3[2], p0[2], p4[2], p1[2], p5[2],
                                p2[2], p6[2], p3[2], p7[2], p4[2], p5[2],
                                p4[2], p6[2], p5[2], p7[2], p6[2], p7[2]};

  // 创建形状的点
  std::vector<double> shape_x, shape_y, shape_z;
  for (size_t i = 0; i < verticeList.size(); i++) {
    shape_x.push_back(verticeList[i][0]);
    shape_y.push_back(verticeList[i][1]);
    shape_z.push_back(verticeList[i][2]);
  }

  Gnuplot g1("lines");
  g1.reset_all();
  g1.set_style("points").plot_xyz(shape_x, shape_y, shape_z);
  g1.set_style("linespoints").plot_xyz(aabb_x, aabb_y, aabb_z);

  wait_for_key();
}

void showAABBwithShape(const std::vector<Triangle *> tri_list,
                       const AABB &box) {
  // 创建包围盒的点
  VertexPosition min = box.getMin();
  VertexPosition max = box.getMax();
  VertexPosition p0 = {min[0], min[1], min[2]};
  VertexPosition p1 = {min[0], max[1], min[2]};
  VertexPosition p2 = {max[0], min[1], min[2]};
  VertexPosition p3 = {max[0], max[1], min[2]};
  VertexPosition p4 = {min[0], min[1], max[2]};
  VertexPosition p5 = {min[0], max[1], max[2]};
  VertexPosition p6 = {max[0], min[1], max[2]};
  VertexPosition p7 = {max[0], max[1], max[2]};

  std::vector<double> aabb_x = {p0[0], p1[0], p0[0], p2[0], p1[0], p3[0],
                                p2[0], p3[0], p0[0], p4[0], p1[0], p5[0],
                                p2[0], p6[0], p3[0], p7[0], p4[0], p5[0],
                                p4[0], p6[0], p5[0], p7[0], p6[0], p7[0]};
  std::vector<double> aabb_y = {p0[1], p1[1], p0[1], p2[1], p1[1], p3[1],
                                p2[1], p3[1], p0[1], p4[1], p1[1], p5[1],
                                p2[1], p6[1], p3[1], p7[1], p4[1], p5[1],
                                p4[1], p6[1], p5[1], p7[1], p6[1], p7[1]};
  std::vector<double> aabb_z = {p0[2], p1[2], p0[2], p2[2], p1[2], p3[2],
                                p2[2], p3[2], p0[2], p4[2], p1[2], p5[2],
                                p2[2], p6[2], p3[2], p7[2], p4[2], p5[2],
                                p4[2], p6[2], p5[2], p7[2], p6[2], p7[2]};

  // 创建形状的点
  std::vector<double> shape_x, shape_y, shape_z;
  for (size_t i = 0; i < tri_list.size(); i++) {
    VerticeList ver_list = tri_list[i]->getVertex();
    for (size_t j = 0; j < 3; j++) {
      shape_x.push_back(ver_list[j][0]);
      shape_y.push_back(ver_list[j][1]);
      shape_z.push_back(ver_list[j][2]);
    }
  }

  Gnuplot g("lines");
  g.reset_all();
  g.set_style("points").plot_xyz(shape_x, shape_y, shape_z);
  g.set_style("linespoints").plot_xyz(aabb_x, aabb_y, aabb_z);

  wait_for_key();
}

void saveBVH(BVHBuildNode *node, std::ofstream &f_vertices,
             std::ofstream &aabb_vertices) {
  // 包围盒
  AABB box = node->m_bounds;
  VertexPosition min = box.getMin();
  VertexPosition max = box.getMax();
  // aabb_vertices << min[0] << " " << min[1] << " " << min[2] << std::endl;
  // aabb_vertices << max[0] << " " << max[1] << " " << max[2] << std::endl;
  VertexPosition p0 = {min[0], min[1], min[2]};
  VertexPosition p1 = {min[0], max[1], min[2]};
  VertexPosition p2 = {max[0], min[1], min[2]};
  VertexPosition p3 = {max[0], max[1], min[2]};
  VertexPosition p4 = {min[0], min[1], max[2]};
  VertexPosition p5 = {min[0], max[1], max[2]};
  VertexPosition p6 = {max[0], min[1], max[2]};
  VertexPosition p7 = {max[0], max[1], max[2]};
  aabb_vertices << p0[0] << " " << p0[1] << " " << p0[2] << std::endl;
  aabb_vertices << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
  aabb_vertices << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;
  aabb_vertices << p3[0] << " " << p3[1] << " " << p3[2] << std::endl;
  aabb_vertices << p4[0] << " " << p4[1] << " " << p4[2] << std::endl;
  aabb_vertices << p5[0] << " " << p5[1] << " " << p5[2] << std::endl;
  aabb_vertices << p6[0] << " " << p6[1] << " " << p6[2] << std::endl;
  aabb_vertices << p7[0] << " " << p7[1] << " " << p7[2] << std::endl;
  if (node->m_tri_list != nullptr) {
    // 顶点
    for (size_t i = 0; i < 3; i++) {
      f_vertices << node->m_tri_list->getVertex()[i][0] << " "
                 << node->m_tri_list->getVertex()[i][1] << " "
                 << node->m_tri_list->getVertex()[i][2] << std::endl;
    }
  } else {
    saveBVH(node->m_left, f_vertices, aabb_vertices);
    saveBVH(node->m_right, f_vertices, aabb_vertices);
  }
}

void showBVHAABB(const std::string &aabb_path) {
  std::ifstream aabb_f(aabb_path, std::ios::in);
  std::string line;
  std::vector<VertexPosition> aabb_pt_list;
  if (aabb_f.is_open()) {
    while (std::getline(aabb_f, line)) {
      std::istringstream sstream(line);
      std::string field;
      VertexPosition pt;
      std::vector<std::string> row;
      while (std::getline(sstream, field, ' ')) {
        pt.push_back(std::stod(field));
        // std::cout << field << std::endl;
      }

      aabb_pt_list.push_back(pt);
    }
  }
  aabb_f.close();

  Gnuplot g("lines");
  for (size_t i = 0; i < aabb_pt_list.size(); i = i + 2) {
    VertexPosition min = aabb_pt_list[i];
    VertexPosition max = aabb_pt_list[i + 1];
    VertexPosition p0 = {min[0], min[1], min[2]};
    VertexPosition p1 = {min[0], max[1], min[2]};
    VertexPosition p2 = {max[0], min[1], min[2]};
    VertexPosition p3 = {max[0], max[1], min[2]};
    VertexPosition p4 = {min[0], min[1], max[2]};
    VertexPosition p5 = {min[0], max[1], max[2]};
    VertexPosition p6 = {max[0], min[1], max[2]};
    VertexPosition p7 = {max[0], max[1], max[2]};

    std::vector<double> aabb_x = {p0[0], p1[0], p0[0], p2[0], p1[0], p3[0],
                                  p2[0], p3[0], p0[0], p4[0], p1[0], p5[0],
                                  p2[0], p6[0], p3[0], p7[0], p4[0], p5[0],
                                  p4[0], p6[0], p5[0], p7[0], p6[0], p7[0]};
    std::vector<double> aabb_y = {p0[1], p1[1], p0[1], p2[1], p1[1], p3[1],
                                  p2[1], p3[1], p0[1], p4[1], p1[1], p5[1],
                                  p2[1], p6[1], p3[1], p7[1], p4[1], p5[1],
                                  p4[1], p6[1], p5[1], p7[1], p6[1], p7[1]};
    std::vector<double> aabb_z = {p0[2], p1[2], p0[2], p2[2], p1[2], p3[2],
                                  p2[2], p3[2], p0[2], p4[2], p1[2], p5[2],
                                  p2[2], p6[2], p3[2], p7[2], p4[2], p5[2],
                                  p4[2], p6[2], p5[2], p7[2], p6[2], p7[2]};

    g.set_style("linespoints").plot_xyz(aabb_x, aabb_y, aabb_z);
  }
  wait_for_key();
}

void wait_for_key() {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) ||                 \
    defined(__TOS_WIN__) // every keypress registered, also arrow keys
  cout << endl << "Press any key to continue..." << endl;

  FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
  _getch();
#elif defined(unix) || defined(__unix) || defined(__unix__) ||                 \
    defined(__APPLE__)
  std::cout << std::endl << "Press ENTER to continue..." << std::endl;

  std::cin.clear();
  std::cin.ignore(std::cin.rdbuf()->in_avail());
  std::cin.get();
#endif
  return;
}
