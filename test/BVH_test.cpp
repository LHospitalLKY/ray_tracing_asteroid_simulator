/*
 * @Author: LHospitalLKY 1141165506@qq.com
 * @Date: 2024-10-03 10:18:57
 * @LastEditors: LHospitalLKY 1141165506@qq.com
 * @LastEditTime: 2024-10-05 13:09:53
 * @FilePath: /ray_tracing_asteroid_simulator/test/BVH_test.cpp
 * @Description: 测试BVH树
 */

#include "../include/BVH.h"
#include "../include/obj_loader.h"
#include <cstddef>

void test_triangle() {
  VertexPosition p1 = {1.0, 0.0, 0.0};
  VertexPosition p2 = {0.0, 1.0, 0.0};
  VertexPosition p3 = {0.0, 0.0, 1.0};
  ViewVector normal = facet_normal_calcu(p1, p2, p3);

  Triangle tri(p1, p2, p3, normal);

  // 直线与其的交点
  VertexPosition origin = {0., 0., 10};
  ViewVector direction = {0., 0., -1.};
  double t;
  VertexPosition intersect;
  Intersect inter = tri.intersect(origin, direction);
  if (inter.hitted == true)
    std::cout << "intersect point at " << inter.t << " : " << inter.p;

  origin = {10., 10., 10};
  direction = {-1., -1., -1.};
  inter = tri.intersect(origin, direction);
  if (inter.hitted == true)
    std::cout << "intersect point at " << inter.t << " : " << inter.p;

  origin = {10., 10., 10};
  direction = {1., 1., 1.};
  inter = tri.intersect(origin, direction);
  if (inter.hitted == true)
    std::cout << "intersect point at " << inter.t << " : " << inter.p;

  // AABB包围盒
  AABB aabb = tri.getAABB();
  showAABB(aabb);
}

void test_root_aabb_gen() {
  // 读取形状文件
  OBJLoader loader;
  loader.load("/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/"
              "test_data/monkey.obj");
  const VerticeList *vertexList = loader.getVertices();
  const FacetList *facetList = loader.getFacets();
  const NormalList *normalList = loader.getNormals();

  std::vector<Triangle *> tri_list;
  for (size_t i = 0; i < facetList->size(); i++) {
    Triangle *tri = new Triangle(
        (*vertexList)[(*facetList)[i][0]], (*vertexList)[(*facetList)[i][1]],
        (*vertexList)[(*facetList)[i][2]], (*normalList)[i]);
    tri_list.push_back(tri);
  }

  std::cout << tri_list.size() << std::endl;

  BVHBuildNode *node = recursiveBuild(tri_list);

  std::ofstream f_bvh_pt("monkey_head_bvh_pt_test.txt", std::ios::out);
  std::ofstream f_aabb("monkey_head_bvh_test.txt", std::ios::out);
  saveBVH(node, f_bvh_pt, f_aabb);
  f_bvh_pt.close();
  f_aabb.close();

  VertexPosition obsPos = {0.0, 10.0, 0.0};

  double half_length = 1.5;
  int height = 1024;
  int width = 1024;
  int k = 0;
  // double img[height][width];

  std::ofstream f("monkey_head_bvh_test.pgm", std::ios::out);
  std::ofstream f_p("monkey_head_bvh_test.txt", std::ios::out);
  f << "P2" << std::endl;
  f << height << " " << width << std::endl;
  f << 1024 << std::endl;
  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j++) {
      // 得到观测方向
      // VertexPosition endPos = {((double)i * (2*half_length) / (double)height)
      // - half_length, ((double)j * (2*half_length) / (double)width) -
      // half_length, 0}; VertexPosition endPos = {0, ((double)i *
      // (2*half_length) / (double)height) - half_length, ((double)j *
      // (2*half_length) / (double)width) - half_length};
      VertexPosition endPos = {
          ((double)i * (2 * half_length) / (double)height) - half_length, 0,
          ((double)j * (2 * half_length) / (double)width) - half_length};
      ViewVector obsDir = endPos - obsPos;
      obsDir = normalize(obsDir);

      // double t;
      // VertexPosition first_intersec_pt;
      Intersect inter = getIntersect(node, obsPos, obsDir);
      bool intersect_or_not = inter.hitted;
      double t = inter.t;
      VertexPosition first_intersec_pt = inter.p;
      // bool intersect_or_not = pointAndShapeIntersect(*facetList, *vertexList,
      // obsPos, obsDir, t, first_intersec_pt);

      // 若无交点
      if (intersect_or_not == false) {
        f << 0 << " ";
      } else {
        // TODO: 这里改成深度, 而不是t
        // TODO: 测试一下其他方向
        f << (int)((11. - t) * 100) << " ";
        f_p << first_intersec_pt[0] << " " << first_intersec_pt[1] << " "
            << first_intersec_pt[2] << std::endl;
      }
    }
    f << std::endl;
  }
  f.close();
  f_p.close();

  return;
}

void test_BVH_build() {
  VertexPosition p1 = {1.0, 0.0, 0.0};
  VertexPosition p2 = {0.0, 1.0, 0.0};
  VertexPosition p3 = {0.0, 0.0, 1.0};
  VertexPosition p4 = {0.0, -1.0, 0.0};
  VertexPosition p5 = {-1.0, 0.0, 0.0};
  VerticeList verticeList = {p1, p2, p3, p4, p5};
  FacetIndex f1 = {0, 1, 2};
  FacetIndex f2 = {0, 2, 3};
  FacetIndex f3 = {1, 4, 2};
  FacetList facetList = {f1, f2, f3};
  NormalList normalList;
  for (size_t i = 0; i < facetList.size(); i++) {
    normalList.push_back(facet_normal_calcu(verticeList[facetList[i][0]],
                                            verticeList[facetList[i][1]],
                                            verticeList[facetList[i][2]]));
  }

  std::vector<Triangle *> tri_list;
  for (size_t i = 0; i < facetList.size(); i++) {
    Triangle *tri =
        new Triangle(verticeList[facetList[i][0]], verticeList[facetList[i][1]],
                     verticeList[facetList[i][2]], normalList[i]);
    tri_list.push_back(tri);
  }
  // std::cout << normalList.size() << std::endl;

  recursiveBuild(tri_list);

}

int main(int argc, const char **argv) {
  test_triangle();

  test_BVH_build();

  test_root_aabb_gen();


  return 0;
}
