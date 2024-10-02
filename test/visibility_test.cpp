/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-10-01 14:05:18
 * @LastEditors: LHospitalLKY 1141165506@qq.com
 * @LastEditTime: 2024-10-02 18:12:01
 * @FilePath: /g2o_test/test/visibility_test.cpp
 * @Description: 可见性测试程序
 */

#include "../include/bright_calcu.h"
#include "../include/obj_loader.h"
#include <cmath>
#include <cstddef>

void write_vis_shape_stl(std::string filename, const FacetList &facetList,
                         const VerticeList &verticeList,
                         const NormalList &normalList,
                         const VisiableList visiableList) {
  FacetList facet_vis;
  NormalList normal_vis;

  for (size_t i = 0; i < visiableList.size(); i++) {
    if (visiableList[i] == true) {
      facet_vis.push_back(facetList[i]);
      normal_vis.push_back(normalList[i]);
    }
  }
  std::cout << "Visiable Facets num: " << facet_vis.size() << std::endl;

  write_shape_stl(filename, facet_vis, verticeList, normalList);
}

int main(int argc, const char **argv) {
  // 测试猴头的与射线的交点
  OBJLoader loader;
  loader.load("/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/monkey.obj");
  const VerticeList *vertexList = loader.getVertices();
  const FacetList *facetList = loader.getFacets();
  const NormalList *normalList = loader.getNormals();

  // 观测方向
  VertexPosition sunPos = {0.01, 0.5, 10.0};
  ViewVector sunDirect = {0.01, 0.0, -1.0};
  VertexPosition obsPos = {0.01, 0.5, 10.0};
  ViewVector obsDirect = {0.0, 0.0, -1.0};

  VisiableList vis_list;

  // facetVisiable_calcu_final(*facetList, *vertexList, sunPos, sunDirect, obsPos, obsDirect, vis_list);
  double t;
  VertexPosition first_intersec_pt;
  pointAndShapeIntersect(*facetList, *vertexList, obsPos, obsDirect, t, first_intersec_pt);

  std::cout << "First intersection at " << t << ": " << first_intersec_pt[0] << " " << first_intersec_pt[1] << " " << first_intersec_pt[2] << std::endl;

  // 测试一下面片交点的计算
  // VertexPosition p0 = {1., 0., 0.};
  // VertexPosition p1 = {0., 1., 0.};
  // VertexPosition p2 = {0., 0., 1.};
  // VertexPosition p_o = {0., 0., 0.};
  // ViewVector direction = {1, 1, 0};
  // VerticeList p_list;
  // p_list.push_back(p0);
  // p_list.push_back(p1);
  // p_list.push_back(p2);
  // FacetIndex facet = {0, 1, 2};

  // for (size_t i = 0; i <= 20; i++) {
  //   direction[2] = 0.1*(double)i - 1;
  //   ViewVector direction_ = normalize(direction);
  //   ViewVector insec_p;
  //   double t;
  //   bool insec_or_not = pointToTriangleDistance(p_o, direction_, facet, p_list, insec_p, t);
  //   if (insec_or_not == true) {
  //     std::cout << "Find intersect point with "<< t << " at: " << insec_p[0] << " " << insec_p[1] << " " << insec_p[2] << std::endl;
  //   }
  //   else {
  //     std::cout << "Not intersection." << std::endl;
  //   }
  // }



  // 读取shape文件(obj)
  // std::cout << "=========Read Shape File: monkey.obj==========" << std::endl;
  // const FacetList *facetList;
  // const VerticeList *verticeList;
  // const NormalList *normalList;
  
  // OBJLoader loader;
  // loader.load("/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/monkey.obj");
  // verticeList = loader.getVertices();
  // facetList = loader.getFacets();
  // normalList = loader.getNormals();

  // // 太阳与观测方向
  // ViewVector sunVec = {0.0, 0.0, 1.0};
  // ViewVector obsVec = {1.0, 0.0, 0.0};

  // // 使用快速法判断可见性
  // VisiableList vis_method_1;
  // facetVisiable_calcu(*normalList, sunVec, obsVec, vis_method_1);
  // // 写入可见shape文件
  // write_vis_shape_stl("/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/monkey_vis_1.stl", *facetList, *verticeList, *normalList, vis_method_1);

  return 0;
}
