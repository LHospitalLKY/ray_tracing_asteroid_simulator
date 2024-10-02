/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-10-01 14:05:18
 * @LastEditors: LHospitalLKY 1141165506@qq.com
 * @LastEditTime: 2024-10-02 23:03:59
 * @FilePath: /g2o_test/test/visibility_test.cpp
 * @Description: 可见性测试程序
 */

#include "../include/bright_calcu.h"
#include "../include/obj_loader.h"
#include <cmath>
#include <cstddef>
#include <fstream>

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

  // 观测原点
  VertexPosition obsPos = {0.0, 0.0, 10.0};

  double half_length = 1.5;
  int height = 1024;
  int width = 1024;
  int k = 0;
  // pgm图像与点列
  std::ofstream f("monkey_head_test.pgm", std::ios::out);
  std::ofstream f_p("monkey_head_test.txt", std::ios::out);
  f << "P2" <<  std::endl;
  f << height << " " << width << std::endl;
  f << 1024 << std::endl;
  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j++) {
      // 得到观测方向
      VertexPosition endPos = {((double)i * (2*half_length) / (double)height) - half_length, ((double)j * (2*half_length) / (double)width) - half_length, 0};
      ViewVector obsDir = endPos - obsPos;
      obsDir = normalize(obsDir);

      double t;
      VertexPosition first_intersec_pt;
      bool intersect_or_not = pointAndShapeIntersect(*facetList, *vertexList, obsPos, obsDir, t, first_intersec_pt);

      // 若无交点
      if (intersect_or_not == false) {
        f << 0 << " ";
      }
      else {
        // TODO: 这里改成深度, 而不是t
        // TODO: 测试一下其他方向 
        f << (int)((11. - t) * 100) << " ";
        f_p << first_intersec_pt[0] << " " << first_intersec_pt[1] << " " <<first_intersec_pt[2] << std::endl;
      }
    }
    f << std::endl;
  }
  f.close();
  f_p.close();


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
