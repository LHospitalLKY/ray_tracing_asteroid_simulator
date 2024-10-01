/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-10-01 14:05:18
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-10-01 23:56:10
 * @FilePath: /g2o_test/test/visibility_test.cpp
 * @Description: 可见性测试程序
 */

#include "../include/bright_calcu.h"
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
  // 读取shape文件
  std::cout << "======Read Shape File: ariadne======" << std::endl;
  FacetList facetList;
  VerticeList verticeList;
  read_shape(
      "/home/lho/MyProgramm/2024.04/g2o_test/test_data/ariadne_shape.txt",
      facetList, verticeList, 1);
  return 0;
}
