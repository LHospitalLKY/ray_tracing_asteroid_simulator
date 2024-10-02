/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 16:14:17
 * @LastEditors: LHospitalLKY 1141165506@qq.com
 * @LastEditTime: 2024-10-02 16:13:18
 * @FilePath: /ray_tracing_asteroid_simulator/test/bright_calcu_test.cpp
 * @Description: 测试bright_calcu.cpp的功能
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
  read_shape("/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/ariadne_shape.txt",
             facetList, verticeList, 1);

  // 计算一个面的法向量和面积
  std::cout << "======calculate one facet area and normals======" << std::endl;
  FacetIndex facet_first = facetList[0];
  FacetNormal normalVec;
  double area;
  calculate_areas_normals_onepiece(facet_first, verticeList, normalVec, area);

  std::cout << normalVec[0] << " " << normalVec[1] << " " << normalVec[2]
            << std::endl;
  std::cout << area << std::endl;

  // 计算所有面的法向量和面积
  std::cout << "======calculate all facets area and normals======" << std::endl;
  NormalList normalList;
  AreaList areaList;
  calculate_areas_normals(facetList, verticeList, normalList, areaList);
  std::cout << "Find " << normalList.size() << " normals and "
            << areaList.size() << " areas." << std::endl;
  // for (size_t i = 0; i < facetList.size(); i++) {
  //     std::cout << "Facet " << i << "Information" << std::endl;
  //     std::cout << "Facet " << facetList[i][0] << " " << facetList[i][1] << " " << facetList[i][2] << std::endl;
  //     std::cout << "Normal " << normalList[i][0] << " " << normalList[i][1] << " " << normalList[i][2] << std::endl;
  //     std::cout << "Area " << areaList[i] << std::endl;
  // }

  // 可见性判断函数验证
  std::cout << "======Visibility Check======" << std::endl;
  ViewVector sun_vector = {1.46753600 * 1000000000, 1.30933200 * 1000000000,
                           0.08241928 * 1000000000};
  ViewVector obs_vector = {0.71406180 * 1000000000, 0.63999610 * 1000000000,
                           0.08237720 * 1000000000};

  std::vector<bool> visibleList;
  facetVisiable_calcu(normalList, sun_vector, obs_vector, visibleList);
  int visiable_count = 0;
  for (size_t i = 0; i < visibleList.size(); i++) {
    if (visibleList[i]) {
      visiable_count++;
#ifdef DEBUG
      std::cout << "Facet " << i << " is visiable" << std::endl;
      std::cout << "Normal " << normalList[i][0] << " " << normalList[i][1] <<" " << normalList[i][2] << std::endl;
      std::cout << "Area " << areaList[i] << std::endl;
#endif
    }
  }
  std::cout << "Find " << visiable_count << " visible facets." << std::endl;

  write_vis_shape_stl(
      "/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/visiable_test.stl",
      facetList, verticeList, normalList, visibleList);

  // 读取damit光变文件
  std::cout << "======Load Damit======" << std::endl;
  VectorList sun_vec_list, obs_vec_list;
  TimeList jd_time_list;
  BriList bri_list;
  read_lcurve_damit("/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/ariadne.lc",
                    jd_time_list, bri_list, sun_vec_list, obs_vec_list);
  std::cout << jd_time_list.size() << " " << sun_vec_list.size() << " "
            << obs_vec_list.size() << std::endl;

  // 转换太阳和观测向量到固联坐标系
  std::cout << "======Convert to IAU Coordinate======" << std::endl;
  // target ariadne
  double jd_0 = 2438882.233275;
  double beta = -15.;
  double lambda = 253.;
  double P = 5.76198;
  double phi_0 = 0.0;
  
  // ==target 135==
  // double jd_0 = 2443845.852388;
  // double beta = 58.34;
  // double lambda = 274.86;
  // double P = 8.40059506;
  // double phi_0 = 0.0;
  // double M[3][3];
  Eigen::Matrix3d R;
  for (int i = 0; i < jd_time_list[i]; i++) {
    rotate(beta, lambda, P, phi_0, jd_time_list[i] - jd_0, R);
    std::cout << "rotated sun and obs vec by myself" << std::endl;
    // std::cout << R << std::endl;
    Eigen::Vector3d sunView{sun_vec_list[i][0], sun_vec_list[i][1],
                            sun_vec_list[i][2]};
    Eigen::Vector3d obsView{obs_vec_list[i][0], obs_vec_list[i][1],
                            obs_vec_list[i][2]};
    std::cout << (R * sunView).transpose() << std::endl;
    std::cout << (R * obsView).transpose() << std::endl;
  }

  // 验证亮度计算函数
  std::cout << "======Brightness calculate function======" << std::endl;
  double A0 = 0.5;
  double D = 0.1;
  double k = -0.5;
  double c = 0.1;
  double albedo = 1.0;
  // 带相函数的亮度计算
  BriList bright_with_phase;
  BriList bright_without_phase;
  calculate_light(facetList, verticeList, jd_time_list, obs_vec_list,
                  sun_vec_list, bright_with_phase, beta, lambda, P, phi_0, &A0,
                  &D, &k, c, albedo, jd_0);
  calculate_light(facetList, verticeList, jd_time_list, obs_vec_list,
                  sun_vec_list, bright_without_phase, beta, lambda, P, phi_0,
                  NULL, NULL, NULL, c, albedo, jd_0);
  std::cout << "Bright calculate with phase function" << std::endl;
  for (size_t i = 0; i < bright_with_phase.size(); i++) {
    std::cout << bright_with_phase[i] << ", ";
  }
  std::cout << std::endl;
  std::cout << "Bright calculate without phase function" << std::endl;
  for (size_t i = 0; i < bright_without_phase.size(); i++) {
    std::cout << bright_without_phase[i] << ", ";
  }
  std::cout << std::endl;
  std::cout << "Origin Brightness" << std::endl;
  for (size_t i = 0; i < bri_list.size(); i++) {
    std::cout << bri_list[i] << ", ";
  }

  // 得到不同周期下的亮度, 以及与真实亮度之间的cost
  std::cout << "======Brightness with different Period======" << std::endl;
  for (double P = 5.0; P < 7.0; P += 0.01) {
    BriList bright_with_phase;
    calculate_light(facetList, verticeList, jd_time_list, obs_vec_list,
                    sun_vec_list, bright_with_phase, beta, lambda, P, phi_0,
                    &A0, &D, &k, c, albedo, jd_0);
    normal_bright(bright_with_phase);
    double cost_with_phase = 0.0;
    for (size_t i = 0; i < bright_with_phase.size(); i++) {
      cost_with_phase += (bright_with_phase[i] - bri_list[i]) *
                         (bright_with_phase[i] - bri_list[i]);
    }
    std::cout << "P: " << P << " Cost with phase: " << cost_with_phase << std::endl;
  }
  // calculate_light(facetList, verticeList, jd_time_list, obs_vec_list,
  //                 sun_vec_list, bright_without_phase, beta, lambda, P, phi_0,
  //                 NULL, NULL, NULL, c, albedo, jd_0);
  // std::cout << "Bright calculate with phase function" << std::endl;
  // for (size_t i = 0; i < bright_with_phase.size(); i++) {
  //   std::cout << bright_with_phase[i] << ", ";
  // }

  return 0;
}