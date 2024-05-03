// bright_calcu.cpp

#include "../include/bright_calcu.h"
#include <cassert>
#include <cstddef>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <g2o/stuff/macros.h>
#include <math.h>

double phaseFunction(const ViewVector &obsView, const ViewVector &sunView,
                     double A0, double D, double k) {
  // 观测相角
  double alpha = phaseAngle(obsView, sunView);
  // 函数项1: A0*exp(-\alpha/D)
  double term_1 = A0 * exp(-alpha / D);
  // 函数项2: k\alpha
  double term_2 = k * alpha;
  // 函数值
  return term_1 + term_2 + 1;
}

double LSLScatter(double phase, const FacetNormal &normalVec,
                  const ViewVector &obsView, const ViewVector &sunView,
                  double c, double albedo) {
  // 计算入射角与出射角
  double mu;  // 出射角
  double mu0; // 入射角

  mu = obsView[0] * normalVec[0] + obsView[1] * normalVec[1] +
       obsView[2] * normalVec[2];
  mu0 = sunView[0] * normalVec[0] + sunView[1] * normalVec[1] +
        sunView[2] * normalVec[2];

  // 计算散射反射率
  return phase * albedo * mu * mu0 * (1.0 / (mu + mu0) + c);
}

bool visiable_by_Vec(const FacetNormal &normalVec, const FacetNormal &sunVec) {
  // 计算两个向量的夹角余弦, 余弦小于0则可见
  double cos_theta = normalVec[0] * sunVec[0] + normalVec[1] * sunVec[1] +
                     normalVec[2] * sunVec[2];
  return cos_theta > 0;
}

void facetVisiable_calcu(const NormalList &normalList, const ViewVector sunVec,
                         const ViewVector obsVec, VisiableList &visiableList) {
  // 计算面的可见性
  // TODO: 增加通过面遮挡判断来计算可见性的方法
  for (size_t i = 0; i < normalList.size(); i++) {
    visiableList.push_back(visiable_by_Vec(normalList[i], sunVec) &&
                           visiable_by_Vec(normalList[i], obsVec));
  }
}

void calculate_light(const FacetList &facetList, const VerticeList &vertexList,
                     // const NormalList &normalList,
                     const TimeList &timeList, const VectorList &obsViewVector,
                     const VectorList &sunViewVector, BriList &brightList,
                     double beta, double lambda, double P, double phi_0,
                     const double *A0, const double *D, const double *k,
                     double c, double albedo, double time_0) {
  // 判断太阳、观测者方向向量的长度是否相同
  assert(obsViewVector.size() == sunViewVector.size());

  // 1. 旋转观测和太阳向量到固联系
  VectorList obsView_IAU, sunView_IAU;
  // 初始时刻
  if (time_0 - 0.0 < 1e-3) {
    time_0 = timeList[0];
  }
  // 遍历
  for (size_t i = 0; i < obsViewVector.size(); i++) {
    Eigen::Matrix3d R;
    rotate(beta, lambda, P, phi_0, timeList[i] - time_0, R);
    // std::cout << R << std::endl;
    Eigen::Vector3d obsView{obsViewVector[i][0], obsViewVector[i][1],
                            obsViewVector[i][2]};
    Eigen::Vector3d sunView(sunViewVector[i][0], sunViewVector[i][1],
                            sunViewVector[i][2]);
    obsView = R * obsView;
    sunView = R * sunView;

#ifdef DEBUG
    std::cout << "Obs and Sun Vec in IAU: " << std::endl;
    std::cout << (obsView / obsView.norm()).transpose() << std::endl;
    std::cout << (sunView / sunView.norm()).transpose() << std::endl;
#endif

    obsView_IAU.push_back({obsView[0], obsView[1], obsView[2]});
    sunView_IAU.push_back({sunView[0], sunView[1], sunView[2]});
  }

  // 2. 计算所有面的法向和面积, 此外要增加一个简单的法向朝向判断语句
  NormalList normalList;
  AreaList areaList;
  calculate_areas_normals(facetList, vertexList, normalList, areaList);

  for (size_t i = 0; i < obsViewVector.size(); i++) {
    // 计算面可见性
    // version 1: 仅通过太阳、观测者方向向量判断
    VisiableList visiableList;
    facetVisiable_calcu(normalList, sunView_IAU[i], obsView_IAU[i],
                        visiableList);

    // 计算亮度
    brightList.push_back(calculate_light_oneView(
        normalList, areaList, visiableList, obsView_IAU[i], sunView_IAU[i], A0,
        D, k, c, albedo));
  }

  normal_bright(brightList);
}

// normalBright
void normal_bright(BriList &bright_list) {
  double sumBright = 0.0;
  int sample_num = bright_list.size();
  
  if (bright_list.empty()) {
    std::cerr << "Brightness list is empty!" << std::endl;
    return;
  }

  for (size_t i = 0; i < bright_list.size(); i++) {
    sumBright += bright_list[i];
  }
  // sumBright = pow(sumBright, 2);

  for (size_t i = 0; i < bright_list.size(); i++) {
    bright_list[i] = sample_num * bright_list[i] / sumBright;
  }
}

double calculate_light_oneView(const NormalList &normalList,
                               const AreaList &areaList,
                               const VisiableList &visiableList,
                               const ViewVector &obsView,
                               const ViewVector &sunView, const double *A0,
                               const double *D, const double *k, double c,
                               double albedo) {
  // 1. 判断验证
  assert(normalList.size() == areaList.size());
  assert(normalList.size() == visiableList.size());

  // 2. 计算相函数
  double phase_value;
  phase_value = 1.0;
  if (A0 != NULL || D != NULL || k != NULL) {
    phase_value = phaseFunction(obsView, sunView, *A0, *D, *k);
  }
  // std::cout << phase_value << std::endl;
  // 3. 遍历所有的面
  double brightness = 0.0;
  for (size_t i = 0; i < normalList.size(); i++) {
    if (visiableList[i] != true) {
      // FIXME: DEBUG语句
#ifdef DEBUG
      std::cout << "The facet is not visiable!" << std::endl;
#endif // DEBUG
      continue;
    }
    // 得到散射亮度
    double scatter =
        LSLScatter(phase_value, normalList[i], obsView, sunView, c);
    brightness = brightness + albedo * scatter * areaList[i];
  }

  return brightness;
}

void calculate_areas_normals_onepiece(const FacetIndex &facetIndex,
                                      const VerticeList &vertexList,
                                      FacetNormal &normalVec, double &area) {
  // TODO: 写一个法向量方向判断函数, 保证法向量朝外,
  // 否则需要保证顶点顺序能够保证法向量朝外 得到面片的三个顶点
  if (facetIndex.size() != 3) {
    std::cerr << "Facet is not triangle!" << std::endl;
    return;
  }

  // 三角形的三个顶点索引
  int index_1 = facetIndex[0] - 1;
  int index_2 = facetIndex[1] - 1;
  int index_3 = facetIndex[2] - 1;

  // 得到三角形三个顶点的坐标
  double x1 = vertexList[index_1][0];
  double y1 = vertexList[index_1][1];
  double z1 = vertexList[index_1][2];

  double x2 = vertexList[index_2][0];
  double y2 = vertexList[index_2][1];
  double z2 = vertexList[index_2][2];

  double x3 = vertexList[index_3][0];
  double y3 = vertexList[index_3][1];
  double z3 = vertexList[index_3][2];

  // 打印x1, y1, z1, x2, y2, z2, x3, y3, z3
  // std::cout << x1 << " " << y1 << " " << z1 << std::endl;
  // std::cout << x2 << " " << y2 << " " << z2 << std::endl;
  // std::cout << x3 << " " << y3 << " " << z3 << std::endl;

  // 计算叉乘
  double v1[3], v2[3], cv[3];
  v1[0] = x2 - x1;
  v1[1] = y2 - y1;
  v1[2] = z2 - z1;

  v2[0] = x3 - x1;
  v2[1] = y3 - y1;
  v2[2] = z3 - z1;

  // 打印v1, v2
  // std::cout << v1[0] << " " << v1[1] << " " << v1[2] << std::endl;
  // std::cout << v2[0] << " " << v2[1] << " " << v2[2] << std::endl;

  cross(v1, v2, cv);
  // 打印cv
  // std::cout << cv[0] << " " << cv[1] << " " << cv[2] << std::endl;

  // 得到法向量
  double normCV = norm(cv);
  // std::cout << normCV << std::endl;
  FacetNormal().swap(normalVec);
  normalVec.push_back(cv[0] / normCV);
  normalVec.push_back(cv[1] / normCV);
  normalVec.push_back(cv[2] / normCV);

  // 得到面积
  area = 0.5 * normCV;
}

void calculate_areas_normals(const FacetList &facetList,
                             const VerticeList &vertexList,
                             NormalList &normalList, AreaList &areaList) {
  // 遍历所有面片
  for (size_t i = 0; i < facetList.size(); i++) {
    // 计算面片的法向和面积
    FacetNormal normalVec;
    double area;
    calculate_areas_normals_onepiece(facetList[i], vertexList, normalVec, area);
    normalList.push_back(normalVec);
    areaList.push_back(area);
  }
}