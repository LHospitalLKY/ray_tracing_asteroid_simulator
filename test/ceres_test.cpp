/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-04 21:32:11
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-08-12 22:20:42
 * @FilePath: /g2o_test/test/ceres_test.cpp
 * @Description: 尝试使用ceres来进行优化
 */

#include "../include/bright_calcu.h"

#include <bits/getopt_core.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <cstddef>
#include <iostream>
#include <random>

struct BrightnessResidual {
  BrightnessResidual(double P, double brightMeasured)
      : _P(P), _brightMeasured(brightMeasured) {}

  // 设置模型
  void setModel(const FacetList *facetList, const VerticeList *verticeList,
                const NormalList *normalList, const AreaList *areaList,
                const ViewVector *obsVec, const ViewVector *sunVec, double *A0,
                double *D, double *k, double c, double albedo, // double beta,
                // double lambda,
                double phi_0, double t0, double t) {
    _facetList = facetList;
    _verticeList = verticeList;
    _normalList = normalList;
    _areaList = areaList;
    _obsVector = obsVec;
    _sunVector = sunVec;
    _A0 = A0;
    _D = D;
    _k = k;
    _c = c;
    _albedo = albedo;
    // _beta = beta;
    // _lambda = lambda;
    _phi_0 = phi_0;
    _t0 = t0;
    _t = t;
  }

  template <typename T>
  bool operator()(const T *const P, const T *const L, const T *const B,
                  T *residual) const {
    // ceres::Jet<double, 1> P_(P);
    // std::cout << double(P_.a) << std::endl;
    // auto p = P[0];
    double lambda = L[0];
    double beta = B[0];
    double p = P[0];
    // ceres::Jet<double, 1> j;
    // j.a = 1;
    // auto p_ = p.a;
    double brightCalcu = calcuBright(lambda, beta, p);

    // 计算出所有的亮度
    // TODO: 亮度归一化这部分到底应该怎么做? 如何避免重复计算

    residual[0] = T(brightCalcu) - _brightMeasured;

    // residual[0] = T(0);

    return true;
  }

  // 计算亮度
  double calcuBright(double lambda, double beta, double P) const {
    // std::cout << "Called calcuBright: P " << P << std::endl;
    // 1. 坐标转换
    // 旋转矩阵
    Eigen::Matrix3d R;

    // std::cout << (*_obsVector)[0] << " " << (*_obsVector)[1] << " " <<
    // (*_obsVector)[2] << std::endl; std::cout << (*_sunVector)[0] << " " <<
    // (*_sunVector)[1] << " " << (*_sunVector)[2] << std::endl; double omega =
    // 2 * M_PI / 8.40059506; rotate(_beta, _lambda, measurement(), _phi_0, _t -
    // _t0, R);

    // rotate(_beta, _lambda, 8.40059506, _phi_0, _t - _t0, R);
    rotate(beta, lambda, P, _phi_0, _t - _t0, R);

    // std::cout << "Rotation" << std::endl;
    // std::cout << R << std::endl;

    // 旋转观测向量
    Eigen::Vector3d obsVec_iau_eigen{(*_obsVector)[0], (*_obsVector)[1],
                                     (*_obsVector)[2]};
    Eigen::Vector3d sunVec_iau_eigen{(*_sunVector)[0], (*_sunVector)[1],
                                     (*_sunVector)[2]};
    obsVec_iau_eigen = R * obsVec_iau_eigen;
    sunVec_iau_eigen = R * sunVec_iau_eigen;
    ViewVector obsVec_iau{obsVec_iau_eigen[0], obsVec_iau_eigen[1],
                          obsVec_iau_eigen[2]};
    ViewVector sunVec_iau{sunVec_iau_eigen[0], sunVec_iau_eigen[1],
                          sunVec_iau_eigen[2]};

#ifdef DEBUG
    std::cout << "Obs IAU Vector:" << std::endl;
    std::cout << obsVec_iau[0] << " " << obsVec_iau[1] << " " << obsVec_iau[2]
              << std::endl;
    std::cout << "Sun IAU Vector:" << std::endl;
    std::cout << sunVec_iau[0] << " " << sunVec_iau[1] << " " << sunVec_iau[2]
              << std::endl;
#endif

    // 2. 计算亮度
    // 首先计算面的可见性
    VisiableList visList;
    facetVisiable_calcu(*_normalList, sunVec_iau, obsVec_iau, visList);

#ifdef DEBUG
    for (size_t i = 0; i < visList.size(); i++) {
      if (visList[i]) {
        std::cout << "Facet " << i << " is visiable" << std::endl;
        std::cout << "Normal " << (*_normalList)[i][0] << " "
                  << (*_normalList)[i][1] << " " << (*_normalList)[i][2]
                  << std::endl;
        std::cout << "Area " << (*_areaList)[i] << std::endl;
      }
    }
#endif

    double bright = calculate_light_oneView(
        *_normalList, *_areaList, visList, obsVec_iau, sunVec_iau, _A0, _D, _k);

    // std::cout << bright << std::endl;
    return bright;
  }
  // 自变量因变量
  const double _P;
  const double _brightMeasured;

  // 形状相关变量
  const FacetList *_facetList = nullptr;     // 面片列表
  const VerticeList *_verticeList = nullptr; // 顶点列表
  const NormalList *_normalList = nullptr;   // 法向量列表
  const AreaList *_areaList = nullptr;       // 面积列表
  // double _allArea;                        // 可见面面积和

  // 自转状态相关变量
  // double _beta = 0;   // 自转轴指向beta
  // double _lambda = 0; // 自转轴指向lambda
  double _phi_0 = 0; // 初始自转相位
  double _t0 = 0;    // 初始时刻
  double _t = 0;     // 观测时刻

  // 观测几何
  const ViewVector *_obsVector = nullptr; // 观测方向向量, 惯性系
  const ViewVector *_sunVector = nullptr; // 太阳方向向量, 惯性系

  // 散射参数
  double *_A0 = nullptr; // 散射相函数参数A0
  double *_D = nullptr;  // 散射相函数参数D
  double *_k = nullptr;  // 散射相函数参数k

  double _c = 0.1;      // LSL参数c
  double _albedo = 1.0; // 反照率
};

int main(int argc, const char **argv) {
  google::InitGoogleLogging(argv[0]);

  double jd_0 = 2438882.233275;
  double beta = -15.;
  double lambda = 253.;
  const double P = 5.76198;
  const double P_start = 5.3;
  const double beta_start = -13.;
  const double lambda_start = 251.;
  // double jd_0 = 2443845.852388;
  // double beta = -15.;
  // double lambda = 253.;
  // const double P = 5.76198;
  // const double P_start = 6.0;
  double P_dyn = P_start;
  double beta_dyn = beta_start;
  double lambda_dyn = lambda_start;
  double param[3] = {lambda_dyn, beta_dyn, P_dyn};
  double phi_0 = 0.0;
  double A0 = 0.5;
  double D = 0.1;
  double k = -0.5;
  double c = 0.1;
  double albedo = 1.0;

  // 读取形状
  FacetList facetList;
  VerticeList verticeList;
  NormalList normalList;
  AreaList areaList;
  read_shape(
      "/home/lho/MyProgramm/2024.04/g2o_test/test_data/ariadne_shape.txt",
      // "/home/lho/MyProgramm/2024.04/g2o_test/test_data/outshape.txt",
      facetList, verticeList, 1);
  calculate_areas_normals(facetList, verticeList, normalList, areaList);

#ifdef DEBUG
  for (size_t i = 0; i < facetList.size(); i++) {
    std::cout << "Facet " << i << "Information" << std::endl;
    std::cout << "Facet " << facetList[i][0] << " " << facetList[i][1] << " "
              << facetList[i][2] << std::endl;
    std::cout << "Normal " << normalList[i][0] << " " << normalList[i][1] << " "
              << normalList[i][2] << std::endl;
    std::cout << "Area " << areaList[i] << std::endl;
  }
#endif

  // 读取观测数据
  VectorList sun_vec_list, obs_vec_list;
  TimeList jd_time_list;
  BriList bri_list;
  read_lcurve_damit(
      // "/home/lho/MyProgramm/2024.04/g2o_test/test_data/ariadne.lc",
      "/home/lho/MyProgramm/2024.04/g2o_test/test_data/ariadne_with_sim.lc",
      // "/home/lho/MyProgramm/2024.04/g2o_test/test_data/135.lc",
      jd_time_list, bri_list, sun_vec_list, obs_vec_list);

  // 添加随机数
  std::random_device r;
  std::default_random_engine e(r());
  std::normal_distribution<double> n(0, 0.1);
  for (size_t i = 0; i < bri_list.size(); i++) {
    // bri_list[i] = bri_list[i] * 8.679929627312347;
    double noise = n(e);
    bri_list[i] = bri_list[i] + noise;
    // std::cout << noise << std::endl;
  }

  BriList bri_list_for_normalize; // 如果是rel亮度, 需要先计算一个总亮度,
                                  // 根据总亮度归一化
  calculate_light(facetList, verticeList, jd_time_list, obs_vec_list,
                  sun_vec_list, bri_list_for_normalize, beta, lambda, P, phi_0,
                  &A0, &D, &k, 0.1, 1.0, jd_0);
  double ave_bright = 0.0;
  for (size_t i = 0; i < bri_list_for_normalize.size(); i++) {
    ave_bright += bri_list_for_normalize[i];
  }
  ave_bright = ave_bright / bri_list_for_normalize.size();
  std::cout << "Ave Bright " << ave_bright << std::endl;

  // ================
  // 创建优化器
  ceres::Problem problem;
  for (size_t i = 0; i < bri_list.size(); i++) {
    // 创建
    // 创建残差

    // 向问题中添加残差项
    BrightnessResidual *residual = new BrightnessResidual(P_start, bri_list[i]);
    // residual->setModel(&facetList, &verticeList, &normalList, &areaList,
    //                    &obs_vec_list[i], &sun_vec_list[i], &A0, &D, &k, c,
    //                    albedo, beta, lambda, phi_0, jd_0, jd_time_list[i]);
    residual->setModel(&facetList, &verticeList, &normalList, &areaList,
                       &obs_vec_list[i], &sun_vec_list[i], &A0, &D, &k, c,
                       albedo, phi_0, jd_0, jd_time_list[i]);

    ceres::NumericDiffCostFunction<BrightnessResidual, ceres::CENTRAL, 1, 1, 1,
                                   1> *cost_function =
        new ceres::NumericDiffCostFunction<BrightnessResidual, ceres::CENTRAL,
                                           1, 1, 1, 1>(residual);
    // ceres::AutoDiffCostFunction<BrightnessResidual, 1, 1> *cost_function =
    //     new ceres::AutoDiffCostFunction<BrightnessResidual, 1, 1>(residual);
    // 向问题中添加残差项
    // problem.AddResidualBlock(cost_function, NULL, &P_dyn);
    problem.AddResidualBlock(cost_function, NULL, &P_dyn, &lambda_dyn,
                             &beta_dyn);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.min_line_search_step_size = 1e-32;
  options.minimizer_type = ceres::LINE_SEARCH;
  options.line_search_type = ceres::WOLFE;
  // options.minimizer_type = ceres::TRUST_REGION;
  // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.linear_solver_type = ceres::DENSE_QR;
  // options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial P: " << P_start << "\n";
  std::cout << "Final   P: " << P_dyn << "\n";
  std::cout << "Initial Lambda: " << lambda_start << "\n";
  std::cout << "Final   Lambda: " << lambda_dyn << "\n";
  std::cout << "Initial Beta: " << beta_start << "\n";
  std::cout << "Final   Beta: " << beta_dyn << "\n";

  return 0;
}
