/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 17:42:26
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-05-05 10:14:54
 * @FilePath: /g2o_test/test/period_scan.cpp
 * @Description: 基于g2o的自转周期寻找
 */

#include "../include/common.h"
#include "../include/bright_calcu.h"

#include <eigen3/Eigen/Eigen>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Memory.h>
#include <cstddef>
#include <g2o/core/auto_differentiation.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/sampler.h>

#include <iostream>
#include <math.h>
#include <optimization_algorithm_levenberg.h>
#include <optimization_algorithm_property.h>
#include <tuple>

// g2o的求解模块
G2O_USE_OPTIMIZATION_LIBRARY(dense);

// 图优化中的顶点, 待优化的只有自转周期period
class VertexParams : public g2o::BaseVertex<1, Eigen::Vector<double, 1>> {
public:
    VertexParams() {}

    bool read(std::istream& /*is*/) override { return false; }

    bool write(std::ostream& /*os*/) const override { return false; }

    void setToOriginImpl() override {}

    // 这一步用于更新估计值, update是更新量, _estimate是上一步的估计值
    void oplusImpl(const double* update) override {
        Eigen::Vector<double, 1>::ConstMapType v(update);
        // 注意这里_estimate更新方式不要写错
        _estimate += v;
    }
};

// TODO:
// 形状, 原始光变亮度, 自转轴参数, 散射参数等需要先写到这个边里, 只写单个采样点的数据, 而不是
class EdgePointOnCurve : public g2o::BaseUnaryEdge<1, double, VertexParams> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgePointOnCurve(const FacetList *facetList, const VerticeList *verticeList, const NormalList *normalList, const AreaList *areaList, const ViewVector *obsVec, const ViewVector *sunVec, double *A0, double *D, double *k, double c, double albedo, double beta, double lambda, double phi_0, double t0, double t, bool is_rel = true, double ave_bri = 1.0) {
        // 初始化
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
        _beta = beta;
        _lambda = lambda;
        _phi_0 = phi_0;
        _t0 = t0;
        _t = t;
        _isRelative = is_rel;
        
        if (_isRelative == true) {
            _ave_bri_allSample = ave_bri;
        } else {
            _ave_bri_allSample = 1.0;
        }
    }

    bool read(std::istream& /*is*/) override {
        G2O_ERROR("not implemented yet");
        return false;
    }
    bool write(std::ostream& /*os*/) const override {
        G2O_ERROR("not implemented yet");
        return false;
    }

    void computeError() override {
        // 1. 计算亮度
        const VertexParams *v = static_cast<const VertexParams *>(_vertices[0]);
        double P_estimate = v -> estimate()(0);
        double bright_estimate = this -> calcuBright(P_estimate);
        // std::cout << P_estimate << std::endl;
        // std::cout << bright_estimate << std::endl;
        // std::cout << _measurement << std::endl;
        // std::cout << bright_estimate - _measurement << "," << std::endl;
        // 2. 计算误差
        _error(0) = bright_estimate - _measurement;
        // 打印雅可比矩阵
        // std::cout << _jacobianOplus << std::endl;
        // std::cout << std::get<0>(_jacobianOplus) << std::endl;;
        // std::cout << _jacobianOplusXi.any() << std::endl;
        // std::cout << _jacobianOplusXi[0] << std::endl;
        
    }

    void linearizeOplus() override {
        // 1. 计算左右亮度
        const VertexParams *v = static_cast<const VertexParams *>(_vertices[0]);
        double P_estimate = v -> estimate()(0);
        // double bright_estimate = this -> calcuBright(P_estimate);
        // 2. 计算雅可比矩阵
        // std::cout << "bright_estimate = " << bright_estimate << std::endl;
        // std::cout << "_measurement = " << _measurement << std::endl;
        // std::cout << "P_estimate = " << P_estimate << std::endl;
        std::cout << "this -> calcuBright(P_estimate + 1e-3) = " << this -> calcuBright(P_estimate + 1e-3) << std::endl;
        std::cout << "this -> calcuBright(P_estimate - 1e-3) = " << this -> calcuBright(P_estimate - 1e-3) << std::endl;
        // std::cout << "this -> calcuBright(P_estimate + 1e-6) - this -> calcuBright(P_estimate - 1e-6) = " << this -> calcuBright(P_estimate + 1e-6) - this -> calcuBright(P_estimate - 1e-6) << std::endl;
        // std::cout << "1e-6 = " << 1e-6 << std::endl;
        // std::cout << "bright_estimate - _measurement = " << bright_estimate - _measurement << std::endl;

        double jacobian = (this -> calcuBright(P_estimate + 1e-3) - this -> calcuBright(P_estimate - 1e-3)) / 2e-3;

        std::cout << "jacobian value: " << jacobian << std::endl;

        // std::cout << "1e-6 * _error(0) = " << 1e-6 * _error(0) << std::endl;
        // std::cout << "1e-6 * _error(0) / (bright_estimate - _measurement) = " << 1e-6 * _error(0) / (bright_estimate - _measurement) << std::endl;
        // std::cout << "1e-6 * _error(0) / (bright_estimate - _measurement) / 1e-6 = " << 1e-6 * _error(0) / (bright_estimate - _measurement) / 1e-6 << std::endl;
        // std::cout << "1e-6 *

        _jacobianOplusXi[0] = jacobian;
        std::cout << _jacobianOplusXi[0] << std::endl;
        // std::cout << _jacobianOplusXi[1] << std::endl;

        return;
    }
    

    // 误差计算函数
    // 这里定义"()"操作是因为调用了自动求导, 自动求导的时候会调用这个函数
    // template<typename T>
    // bool operator()  (const T* params, T* error) const {
    //     // std::cout << "typeid(T) = " << typeid(T).name() << std::endl;
    //     // typename g2o::VectorN<1, T>::ConstMaptType v(params);
    //     // Eigen::Vector<double, 1>::ConstMapType v(params);
    //     const T& period = params[0];
    //     // double period = params[0];
    //     // 计算出亮度
    //     const VertexParams *v = static_cast<const VertexParams *>(_vertices[0]);
    //     double P_estimate = v -> estimate()(0);
    //     double bright_estimate = this -> calcuBright(P_estimate);
        
    //     // T fval = a * exp(-lambda * T(measurement()(0))) + b;
    //     // 注意这里error的计算不要写错
    //     error[0] = bright_estimate - T(measurement());

    //     return true;
    // }

    // G2O_MAKE_AUTO_AD_FUNCTIONS;             // 自动求导

public:
    // 计算亮度
    double calcuBright(double P) const {
        // std::cout << "Called calcuBright: P " << P << std::endl;
        // 1. 坐标转换
        // 旋转矩阵
        Eigen::Matrix3d R;

        // std::cout << (*_obsVector)[0] << " " << (*_obsVector)[1] << " " << (*_obsVector)[2] << std::endl;
        // std::cout << (*_sunVector)[0] << " " << (*_sunVector)[1] << " " << (*_sunVector)[2] << std::endl;
        // double omega = 2 * M_PI / 8.40059506;
        // rotate(_beta, _lambda, measurement(), _phi_0, _t - _t0, R);

        // rotate(_beta, _lambda, 8.40059506, _phi_0, _t - _t0, R);
        rotate(_beta, _lambda, P, _phi_0, _t - _t0, R);

        // std::cout << "Rotation" << std::endl;
        // std::cout << R << std::endl;

        // 旋转观测向量
        Eigen::Vector3d obsVec_iau_eigen{(*_obsVector)[0], (*_obsVector)[1], (*_obsVector)[2]};
        Eigen::Vector3d sunVec_iau_eigen{(*_sunVector)[0],(*_sunVector)[1], (*_sunVector)[2]};
        obsVec_iau_eigen = R * obsVec_iau_eigen;
        sunVec_iau_eigen = R * sunVec_iau_eigen;
        ViewVector obsVec_iau{obsVec_iau_eigen[0], obsVec_iau_eigen[1], obsVec_iau_eigen[2]};
        ViewVector sunVec_iau{sunVec_iau_eigen[0], sunVec_iau_eigen[1], sunVec_iau_eigen[2]};

 #ifdef DEBUG       
        std::cout << "Obs IAU Vector:" << std::endl;
        std::cout << obsVec_iau[0] << " " << obsVec_iau[1] << " " << obsVec_iau[2] << std::endl;
        std::cout << "Sun IAU Vector:" << std::endl;
        std::cout << sunVec_iau[0] << " " << sunVec_iau[1] << " " << sunVec_iau[2] << std::endl;
#endif

        // 2. 计算亮度
        // 首先计算面的可见性
        VisiableList visList;
        facetVisiable_calcu(*_normalList, sunVec_iau, obsVec_iau, visList);

#ifdef DEBUG
        for (size_t i = 0; i < visList.size(); i++) {
            if (visList[i]) {
            std::cout << "Facet " << i << " is visiable" << std::endl;
            std::cout << "Normal " << (*_normalList)[i][0] << " " << (*_normalList)[i][1] <<" " << (*_normalList)[i][2] << std::endl;
            std::cout << "Area " << (*_areaList)[i] << std::endl;
            }
        }
#endif
        
        double bright = calculate_light_oneView(*_normalList, *_areaList, visList, obsVec_iau, sunVec_iau, _A0, _D, _k);

        if (_isRelative == true) {
            bright = bright / _ave_bri_allSample;
            // bright = bright / _allArea;
        }
        // std::cout << bright << std::endl;
        return bright;
    }

private:
    // 形状相关变量
    const FacetList *_facetList;            // 面片列表
    const VerticeList *_verticeList;        // 顶点列表
    const NormalList *_normalList;          // 法向量列表
    const AreaList *_areaList;              // 面积列表
    // double _allArea;                        // 可见面面积和

    // 自转状态相关变量
    double _beta;                           // 自转轴指向beta
    double _lambda;                         // 自转轴指向lambda
    double _phi_0;                          // 初始自转相位
    double _t0;                             // 初始时刻
    double _t;                              // 观测时刻

    // 观测几何
    const ViewVector *_obsVector;           // 观测方向向量, 惯性系
    const ViewVector *_sunVector;           // 太阳方向向量, 惯性系

    // 散射参数
    double *_A0 = NULL;                     // 散射相函数参数A0
    double *_D = NULL;                      // 散射相函数参数D
    double *_k = NULL;                      // 散射相函数参数k

    double _c = 0.1;                        // LSL参数c
    double _albedo = 1.0;                   // 反照率

    // 是否为相对亮度
    bool _isRelative = true;
    double _ave_bri_allSample = 1.0;           // 所有样本的平均亮度, 相对亮度模式下使用

};

int main(int argc, const char** argv) {
    // 首先测试一下是不是对的
    // 测试边的参数
    // double jd_0 = 2438882.233280;
    // double beta = 58.34;
    // double lambda = 274.86;
    // double P = 8.40059506;
    // double P_start = 9.0;
    // double phi_0 = 0.0;
    // double A0 = 0.5;
    // double D = 0.1;
    // double k = -0.5;
    // double c = 0.1;
    // double albedo = 1.0;
    double jd_0 = 2443845.852388;
    double beta = -15.;
    double lambda = 253.;
    double P = 5.76198;
    double P_start = 4.0;
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
    read_shape("/home/lho/MyProgramm/2024.04/g2o_test/test_data/ariadne_shape.txt",
             facetList, verticeList, 1);
    calculate_areas_normals(facetList, verticeList, normalList, areaList);

#ifdef DEBUG
    for (size_t i = 0; i < facetList.size(); i++) {
        std::cout << "Facet " << i << "Information" << std::endl;
        std::cout << "Facet " << facetList[i][0] << " " << facetList[i][1] << " " << facetList[i][2] << std::endl;
        std::cout << "Normal " << normalList[i][0] << " " << normalList[i][1] << " " << normalList[i][2] << std::endl;
        std::cout << "Area " << areaList[i] << std::endl;
    }
#endif

    // 读取观测数据
    VectorList sun_vec_list, obs_vec_list;
    TimeList jd_time_list;
    BriList bri_list;
    read_lcurve_damit("/home/lho/MyProgramm/2024.04/g2o_test/test_data/ariadne.lc",
                        jd_time_list, bri_list, sun_vec_list, obs_vec_list);
    for (size_t i = 0; i < bri_list.size(); i++) {
        bri_list[i] = bri_list[i] * 8.679929627312347;
    }

    BriList bri_list_for_normalize;         // 如果是rel亮度, 需要先计算一个总亮度, 根据总亮度归一化
    calculate_light(facetList, verticeList, jd_time_list, obs_vec_list, sun_vec_list, bri_list_for_normalize, beta, lambda, P, phi_0, &A0, &D, &k, 0.1, 1.0, jd_0);
    double ave_bright = 0.0;
    for (size_t i = 0; i < bri_list_for_normalize.size(); i++) {
        ave_bright += bri_list_for_normalize[i];
    }
    ave_bright = ave_bright/bri_list_for_normalize.size();
    std::cout << "Ave Bright " << ave_bright << std::endl;

    // 构建优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // 设置优化器的优化方法
    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(
        g2o::OptimizationAlgorithmFactory::instance()->construct(
            "lm_dense", 
            solverProperty
        )
    );

    // 添加边
    VertexParams *params = new VertexParams();
    params -> setId(0);
    params -> setEstimate(Eigen::Vector<double, 1>(P_start));
    optimizer.addVertex(params);

    // 添加边
    BriList bri_list_g2o;
    for (size_t i = 0; i < jd_time_list.size(); i++) {
        // 读取观测数据
        double jd = jd_time_list[i];
        ViewVector &obs = obs_vec_list[i];
        ViewVector &sun = sun_vec_list[i];

        // 创建边
        EdgePointOnCurve *edge = new EdgePointOnCurve(&facetList, &verticeList, &normalList, &areaList, &obs, &sun, &A0, &D, &k, c, albedo, beta, lambda, phi_0, jd_0, jd, false);
        // edge -> setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        edge -> setVertex(0, params);
        edge -> setMeasurement(bri_list[i]);
        // edge -> computeError();
        optimizer.addEdge(edge);
        // edge->setMeasurement(t);
        // edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        // edge->setVertex(0, new VertexParams(P));

        // 计算亮度
        // double bright = edge->calcuBright(4.0);
        // bri_list_g2o.push_back(bright);
    }


    // for (double bri : bri_list_g2o) {
    //     std::cout << bri << ", ";
    // }
    // std::cout << std::endl;
    
    // 启动优化器
    optimizer.initializeOptimization();         // 初始化
    optimizer.setVerbose(true);
    // optimizer.jacobianWorkspace();
    optimizer.optimize(100);         // 迭代优化
    // for (int i = 0; i < 100; i++) {
    //     optimizer.optimize(1);
    //     std::cout << "P = " << params->estimate()(0) << std::endl;
    //     optimizer.jacobianWorkspace().workspaceForVertex(i);
    // }
    // optimizer.solver()

    // 输出最终结果
    std::cout << "Target parameters" << std::endl;
    std::cout << "P" << std::endl;
    std::cout << "Iterative least squares solution" << std::endl;
    std::cout << "P      = " << params->estimate()(0) << std::endl;
    std::cout << std::endl;

    return 0;
}
