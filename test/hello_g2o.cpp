/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-04-23 19:40:23
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-04-23 22:14:53
 * @FilePath: /g2o_test/hello_g2o.cpp
 * @Description: 最简单的最小二乘法
 * TODO: 写一个椭球自转周期拟合的例子
 */

#include <Eigen/Eigen>

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
#include <optimization_algorithm_property.h>

// 尝试拟合函数f = a * exp(-lambda * t) + b
// 待拟合参数a, b, lambda

// g2o的求解模块
G2O_USE_OPTIMIZATION_LIBRARY(dense);

// 定义图优化中的顶点, 在本例中, 优化的参数只有一个, 那就是函数的参数a, b, lambda
// BaseVertex模板类中, 第一个参数是当前的vertex中参数的数量, 第二个参数是用来表示这些参数的数据类型
class VertexParams : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    // 使Eigen能够用在模板中
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexParams() {}

    bool read(std::istream& /*is*/) override { return false; }

    bool write(std::ostream& /*os*/) const override { return false; }

    void setToOriginImpl() override {}

    // 这一步用于更新估计值, update是更新量, _estimate是上一步的估计值
    void oplusImpl(const double* update) override {
        Eigen::Vector3d::ConstMapType v(update);
        // 注意这里_estimate更新方式不要写错
        _estimate += v;
    }
};

// 定义边, 边是用来计算误差的, 在本例中, 误差是函数值和观测值之间的差值
// BaseUnaryEdge是单边, 这个边只有一个顶点, 模板类中, 
// 第一个参数是1, 应该指的是残差的维度, 
// 第二个参数是measurement的类型, 这里选择measurement是为了同时保存自变量和因变量
// 第三个参数是顶点的类型, 这里是VertexParams, 是上文定义的
class EdgePointOnCurve : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgePointOnCurve() {}

    bool read(std::istream& /*is*/) override {
        G2O_ERROR("not implemented yet");
        return false;
    }
    bool write(std::ostream& /*os*/) const override {
        G2O_ERROR("not implemented yet");
        return false;
    }

    // 误差计算函数
    // 这里定义"()"操作是因为调用了自动求导, 自动求导的时候会调用这个函数
    template<typename T>
    bool operator()  (const T* params, T* error) const {
        const T& a = params[0];
        const T& b = params[1];
        const T& lambda = params[2];
        T fval = a * exp(-lambda * T(measurement()(0))) + b;
        // 注意这里error的计算不要写错
        error[0] = fval - measurement()(1);

        return true;
    }

    G2O_MAKE_AUTO_AD_FUNCTIONS;             // 自动求导
};


int main(int argc, const char** argv) {
    // 1. 生成数据
    int numPoints = 50;         // 样本数量
    int maxIterations = 100;    // 最大迭代次数
    double a = 2;               // 参数a
    double b = 0.4;             // 参数b
    double lambda = 0.2;        // 参数lambda
    double sigma = 0.02;        // 噪声标准差

    Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];
    g2o::Sampler::seedRand();
    for (size_t i  = 0; i < numPoints; i++) {
        double t = g2o::Sampler::uniformRand(0, 10);
        double y = a * exp(-t * lambda) + b;
        // std::cout << t <<", " << y << std::endl;
        
        // 为因变量添加噪声
        y = g2o::Sampler::gaussRand(y, sigma);
        points[i] << t, y;
    }


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

    // 添加顶点
    VertexParams *params = new VertexParams();
    params -> setId(0);
    params -> setEstimate(Eigen::Vector3d(1, 1, 1));
    optimizer.addVertex(params);

    // 添加边
    for (size_t i = 0; i < numPoints; i++) {
        EdgePointOnCurve *e = new EdgePointOnCurve();
        // 设置信息矩阵, 这里的信息矩阵指的是什么来着？
        e -> setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        // 设置当前边的顶点
        e -> setVertex(0, params);
        // 设置测量值
        e -> setMeasurement(points[i]);
        optimizer.addEdge(e);
    }

    // 启动优化器
    optimizer.initializeOptimization();         // 初始化
    optimizer.setVerbose(true);
    optimizer.optimize(maxIterations);         // 迭代优化

    // 输出最终结果
    std::cout << "Target curve" << std::endl;
    std::cout << "a * exp(-lambda * x) + b" << std::endl;
    std::cout << "Iterative least squares solution" << std::endl;
    std::cout << "a      = " << params->estimate()(0) << std::endl;
    std::cout << "b      = " << params->estimate()(1) << std::endl;
    std::cout << "lambda = " << params->estimate()(2) << std::endl;
    std::cout << std::endl;

    delete [] points;

    return 0;
}
