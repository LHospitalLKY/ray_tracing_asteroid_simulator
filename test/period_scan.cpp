/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 17:42:26
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-05-03 17:47:30
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
#include <optimization_algorithm_property.h>

// g2o的求解模块
G2O_USE_OPTIMIZATION_LIBRARY(dense);

int main(int argc, const char** argv) {
    return 0;
}
