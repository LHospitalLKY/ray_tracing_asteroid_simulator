/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-15 15:01:58
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-05-15 16:32:03
 * @FilePath: /g2o_test/include/bright_calcu_jet.h
 * @Description: 使用jet模板类来计算亮度
 */

#include <Eigen/src/Core/Matrix.h>
#include <iostream>

#include "common.h"
#include "ceres/jet.h"

// Jet模板类
// 三维向量
typedef ceres::Jet<Eigen::Vector3d, 3> Jet3d;
// 一维标量
typedef ceres::Jet<double, 1> Jet1d;

// 数据结构类
typedef std::vector<int> FacetIndex_jet;        // 面片顶点索引结构
typedef Jet3d VertexPosition_jet;               // 顶点结构, x, y, z
typedef Jet3d FacetNormal_jet;                  // 面片法向结构, nx, ny, nz

typedef std::vector<Jet1d> BriList_jet;         // 亮度列表
typedef Jet3d ViewVector_jet;                   // 观测与光源方向向量, 顺序为x, y, z

typedef std::vector<FacetIndex_jet> FacetList_jet;          // 面片列表
typedef std::vector<VertexPosition_jet> VerticeList_jet;    // 顶点列表
typedef std::vector<FacetNormal_jet> NormalList_jet;        // 面片法向列表
typedef std::vector<ViewVector_jet> VectorList_jet;         // 方向向量列表
typedef std::vector<Jet1d> AreaList_jet;                    // 面片面积列表
typedef std::vector<bool> VisiableList_jet;                 // 面片可见性列表

// 计算观测相角
double phaseAngle_jet(const ViewVector_jet &obsView, const ViewVector_jet &sunView) {
    Jet1d cos_alpha;
    // 
}

