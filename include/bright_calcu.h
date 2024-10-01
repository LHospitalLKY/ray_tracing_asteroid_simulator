/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-04-26 19:33:28
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-10-01 13:54:52
 * @FilePath: /g2o_test/include/bright_calcu.h
 * @Description: 输入形状模型和观测条件, 计算亮度
 */

/*ADAM中, 计算亮度的函数输入为:
 * tlist: 面片列表
 * vlist: 顶点列表
 * numfac: 面数量
 * numvert: 顶点数量
 * angles: 自转中涉及到的角的参数, beta, lambda, 自转角速度, 初始自转相位角,
 * 单位均为deg Eo: 观测方向 E0o: 光源方向
 * 这些参数是前向计算目标亮度所必须的信息, 其实这里应该还有反射率参数
 */

#ifndef BRIGHT_CALCU_H

#include "common.h"

// #define DEBUG

/**
 * @description: 计算散射定律中的相函数, f(\alpha) = A0*exp(-\alpha/D) + k\alpha
 * + 1
 * @param {ViewVector} &obsView, 观测方向向量
 * @param {ViewVector} &sunView, 太阳方向向量
 * @param {double} A0
 * @param {double} D
 * @param {double} k
 * @return {double} 相函数值
 */
double phaseFunction(const ViewVector &obsView, const ViewVector &sunView,
                     double A0, double D, double k);

/**
 * @description: 计算LSL散射反射率, LSL = albdeo * mu * mu0 * (1.0/(mu + mu0)
 * +c)
 * @param {double} phase, 散射相函数值, phaseFunction算出来的结果
 * @param {FacetNormal} &normalVec, 当前面的法向量
 * @param {ViewVector} &obsView, 观测方向
 * @param {ViewVector} &sunView, 太阳方向
 * @param {double} c, LSL参数c
 * @param {double} albedo, 固有反照率
 * @return {*}
 */
double LSLScatter(double phase, const FacetNormal &normalVec,
                  const ViewVector &obsView, const ViewVector &sunView,
                  double c, double albedo = 1.0);

/**
 * @description: 通过方向判断面是否可见,
 * 当太阳方向或观测方向与法线的夹角小于90度时, 表示可见
 * @param {FacetNormal} &normalVec, 当前面的法向
 * @param {FacetNormal} &sunVec, 当前太阳或观测方向, 固联系
 * @return {bool} true表示可见, false表示不可见
 */
bool visiable_by_Vec(const FacetNormal &normalVec, const FacetNormal &sunVec);

/**
 * @description: 计算面片的可见性,
 * 通过面片的法向量和观测方向、太阳方向判断面片是否可见
 * @param {NormalList} &normalList, 面片法向量列表
 * @param {ViewVector} sunVec, 太阳方向, 固联系
 * @param {ViewVector} obsVec, 观测方向, 固联系
 * @param {VisiableList} &visiableList, 面片可见性列表
 * @return {*}
 */
void facetVisiable_calcu(const NormalList &normalList, const ViewVector sunVec,
                         const ViewVector obsVec, VisiableList &visiableList);

/**
 * @description: 计算面片的可见性
 * 采用两个判断准则来得到可见性列表:
 * 1. 通过面片的法向量和观测方向、太阳方向判断面片是否可见
 * 2. 计算当前方向下, 当前面片是否是太阳或观测者发出的, 所看到的第一个位置
 * @param {NormalList} &normalList, 面片法向量列表
 * @param {ViewVector} sunPos, 太阳位置, 单位是km
 * @param {ViewVector} obsPos, 观测者位置, 单位是km
 * @param {VisiableList} &visiableList, 可见性列表
 * @return {*}
 */
void facetVisiable_calcu_final(const NormalList &normalList,
                               const ViewVector sunPos, const ViewVector obsPos,
                               VisiableList &visiableList);

/**
 * @description: 计算点p到三角形面片的距离
 * @param {VerticeList} &p, 待计算的点
 * @param {FacetIndex} &f, 待计算的三角形面片
 * @param {VerticeList} &verList, 所有顶点列表, 用于得到三角形的三个顶点
 * @return {*}
 */
void pointToTriangleDistance(const VerticeList &p, const FacetIndex &f, const VerticeList &verList);

/**
 * @description:
 * 根据输入的面片列表、顶点列表、时间列表、观测方向、太阳方向和散射参数 计算亮度
 * @return {BriList&} brightList, 亮度列表
 */
void calculate_light(const FacetList &facetList, const VerticeList &vertexList,
                     const TimeList &timeList, const VectorList &obsViewVector,
                     const VectorList &sunViewVector, BriList &brightList,
                     double beta, double lambda, double P, double phi_0,
                     const double *A0, const double *D, const double *k,
                     double c = 0.1, double albedo = 1.0, double time_0 = 0.0);

/**
 * @description: 归一化亮度
 * @param {BriList} &bright_list
 * @return {*}
 */
void normal_bright(BriList &bright_list);

/**
 * @description: 计算单个视角下的亮度, 无需重新计算面片法向量、面积、可见性
 * @param {NormalList} &normalList, 面片法向量列表
 * @param {AreaList} &areaList, 面片面积列表
 * @param {VisiableList} &visiableList, 面片可见性列表
 * @param {ViewVector} &obsView, 观测方向, 固联系统坐标系
 * @param {ViewVector} &sunView, 太阳方向, 固联系统坐标系
 * @param {double*} A0, 散射定律相函数参数A0指针, 指针为NULL时表示无需该参数
 * @param {double*} D, 散射定律相函数参数D, 指针为NULL时表示无需该参数
 * @param {double*} k, 散射定律相函数参数k, 指针为NULL时表示无需该参数
 * @param {double} c, LSL参数c, 默认值为0.1
 * @param {double} albedo, 固有反照率, 默认值为1.0
 * @return {*}
 */
double calculate_light_oneView(const NormalList &normalList,
                               const AreaList &areaList,
                               const VisiableList &visiableList,
                               const ViewVector &obsView,
                               const ViewVector &sunView, const double *A0,
                               const double *D, const double *k, double c = 0.1,
                               double albedo = 1.0);

/**
 * @description: 计算面片的法向量与面积
 * @param {FacetList} &facetList, 面片列表
 * @param {VerticeList} &vertexList, 顶点列表
 * @return {NormalList} &normalList, 面片法向量列表
 * @return {AreaList} &areaList, 面片面积列表
 */
void calculate_areas_normals(const FacetList &facetList,
                             const VerticeList &vertexList,
                             NormalList &normalList, AreaList &areaList);

/**
 * @description: 计算单个面片的法向量与面积
 * @return {*}
 */
void calculate_areas_normals_onepiece(const FacetIndex &facetIndex,
                                      const VerticeList &vertexList,
                                      FacetNormal &normalVec, double &area);

#endif // !BRIGHT_CALCU_H
