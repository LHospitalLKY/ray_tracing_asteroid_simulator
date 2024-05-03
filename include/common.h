/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 16:17:42
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-05-03 16:38:37
 * @FilePath: /g2o_test/include/common/common.h
 * @Description: 一些公用的数据类型与函数
 */

#include <iostream>
#include <vector>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <g2o/stuff/macros.h>

/**
 * @description: 分割字符串
 * @return {*}
 */
void Stringsplit(std::string str, const char &split,
                 std::vector<std::string> &res);

// 光变计算中所需要的数据结构
typedef std::vector<int> FacetIndex;        // 面片顶点索引结构
typedef std::vector<double> VertexPosition; // 顶点坐标结构, 顺序x, y, z
typedef std::vector<double> FacetNormal; // 面片法向结构, 顺序nx, ny, nz
typedef std::vector<double>
    TimeList; // 时间列表, 时间格式为时间戳或JD时间, 单位为hour
typedef std::vector<double> BriList; // 亮度列表
typedef std::vector<double> ViewVector; // 观测与光源方向向量, 顺序为x, y, z

typedef std::vector<FacetIndex> FacetList;       // 面片列表
typedef std::vector<VertexPosition> VerticeList; // 顶点列表
typedef std::vector<FacetNormal> NormalList;     // 面片法向列表
typedef std::vector<ViewVector> VectorList;      // 方向向量列表
typedef std::vector<double> AreaList;            // 面片面积列表
typedef std::vector<bool> VisiableList;          // 面片可见性列表

typedef std::vector<std::vector<double>> RotMatrix; // 旋转矩阵

/**
 * @description: 计算观测相角
 * @param {ViewVector} &obsView, 观测方向向量
 * @param {ViewVector} &sunView, 太阳方向向量
 * @return {*}
 */
double phaseAngle(const ViewVector &obsView, const ViewVector &sunView);

// 两个向量的叉乘, double[3]格式
void cross(double *A, double *B, double *C);

// 计算向量的模
double norm(double *A);

/**
 * @description: 根据自转参数, 得到基准系(一般是ECLIPJ2000)到固联系的姿态转换
 * @param {double} beta, 自转轴指向beta, deg, damit中的beta直接输入
 * @param {double} lambda, 自转轴指向lambda, deg, damit中的lambda直接输入
 * @param {double} omega, 自转角周期, hour, damit中的P直接输入
 * @param {double} omega0, 初始自转相位, deg, damit中的phi_0直接输入
 * @param {double} t, 距离初始时刻的时间, hour
 * @return {Matrix3d} &R, 旋转矩阵
 */
void rotate(double beta, double lambda, double omega, double omega0, double t,
            Eigen::Matrix3d &R);

/**
 * @description: 读取damit光变文件
 * @param {string} filename, 文件名
 * @param {TimeList} &jd_list, 时间列表
 * @param {BriList} &bri_list, 亮度列表
 * @param {VectorList} &sun_vec_list, 太阳方向向量列表
 * @param {VectorList} &obs_vec_list, 观测方向向量列表
 * @return {*} jd_list, bri_list, sun_vec_list, obs_vec_list
 */
void read_lcurve_damit(std::string filename, TimeList &jd_list, BriList &bri_list, VectorList &sun_vec_list, VectorList &obs_vec_list);

/**
 * @description: 将模型写入stl文件
 * @param {string} filename, 文件名
 * @param {FacetList} &facetList, 面片列表
 * @param {VerticeList} &verticeList, 顶点列表
 * @param {NormalList} &normalList, 面片法向量列表
 * @return {*}
 */
void write_shape_stl(std::string filename, const FacetList &facetList,
                     const VerticeList &verticeList,
                     const NormalList &normalList);

/**
 * @description: 读取damit格式的文件, 这类型的文件中, 每个面包含且仅包含3个顶点
 * @param {string} filename, 文件名
 * @param {FacetList} &facetList, 面片列表
 * @param {VerticeList} &verticeList, 顶点列表
 * @param {int} type3, 只能为3, 表示一个面中有多少顶点
 * @return {*}
 */
void read_shape(std::string filename, FacetList &facetList,
                VerticeList &verticeList, int type3);