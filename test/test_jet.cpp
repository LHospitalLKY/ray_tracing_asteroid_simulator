/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-15 15:01:20
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-05-15 21:37:24
 * @FilePath: /g2o_test/test/test_jet.cpp
 * @Description: 测试使用jet来计算亮度
 */

#include <Eigen/Eigen>
#include <ceres/jet.h>
#include "../include/bright_calcu_jet.h"

struct Facet {
    ceres::Jet<double, 3> _p1[3];
    ceres::Jet<double, 3> _p2[3];
    ceres::Jet<double, 3> _p3[3];

    Facet(const std::vector<ceres::Jet<double, 3>> &p1, const std::vector<ceres::Jet<double, 3>> &p2, const std::vector<ceres::Jet<double, 3>> &p3) {
        _p1[0] = p1[0];
        _p1[1] = p1[1];
        _p1[2] = p1[2];

        _p2[0] = p2[0];
        _p2[1] = p2[1];
        _p2[2] = p2[2];

        _p3[0] = p3[0];
        _p3[1] = p3[1];
        _p3[2] = p3[2];
    }
};

template <typename T>
T f(const T &x, const T &y) {
    return x * x + x * y;
}

std::vector<ceres::Jet<double, 3>> cross(const ceres::Jet<double, 3> *a, const ceres::Jet<double, 3> *b) {
    std::vector<ceres::Jet<double, 3>> res;
    res.push_back(a[1] * b[2] - a[2] * b[1]);
    res.push_back(a[2] * b[0] - a[0] * b[2]);
    res.push_back(a[0] * b[1] - a[1] * b[0]);
    return res;
}

// 面积计算
ceres::Jet<double, 3> area(const Facet &facet) {
    // 首先得到边向量
    ceres::Jet<double, 3> V1[3];
    ceres::Jet<double, 3> v1_x = facet._p2[0] - facet._p1[0];
    ceres::Jet<double, 3> v1_y = facet._p2[1] - facet._p1[1];
    ceres::Jet<double, 3> v1_z = facet._p2[2] - facet._p1[2];
    V1[0] = v1_x;
    V1[1] = v1_y;
    V1[2] = v1_z;

    ceres::Jet<double, 3> V2[3];
    ceres::Jet<double, 3> v2_x = facet._p3[0] - facet._p1[0];
    ceres::Jet<double, 3> v2_y = facet._p3[1] - facet._p1[1];
    ceres::Jet<double, 3> v2_z = facet._p3[2] - facet._p1[2];
    V2[0] = v2_x;
    V2[1] = v2_y;
    V2[2] = v2_z;

    std::cout << "V1: " << std::endl;
    std::cout << V1[0] << std::endl;
    std::cout << V1[1] << std::endl;
    std::cout << V1[2] << std::endl;

    std::cout << "V2: " << std::endl;
    std::cout << V2[0] << std::endl;
    std::cout << V2[1] << std::endl;
    std::cout << V2[2] << std::endl;
    // 叉乘
    std::vector<ceres::Jet<double, 3>> C = cross(V1, V2);

    std::cout << "C: " << std::endl;
    for (ceres::Jet<double, 3> c : C) {
        std::cout << c << std::endl;
    }

    ceres::Jet<double, 3> area = 0.5 * sqrt(C[0] * C[0] + C[1] * C[1] + C[2] * C[2]);

    std::cout << "Area: " << std::endl;
    std::cout << area << std::endl;

    return area;
}

int main(int argc, const char** argv) {
    ceres::Jet<double, 2> x(1, 0);
    ceres::Jet<double, 2> y(3, 1);

    ceres::Jet<double, 2> z = f(x, y);

    std::cout << "z: " << z << std::endl;
    std::cout << "df/dx: " << z.v[0] << std::endl;
    std::cout << "df/dy: " << z.v[1] << std::endl;

    double p0[3] = {0., 0., 0.};
    double p1[3] = {1., 0., 0.};
    double p2[3] = {0., 1., 0.};

    std::vector<ceres::Jet<double, 3>> p0_jet;
    std::vector<ceres::Jet<double, 3>> p1_jet;
    std::vector<ceres::Jet<double, 3>> p2_jet;
    p0_jet.push_back(ceres::Jet<double, 3>(p0[0], 0));
    p0_jet.push_back(ceres::Jet<double, 3>(p0[1], 1));
    p0_jet.push_back(ceres::Jet<double, 3>(p0[2], 2));
    p1_jet.push_back(ceres::Jet<double, 3>(p1[0], 0));
    p1_jet.push_back(ceres::Jet<double, 3>(p1[1], 1));
    p1_jet.push_back(ceres::Jet<double, 3>(p1[2], 2));
    p2_jet.push_back(ceres::Jet<double, 3>(p2[0], 0));
    p2_jet.push_back(ceres::Jet<double, 3>(p2[1], 1));
    p2_jet.push_back(ceres::Jet<double, 3>(p2[2], 2));

    Facet facet(p0_jet, p1_jet, p2_jet);

    area(facet);
    return -1;
}
