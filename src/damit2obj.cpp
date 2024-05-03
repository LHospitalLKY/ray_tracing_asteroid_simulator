/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 11:12:10
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-05-03 17:38:23
 * @FilePath: /g2o_test/src/damit2obj.cpp
 * @Description: damit2obj.h的实现
 */

#include "../include/damit2obj.h"
#include <cstddef>
#include <fstream>

void damit2obj(const std::string &input, const std::string &output) {
    // 读取damit文件
    FacetList facetList;
    VerticeList verticeList;
    read_shape(input, facetList, verticeList, 1);

    std::ofstream ofs;
    ofs.open(output);

    // 文件头
    ofs << "####\n#\n# OBJ File Generated from DAMIT\n#\n####\n#\n# Vertices: 1021\n# Faces: 2038\n#\n####" << std::endl;

    // 顶点
    for (size_t i = 0; i < verticeList.size(); i++) {
        ofs << "v " << verticeList[i][0] << " ";
        ofs << verticeList[i][1] << " ";
        ofs << verticeList[i][2] << std::endl;
    }
    // 面片
    for (size_t i = 0; i < facetList.size(); i++) {
        ofs << "f ";
        for (size_t j = 0; j < facetList[i].size(); j++) {
            ofs << facetList[i][j] << " ";
        }
        ofs << std::endl;
    }
}
