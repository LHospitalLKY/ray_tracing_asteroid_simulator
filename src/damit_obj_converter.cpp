/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 11:12:10
 * @LastEditors: LHospitalLKY 1141165506@qq.com
 * @LastEditTime: 2024-10-02 15:54:21
 * @FilePath: /g2o_test/src/damit2obj.cpp
 * @Description: damit2obj.h的实现
 */

#include "../include/damit_obj_converter.h"
#include "../include/obj_loader.h"
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
    try
    {
        ofs << "####\n#\n# OBJ File Generated from DAMIT\n#\n####\n#\n# Vertices: 1021\n# Faces: 2038\n#\n####" << std::endl;
    }
    catch (const std::exception&)
    {
        std::cout << "Write OBJ model file header failed, please check the program" << std::endl;
        ofs.close();
        return;
    }
    

    // 顶点
    try
    {
        for (size_t i = 0; i < verticeList.size(); i++) {
            ofs << "v " << verticeList[i][0] << " ";
            ofs << verticeList[i][1] << " ";
            ofs << verticeList[i][2] << std::endl;
        }
    }
    catch (const std::exception&)
    {
        std::cout << "Write OBJ model file vertices failed, please check the program" << std::endl;
        ofs.close();
        return;
    }

    // 面片
    try
    {
        for (size_t i = 0; i < facetList.size(); i++) {
            ofs << "f ";
            for (size_t j = 0; j < facetList[i].size(); j++) {
                ofs << facetList[i][j] << " ";
            }
            ofs << std::endl;
        }
    }
    catch (const std::exception&)
    {
        std::cout << "Write OBJ model file facets failed, please check the program" << std::endl;
        ofs.close();
        return;
    }

    ofs.close();
}

void obj2damit(const std::string &input, const std::string &output) {
    // 读取obj模型
    OBJLoader loader;
    loader.load(input);
    
    int vertices_num = loader.getNumVertex();
    int facets_num = loader.getNumFacets();

    const VerticeList *v_list = loader.getVertices();
    const FacetList *f_list = loader.getFacets();

    std::cout << "Num of Vertices: " << vertices_num << std::endl;
    std::cout << "Num of Facets: " << facets_num << std::endl;

    std::cout << "Num of Vertices: " << v_list -> size() << std::endl;
    std::cout << "Num of Facets: " << f_list -> size() << std::endl;

    std::ofstream ofs;
    ofs.open(output);

    // 文件头
    try
    {
        ofs << vertices_num << " " << facets_num << std::endl;
    }
    catch (const std::exception&)
    {
        std::cout << "Write DAMIT model file header failed, please check the program" << std::endl;
        ofs.close();
        return;
    }

    // 遍历点
    try
    {
        for (size_t i = 0; i < vertices_num; i++) {
            ofs << (*v_list)[i][0] << " ";
            ofs << (*v_list)[i][1] << " ";
            ofs << (*v_list)[i][2] << std::endl;
        }
    }
    catch (const std::exception&)
    {
        std::cout << "Write DAMIT model file vertices failed, please check the program" << std::endl;
        ofs.close();
        return;
    }

    // 遍历面
    try
    {
        for (size_t i = 0; i < facets_num; i++) {
            ofs << (*f_list)[i][0] + 1 << " ";
            ofs << (*f_list)[i][1] + 1 << " ";
            ofs << (*f_list)[i][2] + 1 << std::endl;
        }
    }
    catch (const std::exception&)
    {
        std::cout << "Write DAMIT model file facets failed, please check the program" << std::endl;
        ofs.close();
        return;
    }

    ofs.close();
}
