/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 11:12:10
 * @LastEditors: LHospitalLKY 1141165506@qq.com
 * @LastEditTime: 2024-10-02 15:48:45
 * @FilePath: /g2o_test/test/damit_obj_converter.cpp
 * @Description: damit_obj_converter.h的实现
 */

#include "../include/damit_obj_converter.h"
#include <cstddef>
#include <fstream>

int main(int argc, const char** argv) {
    // if (argc != 3) {
    //     std::cerr << "Usage: " << argv[0] << " input.damit output.obj" << std::endl;
    //     return 1;
    // }

    damit2obj("/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/outshape.txt", "/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/outshape_test.obj");

    obj2damit("/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/monkey.obj", "/home/lho/MyProgramm/2024.04/ray_tracing_asteroid_simulator/test_data/monkey.txt");



    return 0;
}