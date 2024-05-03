/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 11:12:10
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-05-03 17:39:43
 * @FilePath: /g2o_test/src/damit2obj.cpp
 * @Description: damit2obj.h的实现
 */

#include "../include/damit2obj.h"
#include <cstddef>
#include <fstream>

int main(int argc, const char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " input.damit output.obj" << std::endl;
        return 1;
    }

    damit2obj(argv[1], argv[2]);

    return 0;
}