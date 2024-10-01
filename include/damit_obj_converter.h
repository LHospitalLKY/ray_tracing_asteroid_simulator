/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 11:12:03
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-10-01 23:39:08
 * @FilePath: /g2o_test/include/damit_obj_converter.h
 * @Description: damit与obj的形状文件相互转换
 */

#include "common.h"
// #include "obj_loader.h"

#include <tiny_obj_loader.h>

// #define DEBUG

void damit2obj(const std::string &input, const std::string &output);

void obj2damit(const std::string &input, const std::string &output);