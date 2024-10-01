/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-08-12 22:05:12
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-08-14 21:51:30
 * @FilePath: /g2o_test/include/AO_calcu.h
 * @Description: AO图像处理
 */


#include <cstddef>
#ifndef AO_CALCU_H

#include "common.h"
#include <valarray>
#include <CCfits/CCfits>
#include <Kissfft/tools/kiss_fftndr.h>
#include "Kissfft/tools/kiss_fft.h"

// AO中所需要的数据结构
typedef struct AO {
    // 单幅AO图像的数据结构
    ViewVector _sunDirec;             // 太阳光方向向量
    ViewVector _viewDirec;            // 观测方向向量
    double _time;                     // 观测时间
    double _distant;                  // 距离(但还不知道是什么距离)

    double _pixelScale_x;             // x方向像素尺寸
    double _pixelScale_y;             // y方向像素尺寸
    ViewVector _up;                   // 仪器方向(要看看这个方向指的是什么)

    int _naxis1;                      // 图像尺寸axis_1
    int _naxis2;                      // 图像尺寸axis_2
    double *_img = nullptr;           // 图像数据序列
}AoStruct;


/**
 * @description: 从fits文件中读取
 * @param {string} FITS_file
 * @param {AoStruct} &ao
 * @return {*}
 */
void readImgFromFITS(const std::string &FITS_file, AoStruct &ao);

/**
 * @description: 将ccfits读取的valarray转换为数组
 * @param {std::valarray<T>&} ccfits读取出的数据引用
 * @param {T*} data_c 转存位置的指针
 * @return {*}
 */
template <typename T>
void ccfits2array(const std::valarray<T> &data, T *data_c, int width, int height);

// TODO: 装好OpenCV库, 尝试转成OpenCV来处理
// template <typename T>
// void ccfits2cvMat(const std::valarray<T> &data, cv::Mat *)

// TODO: 完成PGM图像保存的函数
// void savePGM(const )

void calcImgFFT(
    const AoStruct &ao,                         // AO图像结构体, 包含了图像数据
    double naxis1_scale, double naxis2_scale,   // 图像尺度, 单位
    double *zMr, double *zMi, 
    double *Fx, double *Fy                      // 输出结果, 包含了频谱数据
);

void calc_image_fft(
    const AoStruct &ao,
    double x_scale, double y_scale);

#endif // !AO_CALCU_H




