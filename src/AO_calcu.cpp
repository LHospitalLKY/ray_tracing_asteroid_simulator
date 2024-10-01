/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-08-12 22:23:46
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-08-14 22:14:24
 * @FilePath: /g2o_test/src/AO_calcu.cpp
 * @Description: AO_calcu.h的实现
 */

// TODO: 用别的方法读一下AO图像, 与这里读出来的进行以下对比
// TODO: AO图像需要保证横纵分辨率偶数, 否则会导致FFT计算出错
// TODO: 对比一下ADAM程序计算结果与自己写的程序的计算结果

#include "../include/AO_calcu.h"
#include "Kissfft/tools/kiss_fft.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <memory>
#include <new>
#include <fstream>
#include <valarray>

void readImgFromFITS(const std::string &FITS_file, AoStruct &ao) {
    // 设定读取的keyword
    const std::string NAXIS1 = "NAXIS1";
    const std::string NAXIS2 = "NAXIS2";
    const std::string BITPIX = "BITPIX";
    const std::string DATE_OBS = "MJD-OBS";

    // read fits file header
    std::unique_ptr<CCfits::FITS> pInfile(new CCfits::FITS(FITS_file, CCfits::Read, true));

    CCfits::PHDU& fit = pInfile->pHDU();
    int bitpix = 0;
    int naxis1 = 0, naxis2 = 0;
    double JD = -INFINITY;
    try {
        fit.readKey(BITPIX, bitpix);
    } catch(...) {
        std::cout << "FITS file has no keyword " << BITPIX << std::endl;
    }
    try {
        fit.readKey(NAXIS1, naxis1);
    } catch(...) {
        std::cout << "FITS file has no keyword " << NAXIS1 << std::endl;
    }
    try {
        fit.readKey(NAXIS2, naxis2);
    } catch(...) {
        std::cout << "FITS file has no keyword " << BITPIX << std::endl;
    }
    try {
        fit.readKey(DATE_OBS, JD);
    } catch(...) {
        std::cout << "FITS file has no keyword " << DATE_OBS << std::endl;
    }
    

    std::cout << "BITPIX: " << bitpix << std::endl;
    std::cout << "NAXIS1: " << naxis1 << std::endl;
    std::cout << "NAXIS2: " << naxis2 << std::endl;
    std::cout << "OBS-JD: " << JD <<std::endl;

    ao._naxis1 = naxis1;
    ao._naxis2 = naxis2;
    ao._time = JD;
    ao._img = new double[ao._naxis1 * ao._naxis2];

    // 读取图像数据
    std::valarray<double> data;
    fit.read(data);

    double img_array[naxis1*naxis2];
    ccfits2array<double>(data, ao._img, naxis1, naxis2);

    if (ao._naxis1 % 2 != 0) {
        ao._naxis1 =ao._naxis1 - 1;
    }
    if (ao._naxis2 % 2 != 0) {
        ao._naxis2 = ao._naxis2 - 1;
    }

    return;
}

template <typename T>
void ccfits2array(const std::valarray<T> &data, T *data_c, int width, int height) {
    assert(width * height == data.size());
    int x_size = width;
    int y_size = height;
    if (width % 2 != 0) {
        x_size = width - 1;
    }
    if (height % 2 != 0) {
        y_size = height - 1;
    }
    // int k = data.size();
    int k = 0;
    for (int i = 0; i < width; i++) {
        if (i < x_size) {
            for (int j = 0; j < height; j++) {
                if (j < y_size) {
                    data_c[k] = data[i*height + j];
                    k++;
                }
            }
        }
    }

}

void calc_image_fft(double *M,int m,int n,double dx,double dy,double *zMr,double *zMi,double *Fx,double *Fy)
{
    /*M image matrix mxn, m and n EVEN. Horizontal is x direction, vertical y
     * Zero coordinate at (m/2+1,n/2+1)
     * Lx=n*dx, Ly=m*dy
     * Returns 2d fft transform of M,m x (n/2) matrix, with reals in in zMr and imaginary part in zMi 
     * corresponding frequencies in x-direction
     * 0,1/Lx,2/Lx,...(n/2)*1/Lx
     * and in y direction
     * 0,1/Ly,..,(m/2-1)*1/Ly,-m/2*Ly,...,-1/Ly
     * So first n/2 entries in zMr+i*zMi correspond to spatial frequencies fx=0,1/Lx,...(n/2)*1/Lx and fy=0
     * next n/2 entries correspond to fx=0,1/Lx,...(n/2)*1/Lx and fy=1/Ly
     * NOTE: We normalize with DC term and then drop it*/
    /*Output: zMr,zMi m*(n/2+1) arrays, Fx,Fy m*(n/2+1) arrays. The kth term of zMr+zMi corresponds to frequency Fx(k),Fy(k). */
    /*TBD: PSF!!*/  
    const int dim[2]={m,n};
    const int dimcount=2;
    double dc;
    double Lx,Ly;
    Lx=n*dx;
    Ly=m*dy;
    kiss_fft_cpx *fft2_out;
    // fft2_out=malloc(dim[0]*dim[1]*sizeof(kiss_fft_cpx));
    fft2_out = new kiss_fft_cpx[m*n];
    kiss_fftndr_cfg stf = kiss_fftndr_alloc(dim, dimcount, 0, 0, 0);
    kiss_fftndr(stf, M, fft2_out);
    /*fft2_out is mx(n/2+1) complex matrix matrix*/
    dc=fft2_out[0].r; /*dc term*/
    /*Origo is assumed to at the center of image, so we have to compensate for phase shift
     * caused by fft*/
    
    for(int i=0;i<m;i++)
    {
        for(int j=0;j<n/2+1;j++)
        {
        
            
            zMr[i*(n/2+1)+j]=pow(-1.0,i+j)*fft2_out[i*(n/2+1)+j].r/dc;
            zMi[i*(n/2+1)+j]=pow(-1.0,i+j)*fft2_out[i*(n/2+1)+j].i/dc;
            Fx[i*(n/2+1)+j]=j*1/Lx;
           
            Fy[i*(n/2+1)+j]=i*1/Ly;
            if(i>m/2-1)
                Fy[i*(n/2+1)+j]=(i-m)*1/Ly;
        }
    }
    free(fft2_out);
    free(stf);
   
   
}

void calcImgFFT(
    const AoStruct &ao,                         // AO图像结构体, 包含了图像数据
    double naxis1_scale, double naxis2_scale,   // 图像尺度, 单位
    double *zMr, double *zMi, 
    double *Fx, double *Fy                      // 输出结果, 包含了频谱数据
) {
    // 图像尺寸
    // TODO: 有时候需要从图像中截取子图像, 此时m, n 就不是naxis1了
    int width = ao._naxis1;
    int height = ao._naxis2;
    int dimcount = 2;
    int dim[2] = {height, width};

    // 将图像转换成二维矩阵
    double imgM[height][width];
    for (size_t i = 0; i < ao._naxis2; i++) {
        for (size_t j = 0; j < ao._naxis1; j++) {
            imgM[i][j] = ao._img[i*ao._naxis1 + j];
        }
    }

    // 这里确认一下顺序, naxis1一般是x, naxis2一般是y
    double Lw, Lh;
    Lw = width * naxis1_scale;
    Lh = height * naxis2_scale;

    calc_image_fft(*imgM,height,width,0.009942,0.009942,zMr,zMi,Fx,Fy);

    // calc_image_fft(double *M,int m,int n,double dx,double dy,double *zMr,double *zMi,double *Fx,double *Fy);

    // auto st = kiss_fftr_alloc(width,0,0,0);

    // // 计算图像的FFT
    // kiss_fft_cpx *fft_out = new kiss_fft_cpx[height*width];
    // kiss_fftndr_cfg stf = kiss_fftndr_alloc(dim, dimcount, 0, 0, 0);
    // kiss_fftndr(stf, ao._img, fft_out);

    return;
}

int main(int argc, const char** argv) {

    const string fits_path = "/home/lho/MyProgramm/2024.04/g2o_test/test_data/jittered_2.fits";

    AoStruct ao;
    readImgFromFITS(fits_path, ao);

    double naxis1_scale = 0.009942;
    double naxis2_scale = 0.009942;
    double zMr[ao._naxis1 * ao._naxis2];
    double zMi[ao._naxis1 * ao._naxis2];
    double Fx[ao._naxis1 * ao._naxis2];
    double Fy[ao._naxis1 * ao._naxis2];
    calcImgFFT(ao, naxis1_scale, naxis2_scale, zMr, zMi, Fx, Fy);

    // std::ofstream f("test.pgm", std::ios::out);
    // f << "P2" <<  std::endl;
    // f << ao._naxis1 << " " << ao._naxis2 << std::endl;
    // f << 1024 << std::endl;
    // for (size_t i = 0; i < ao._naxis1; i++) {
    //     for (size_t j = 0; j < ao._naxis2; j++) {
    //         f << (int)ao._img[i*ao._naxis2 + j] << " ";
    //     }
    //     f << std::endl;
    // }
    // f.close();

    return 0;
}