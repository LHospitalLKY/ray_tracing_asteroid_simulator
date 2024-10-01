/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-05-03 16:32:46
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-10-01 14:04:45
 * @FilePath: /g2o_test/src/common/common.cpp
 * @Description: common.h的实现
 */

#include "../include/common.h"

// Stringsplit
void Stringsplit(std::string str, const char &split,
                 std::vector<std::string> &res) {
  // std::istringstream iss(str);       // 输入流
  // std::string token;                 // 接收缓冲区
  // while (getline(iss, token, split)) // 以split为分隔符
  // {
  //   // std::cout << token << std::endl; // 输出
  //   res.push_back(token);
  // }
  if (str == "") {
    return;
  }                               //判空
  std::string strs = str + split; //末尾加上分隔符方便计算
  size_t size = strs.size();
  size_t pos{0};    //分割位置
  bool meet{false}; //分隔符连续标志位

  for (size_t i = 0; i <= size; i++) {
    if (strs[i] != split && !meet) {
      pos = i;
      meet = true;
    }
    if (strs[i] == split && meet) {
      res.push_back(strs.substr(pos, i - pos));
      pos = i;
      meet = false;
    }
  }
}

// 顶点加法
VertexPosition operator+(const VertexPosition &v1, const VertexPosition &v2) {
  VertexPosition v;
  v.push_back(v1[0] + v2[0]);
  v.push_back(v1[1] + v2[1]);
  v.push_back(v1[2] + v2[2]);

  return v;
}
// 顶点减法
VertexPosition operator-(const VertexPosition &v1, const VertexPosition &v2) {
  VertexPosition v;
  v.push_back(v1[0] - v2[0]);
  v.push_back(v1[1] - v2[1]);
  v.push_back(v1[2] - v2[2]);

  return v;
}

// phaseAngle
double phaseAngle(const ViewVector &obsView, const ViewVector &sunView) {
  double _obsView[3] = {obsView[0], obsView[1], obsView[2]};
  double _sunView[3] = {sunView[0], sunView[1], sunView[2]};
  // 计算相角
  double cos_alpha = _obsView[0] * _sunView[0] + _obsView[1] * _sunView[1] +
                     _obsView[2] * _sunView[2];
  cos_alpha = cos_alpha / (norm(_obsView) * norm(_sunView));
  return acos(cos_alpha);
}

// cross
void cross(double *A, double *B, double *C) {
  /*Calculate the cross product of two vectors A and B, place the result
   * in the vector C
   */
  C[0] = A[1] * B[2] - A[2] * B[1];
  C[1] = A[2] * B[0] - A[0] * B[2];
  C[2] = A[0] * B[1] - A[1] * B[0];
}

// norm
double norm(double *A) {
  /*Calculate the norm of a vector A
   */
  return sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
}

// rotate
void rotate(double beta, double lambda, double omega, double omega0, double t,
            Eigen::Matrix3d &R) {
  beta = DEG2RAD(90 - beta);
  lambda = DEG2RAD(lambda);
  omega = 24.0 * 2.0 * M_PI / omega;
  omega0 = DEG2RAD(omega0);

  double f, cf, sf, cb, sb, cl, sl;
  f = omega * t + omega0;
  cf = cos(f);
  sf = sin(f);
  cb = cos(beta);
  sb = sin(beta);
  cl = cos(lambda);
  sl = sin(lambda);

  Eigen::Matrix3d R_1;
  R_1 << cf, sf, 0, -sf, cf, 0, 0, 0, 1;
  Eigen::Matrix3d R_2;
  R_2 << cb * cl, cb * sl, -sb, -sl, cl, 0, sb * cl, sb * sl, cb;

  R = R_1 * R_2;
}

// read_lcurve_damit
void read_lcurve_damit(std::string filename, TimeList &jd_list,
                       BriList &bri_list, VectorList &sun_vec_list,
                       VectorList &obs_vec_list) {
  std::fstream fid;
  fid.open(filename, std::ios::in);
  if (!fid.is_open()) {
    std::cerr << "Cannot open damit lcurve file " << filename << std::endl;
    return;
  }
  // 读取第一行, 得到观测段数
  std::string header;
  getline(fid, header);
  int lc_segment_num = std::stoi(header);
  if (lc_segment_num < 1) {
    std::cerr << "Error reading lcurve file (total number of lightcurves)!"
              << lc_segment_num << std::endl;
    return;
  }

  for (size_t i = 0; i < lc_segment_num; i++) {
    // 读取第二行, 得到观测点数和亮度类型
    std::string line;
    getline(fid, line);
    std::vector<std::string> line_split;
    Stringsplit(line, ' ', line_split);
    // std::cout << "Nobs: " << stoi(line_split[0]) << " Brightness catalog: "
    // << line_split[1] << std::endl;

    for (size_t j = 0; j < stoi(line_split[0]); j++) {
      std::string dataBuffer;
      getline(fid, dataBuffer);
      std::vector<std::string> dataBuffer_split;
      Stringsplit(dataBuffer, ' ', dataBuffer_split);
      // std::cout << dataBuffer_split.size() << " ";
      // 时间
      jd_list.push_back(stod(dataBuffer_split[0]));
      // 亮度
      bri_list.push_back(stod(dataBuffer_split[1]));
      // 太阳方向
      sun_vec_list.push_back(ViewVector{stod(dataBuffer_split[2]),
                                        stod(dataBuffer_split[3]),
                                        stod(dataBuffer_split[4])});
      // 观测方向
      obs_vec_list.push_back({stod(dataBuffer_split[5]),
                              stod(dataBuffer_split[6]),
                              stod(dataBuffer_split[7])});

      // std::cout << std::endl;
    }
  }
}

// write_shape_stl
void write_shape_stl(std::string filename, const FacetList &facetList,
                     const VerticeList &verticeList,
                     const NormalList &normalList) {
  assert(facetList.size() == visiableList.size());
  assert(facetList.size() == normalList.size());

  std::ofstream ofs;
  ofs.open(filename, std::ios::out);

  // 文件头
  ofs << "solid exported by facet list, vertices list and visiableList"
      << std::endl;
  // 遍历所有面片
  for (size_t i = 0; i < facetList.size(); i++) {
    // 写入面片法向量
    ofs << "facet normal " << normalList[i][0] << " " << normalList[i][1] << " "
        << normalList[i][2] << std::endl;
    ofs << "outer loop" << std::endl;
    // 写入面片顶点
    for (size_t j = 0; j < 3; j++) {
      FacetIndex facet = facetList[i];
      ofs << "vertex " << verticeList[facetList[i][j] - 1][0] << " "
          << verticeList[facetList[i][j] - 1][1] << " "
          << verticeList[facetList[i][j] - 1][2] << std::endl;
    }
    ofs << "endloop" << std::endl;
    ofs << "endfacet" << std::endl;
  }
  ofs << "endsolid exported by facet list, vertices list and visiableList"
      << std::endl;
}

// read_shape
void read_shape(std::string filename, FacetList &facetList,
                VerticeList &verticeList, int type3) {
  /* Read shape and return vertices and facets. If type3 is nonzero, then file
   * is assumed to contain '3' between every facet */
  int nvert, nfac;
  std::fstream fid;
  fid.open(filename, std::ios::in);
  if (!fid.is_open()) {
    std::cerr << "Cannot open shape file " << filename << std::endl;
    return;
  }
  // 读取第一行, 顶点数和面片数
  std::string header;
  getline(fid, header);
  std::vector<std::string> header_split;
  Stringsplit(header, ' ', header_split);

  int vert_num = std::stoi(header_split[0]);
  int facet_num = std::stoi(header_split[1]);

  std::cout << "Find " << vert_num << " vertices and " << facet_num
            << " facets." << std::endl;

  // 读取顶点列表和面片列表
  for (int i = 0; i < vert_num; i++) {
    std::string line;
    getline(fid, line);
    std::vector<std::string> line_split;
    Stringsplit(line, ' ', line_split);
    VertexPosition vertex;
    for (int j = 0; j < 3; j++) {
      vertex.push_back(std::stof(line_split[j]));
    }
    verticeList.push_back(vertex);
  }
  std::cout << "Read " << verticeList.size() << " vertices positions."
            << std::endl;

  for (int i = 0; i < facet_num; i++) {
    std::string line;
    getline(fid, line);
    std::vector<std::string> line_split;
    Stringsplit(line, ' ', line_split);
    FacetIndex facet;
    for (int j = 0; j < 3; j++) {
      facet.push_back(std::stoi(line_split[j]));
    }
    facetList.push_back((facet));
  }
  std::cout << "Read " << facetList.size() << " facets indexes." << std::endl;
}