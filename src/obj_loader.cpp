/*
 * @Author: lhopital 1141165506@qq.com
 * @Date: 2024-10-01 14:20:18
 * @LastEditors: lhopital 1141165506@qq.com
 * @LastEditTime: 2024-10-01 23:21:38
 * @FilePath: /g2o_test/src/obj_loader.cpp
 * @Description: obj_loader.h的实现
 */

#include "../include/obj_loader.h"

OBJLoader::OBJLoader() {
    num_facets = 0;
    num_vertex = 0;
    load_flag = false;
}

OBJLoader::OBJLoader(const std::string &objPath) {
    // 导入obj数据
    load_flag = load(objPath);
    if (load_flag == false) {
        num_facets = 0;
        num_vertex = 0;
    }
}

bool OBJLoader::load(const std::string &objPath) {
    // tiny obj
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warning;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warning, &err, objPath.c_str(), nullptr, false);
    if (!ret) {
        std::cout << "Error: " << err << std::endl;
        return false;
    }

    std::cout << "Warning: " << warning << std::endl;

    // 计算顶点数量
    num_vertex = calcNumVertices(attrib);
    // 计算面的数量
    num_facets = calcNumFacets(attrib, shapes);

    load_flag = true;

    return true;
}

int OBJLoader::calcNumVertices(const tinyobj::attrib_t &att) {
    // 将点写入顶点列表中
    vertices_list.clear();
    // vertices.clear();
    int num = att.vertices.size() / 3;
    // TODO: 想方法提高效率, 循环效率低
    for (size_t i = 0; i < num; i++) {
        std::vector<double> vp;
        vp.push_back(att.vertices[3*i]);
        vp.push_back(att.vertices[3*i + 1]);
        vp.push_back(att.vertices[3*i + 2]);

        vertices_list.push_back(vp);
    }

    return att.vertices.size() / 3;
}

int OBJLoader::calcNumFacets(const tinyobj::attrib_t &att, const std::vector<tinyobj::shape_t> &shapes) {
    // 清空面列表
    facet_list.clear();
    // facets.clear();
    int num = 0;
    for (size_t s = 0; s < shapes.size(); s++) {
        int num_s = shapes[s].mesh.num_face_vertices.size();
        num = num + num_s;
        
        int k = 0;
        for (size_t i = 0; i < num_s; i++) {
            FacetIndex facet;
            // Facet facet;
            // facet.allVertex_ptr = std::make_shared<VertexList>(vertices);
            // facet.ver_num = shapes[s].mesh.num_face_vertices[i];
            int vertex_num_this_facet = shapes[s].mesh.num_face_vertices[i];
            for (size_t j = 0; j < shapes[s].mesh.num_face_vertices[i]; j++) {
                size_t vertice_idx = shapes[s].mesh.indices[k].vertex_index;

                // 将索引写到Facet类中
                facet.push_back(vertice_idx);
                // 得到顶点的坐标
                // float x = att.vertices[3 * vertice_idx];
                // float y = att.vertices[3 * vertice_idx + 1];
                // float z = att.vertices[3 * vertice_idx + 2];

                // std::vector<float> vertice_pos;
                // vertice_pos.push_back(x);
                // vertice_pos.push_back(y);
                // vertice_pos.push_back(z);

                // facet.vertex.push_back(vertice_pos);
                
                k++;
            }
            // 计算法向量
            // TODO: 增加法线方向判断函数
            if (facet.size() != vertex_num_this_facet) {
                std::cerr << "Load facet wrong: ";
                std::cerr << "Facet structure has different num of facet.vertex.size() and facet.ver_num" << std::endl;
                return 0;
            }
            if (facet.size() < 3 || vertex_num_this_facet < 3) {
                std::cerr << "Facet has not enough vertex" << std::endl;
                return 0;
            }
            // Eigen::Vector3f p_1(facet.vertex[0][0],facet.vertex[0][1], facet.vertex[0][2]);
            // Eigen::Vector3f p_2(facet.vertex[1][0], facet.vertex[1][1], facet.vertex[1][2]);
            // Eigen::Vector3f p_3(facet.vertex[2][0], facet.vertex[2][1], facet.vertex[2][2]);

            Eigen::Vector3f p_1, p_2, p_3;
            p_1 << vertices_list[facet[0] - 1][0], vertices_list[facet[0] - 1][1], vertices_list[facet[0] - 1][2];
            p_2 << vertices_list[facet[1] - 1][0], vertices_list[facet[1] - 1][1], vertices_list[facet[1] - 1][2];
            p_3 << vertices_list[facet[2] - 1][0], vertices_list[facet[2] - 1][1], vertices_list[facet[2] - 1][2];
            // p_1 << 
            //     (*facet.allVertex_ptr)[facet.vertex[0]][0], 
            //     (*facet.allVertex_ptr)[facet.vertex[0]][1], 
            //     (*facet.allVertex_ptr)[facet.vertex[0]][2];
            // p_2 << 
            //     (*facet.allVertex_ptr)[facet.vertex[1]][0], 
            //     (*facet.allVertex_ptr)[facet.vertex[1]][1], 
            //     (*facet.allVertex_ptr)[facet.vertex[1]][2];
            // p_3 << 
            //     (*facet.allVertex_ptr)[facet.vertex[2]][0], 
            //     (*facet.allVertex_ptr)[facet.vertex[2]][1], 
            //     (*facet.allVertex_ptr)[facet.vertex[2]][2];

            Eigen::Vector3f normalV = (p_2 - p_1).cross(p_3 - p_2);
            normalV.normalize();

            // 验证一下
            /*
            std::cout << (p_3 - p_1).dot(normalV) << std::endl;
            if (facet.ver_num == 4) {
                Eigen::Vector3f p_4(facet.vertex[3][0], facet.vertex[3][1], facet.vertex[3][2]);
                std::cout << (p_4 - p_1).dot(normalV) << std::endl;
                std::cout << (p_4 - p_2).dot(normalV) << std::endl;
                std::cout << (p_4 - p_3).dot(normalV) << std::endl;
            }
            std::cout << std::endl;
            */

            std::vector<double> normalV_;
            normalV_.push_back(normalV(0));
            normalV_.push_back(normalV(1));
            normalV_.push_back(normalV(2));

            normal_list.push_back(normalV_);

            // facet.normalVec = normalV_;

            // facets.push_back(facet);
        }
    }
    return num;
}

int OBJLoader::getNumFacets() {
    return num_facets;
}

int OBJLoader::getNumVertex() {
    return num_vertex;
}

const VerticeList* OBJLoader::getVertices() {
    return &vertices_list;
}
const FacetList* OBJLoader::getFacets() {
    return &facet_list;
}
const NormalList* OBJLoader::getNormals() {
    return &normal_list;
}

// Facet OBJLoader::getFacetStructs(int i) {
//     return facets[i];
// }

// std::vector<float> OBJLoader::getNormalVec(int i) {
//     return facets[i].normalVec;
// }

// const Shape* OBJLoader::getShapeStruct() {
//     return &facets;
// }

// const std::vector<std::vector<float>>* OBJLoader::getVertices() {
//     return &vertices;
// }