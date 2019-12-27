#include <Eigen/Core>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/combine.h>
#include <igl/copyleft/offset_surface.h>
#include <igl/signed_distance.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/bounding_box_diagonal.h>
#include <iostream>
#include <vector>
#include <platonic_solid.h>

void bunny(double offset, int cells) {
    Eigen::MatrixXd V, SV, GV, S, CV, BV;
    Eigen::MatrixXi F, SF, side(1, 3), CF, BF;
    igl::readOBJ("../bunny.obj", V, F);
    std::cout << "Vertices: " << V.rows() << std::endl;
    std::cout << "Faces: " << F.rows() << std::endl;
    std::cout << "Diagonal: " << igl::bounding_box_diagonal(V) << std::endl;
    
    
    igl::copyleft::offset_surface(V, 
                                  F,
                                  offset,
                                  cells,
                                  igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL,
                                  SV,
                                  SF,
                                  GV,
                                  side,
                                  S);

    Eigen::Vector3d M = V.colwise().maxCoeff();
    std::cout << "Top Vertex Original: " << M(0) << " " <<  M(1) << " " <<  M(2) << std::endl;
    Eigen::Vector3d SM = SV.colwise().maxCoeff();
    std::cout << "Top Vertex Offset: " << SM(0) << " " <<  SM(1) << " " <<  SM(2) << std::endl;
    double d = (M - SM).norm();
    std::cout << "Distance: " << d << std::endl;


    igl::writeOBJ("bunny_offset.obj", SV, SF);
    igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V, SV}, {F, SF}, CV, CF);
    igl::writeOBJ("bunny_combined.obj", CV, CF);
    
    std::cin.get();
    int res = std::system("meshlab bunny_combined.obj");
}

void plato(double offset, int cells) {
    Eigen::MatrixXd V, SV, GV, S, CV, BV;
    Eigen::MatrixXi F, SF, side(1, 3), CF, BF;

    platonic_solid::dodecahedron(V, F);
        std::cout << "Vertices: " << V.rows() << std::endl;
    std::cout << "Faces: " << F.rows() << std::endl;
    std::cout << "Diagonal: " << igl::bounding_box_diagonal(V) << std::endl;

    igl::copyleft::offset_surface(V, 
                                  F,
                                  offset,
                                  cells,
                                  igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL,
                                  SV,
                                  SF,
                                  GV,
                                  side,
                                  S);

    Eigen::Vector3d M = V.colwise().maxCoeff();
    std::cout << "Top Vertex Original: " << M(0) << " " <<  M(1) << " " <<  M(2) << std::endl;
    Eigen::Vector3d SM = SV.colwise().maxCoeff();
    std::cout << "Top Vertex Offset: " << SM(0) << " " <<  SM(1) << " " <<  SM(2) << std::endl;
    double d = (M - SM).norm();
    std::cout << "Distance: " << d << std::endl;


    igl::writeOBJ("dodecahedron_offset.obj", SV, SF);
    igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V, SV}, {F, SF}, CV, CF);
    igl::writeOBJ("dodecahedron_combined.obj", CV, CF);
    
    std::cin.get();
    int res = std::system("meshlab dodecahedron_combined.obj");
}


int main(int argc, char **argv) { 
    auto offset = std::stod(argv[1]);
    auto cells = std::stoi(argv[2]);
    plato(offset, cells);
    // bunny(offset, cells);
}