#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iostream>
int main()
{

    Eigen::MatrixXf point(3, 1);
    Eigen::Matrix3f rotate, translate;
    point << 2, 1, 1;
    rotate << sqrt(2) * 1.0 / 2, -sqrt(2) / 2, 0,
        sqrt(2) / 2, sqrt(2) / 2, 0,
        0, 0, 1.0;
    translate << 0, 0, 1.0, 0, 0, 2.0, 0, 0, 1.0;
    std::cout << (translate * rotate * point) << std::endl;
    return 0;
}