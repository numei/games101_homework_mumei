#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
using std::cout, std::endl, std::sin, std::cos, std::atan2, std::endl;

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f make_translation(const Eigen::Vector3f &t)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = t.x();
    T(1, 3) = t.y();
    T(2, 3) = t.z();
    return T;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float x = axis.x(), y = axis.y(), z = axis.z();
    const float EPS = 1e-8f;
    if (axis.norm() < EPS)
        return Eigen::Matrix4f::Identity();
    // method 1: align the rotation axis to one of the coordinate axis, then do the rotation, and then align back

    // float alignYaxisAngle = -std::atan2(x, z);
    // float alignXaxisAngle = std::atan2(y, std::sqrt(x * x + z * z));
    // Eigen::Matrix4f rotate, alignYaxis, alignXaxis, alignYaxisInv, alignXaxisInv, anyAxisRotation;
    // alignYaxis << std::cos(alignYaxisAngle), 0, std::sin(alignYaxisAngle), 0,
    //     0, 1, 0, 0,
    //     -std::sin(alignYaxisAngle), 0, std::cos(alignYaxisAngle), 0,
    //     0, 0, 0, 1;
    // alignXaxis << 1, 0, 0, 0,
    //     0, std::cos(alignXaxisAngle), -std::sin(alignXaxisAngle), 0,
    //     0, std::sin(alignXaxisAngle), std::cos(alignXaxisAngle), 0,
    //     0, 0, 0, 1;
    // rotate << std::cos(angle), -std::sin(angle), 0, 0,
    //     std::sin(angle), std::cos(angle), 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1;
    // Eigen::Matrix4f T = alignXaxis * alignYaxis;
    // anyAxisRotation = T.inverse() * rotate * T;
    // return anyAxisRotation;

    // method 2: Rodrigues' rotation formula
    // Matrix3f I = Eigen::Matrix3f::Identity();
    // Matrix3f N, n;
    // N << 0, -z, y,
    //     z, 0, -x,
    //     -y, x, 0;

    // Matrix4f result = Eigen::Matrix4f::Identity();
    // result.block<3, 3>(0, 0) = cos(angle) * I + (1 - cos(angle)) * axis * axis.transpose() + sin(angle) * N;
    // return result;
    // method 3: use Eigen's AngleAxisf class
    Eigen::Vector3f a = axis.normalized();
    Eigen::Matrix3f R = Eigen::AngleAxisf(angle, a).toRotationMatrix();
    Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
    M.block<3, 3>(0, 0) = R;
    return M;
}
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;
    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    rotation_angle = rotation_angle * MY_PI / 180.f;

    Eigen::Vector3f axis(0, 0, 1);
    Eigen::Vector3f center(0, 0, 0);
    Eigen::Matrix4f T_to_origin = make_translation(-center);
    Eigen::Matrix4f R = get_rotation(axis, rotation_angle);
    Eigen::Matrix4f T_back = make_translation(center);

    model = T_back * R * T_to_origin;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // aspect_ratio = width / height (top/)
    // eye_fov = fovY
    // fovY is the vertical field of view, in degrees. It is the angle between the top and bottom sides of the viewing frustum.
    // fovX=fovY*aspect_ratio

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    eye_fov = eye_fov * MY_PI / 180.f;

    float left, right, top, bottom, near, far, A, B;
    near = zNear;
    far = zFar;
    top = std::tan(eye_fov / 2) * near;
    bottom = -top;
    right = aspect_ratio * top;
    left = -right;

    A = near + far;
    B = -near * far;
    Eigen::Matrix4f persp2ortho;
    Eigen::Matrix4f ortho, translate, scale;

    translate << 1, 0, 0, -(left + right) / 2,
        0, 1, 0, -(top + bottom) / 2,
        0, 0, 1, -(near + far) / 2,
        0, 0, 0, 1;
    scale << 2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (near - far), 0,
        0, 0, 0, 1;
    ortho = scale * translate;

    persp2ortho << near, 0, 0, 0,
        0, near, 0, 0,
        0, 0, A, B,
        0, 0, 1, 0;

    Eigen::Matrix4f Mt(4, 4);
    Mt << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    projection = ortho * persp2ortho * Mt * projection;
    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
