#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>


class SophusExampleNode : public rclcpp::Node {
public:
    SophusExampleNode() : Node("sophus_example_node") {
        // 定义一个旋转矩阵
        Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        
        // 使用 Sophus 将旋转矩阵转换为 SO(3) 群
        Sophus::SO3d SO3_rotation(rotation_matrix);

        // 定义一个平移向量
        Eigen::Vector3d translation(1, 0, 0);

        // 使用 Sophus 定义 SE(3) 群
        Sophus::SE3d SE3_transform(SO3_rotation, translation);

        // 输出 SE(3) 群的变换矩阵
        std::cout << "SE3 Transform Matrix: \n" << SE3_transform.matrix() << std::endl;

        // 进行变换：将一个点通过 SE(3) 变换
        Eigen::Vector3d point(1, 1, 1);
        Eigen::Vector3d transformed_point = SE3_transform * point;

        std::cout << "Transformed Point: [" << transformed_point.transpose() << "]" << std::endl;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SophusExampleNode>());
    rclcpp::shutdown();
    return 0;
}
