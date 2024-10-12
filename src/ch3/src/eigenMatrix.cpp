#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#define MATRIX_SIZE 50

class EigenNode : public rclcpp::Node
{
public:
  EigenNode() : Node("eigen_demo_node")
  {
    // 定义一个2行3列的矩阵，类型为float
    Eigen::Matrix<float,2,3> matrix_23;
    // 定义一个3维向量，其实就是3行1列的矩阵
    Eigen::Vector3d v_3d;
    // 与上述向量等价
    Eigen::Matrix<float,3,1> vd_3d;
    // 定义3x3的矩阵，初始化为0
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    // 定义动态大小的矩阵
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> matrix_dynamic;
    // 与上面等价
    Eigen::MatrixXd matrix_x;

    // 数据初始化
    matrix_23 << 1,2,3,4,5,6;

    // 输出
    std::cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << std::endl;

    // 使用()访问矩阵中的元素
    std::cout << "print matrix 2x3: " << std::endl;
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        std::cout << matrix_23(i,j) << "\t";
      }
      std::cout << std::endl;
    }

    v_3d << 3,2,1;
    vd_3d << 4,5,6;

    // 不能混合两种不同类型的矩阵，要显式转换
    Eigen::Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d;
    std::cout << "[1,2,3;4,5,6]*[3,2,1]=" <<result.transpose() << std::endl;

    Eigen::Matrix<float,2,1> result2 = matrix_23 * vd_3d;
    std::cout << "[1,2,3;4,5,6]*[4,5,6]=" <<result2.transpose() << std::endl;

    // 基本运算
    // + - * /
    matrix_33 = Eigen::Matrix3d::Random();
    std::cout << "random matrix: \n" << matrix_33 << std::endl;
    std::cout << "transpose: \n" << matrix_33.transpose() << std::endl;
    std::cout << "sum: " << matrix_33.sum() << std::endl;
    std::cout << "trace: " << matrix_33.trace() << std::endl;
    std::cout << "times 10: \n" << 10 * matrix_33 << std::endl;
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;
    std::cout << "det: " << matrix_33.determinant() << std::endl;

    // 特征值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose()*matrix_33);
    std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << std::endl;

    // 解方程
    // 求解 matrix_NN * x = v_Nd
    // N的大小在宏中定义，由随机数生成
    Eigen::Matrix<double,MATRIX_SIZE,MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose(); // 保证半正定 方程有唯一解
    Eigen::Matrix<double,MATRIX_SIZE,1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE,1);

    // 计时
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    // 记录开始时间
    rclcpp::Time start_time = clock->now();
    // 直接求逆
    Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse() * v_Nd;
    rclcpp::Time end_time = clock->now();
    rclcpp::Duration elapsed_time = end_time - start_time;

    std::cout << "time of normal inverse is "
              << elapsed_time.nanoseconds() /1000 /1000 << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    // 矩阵分解
    start_time = clock->now();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    end_time = clock->now();
    elapsed_time = end_time - start_time;
    std::cout << "time of Qr decomposition is "
              << elapsed_time.nanoseconds() /1000 /1000 << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;
    
    // cholesky分解
    start_time = clock->now();
    x = matrix_NN.ldlt().solve(v_Nd);
    end_time = clock->now();
    elapsed_time = end_time - start_time;
    std::cout << "time of Qr decomposition is "
              << elapsed_time.nanoseconds() /1000 /1000 << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EigenNode>());
  rclcpp::shutdown();
  return 0;
}
