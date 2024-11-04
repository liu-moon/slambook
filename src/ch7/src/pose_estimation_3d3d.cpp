#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;
using namespace cv;


/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> { // 顶点的维度6 Sophus::SE3d:顶点估计值类型
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3d();
  }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override { // 更新顶点的估计值 _estimate
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(istream &in) override {}

  virtual bool write(ostream &out) const override {}
};

/// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

  virtual void computeError() override {
    const VertexPose *pose = static_cast<const VertexPose *> ( _vertices[0] );
    _error = _measurement - pose->estimate() * _point;  // 计算观测点和预测点的差异
  }

  virtual void linearizeOplus() override { // 计算误差函数对优化变量的雅可比
    VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
    Sophus::SE3d T = pose->estimate();
    Eigen::Vector3d xyz_trans = T * _point; // _point 在相机坐标系下的位置
    _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity(); // 对平移部分的导数
    _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans); // 旋转部分的导数
  }

  bool read(istream &in) {}

  bool write(ostream &out) const {}

protected:
  Eigen::Vector3d _point;
};

class PoseEstimation : public rclcpp::Node
{
public:
    PoseEstimation(const std::string &img1_path, const std::string &img2_path, const std::string &depth1_path, const std::string &depth2_path)
        : Node("PoseEstimation_node"), img1_path_(img1_path), img2_path_(img2_path), depth1_path_(depth1_path), depth2_path_(depth2_path)
    {
        if (img1_path_.empty() || img2_path_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "请提供img1_path和img2_path参数。");
            rclcpp::shutdown();
        }

        //-- 读取图像
        Mat img_1 = imread(img1_path_, IMREAD_COLOR);
        Mat img_2 = imread(img2_path_, IMREAD_COLOR);

        vector<KeyPoint> keypoints_1, keypoints_2;
        vector<DMatch> matches;
        find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
        cout << "一共找到了" << matches.size() << "组匹配点" << endl;

        // 建立3D点
        Mat depth1 = imread(depth1_path_, IMREAD_UNCHANGED); // 深度图为16位无符号数，单通道图像
        Mat depth2 = imread(depth2_path_, IMREAD_UNCHANGED); // 深度图为16位无符号数，单通道图像
        Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); // 内参
        vector<Point3f> pts1, pts2;

        for (DMatch m : matches)
        {
            ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];   // 通过行指针以及列索引获取深度信息
            ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
            if (d1 == 0 || d2 == 0) // bad depth
                continue;
            Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);  // 像素坐标转换为相机坐标系
            Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
            float dd1 = float(d1) / 5000.0; // 缩放为米制单位
            float dd2 = float(d2) / 5000.0;
            pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));   // 转化为3维坐标
            pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
        }

        cout << "3d-3d pairs: " << pts1.size() << endl;
        Mat R, t;
        pose_estimation_3d3d(pts1, pts2, R, t);
        cout << "ICP via SVD results: " << endl;
        cout << "R = " << R << endl;
        cout << "t = " << t << endl;
        cout << "R_inv = " << R.t() << endl;
        cout << "t_inv = " << -R.t() * t << endl;

        cout << "calling bundle adjustment" << endl;

        bundleAdjustment(pts1, pts2, R, t);

        // verify p1 = R * p2 + t
        for (int i = 0; i < 5; i++)
        {
            cout << "p1 = " << pts1[i] << endl;
            cout << "p2 = " << pts2[i] << endl;
            cout << "(R*p2+t) = " << R * (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t
                 << endl;
            cout << endl;
        }
    }

private:
    void find_feature_matches(const Mat &img_1, const Mat &img_2, vector<KeyPoint> &keypoints_1, vector<KeyPoint> &keypoints_2, vector<DMatch> &matches)
    {
        //-- 初始化
        Mat descriptors_1, descriptors_2;
        // used in OpenCV3
        Ptr<FeatureDetector> detector = ORB::create();
        Ptr<DescriptorExtractor> descriptor = ORB::create();
        // use this if you are in OpenCV2
        // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
        // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming"); // 初始化为使用Hamming距离的暴力匹配
        //-- 第一步:检测 Oriented FAST 角点位置
        detector->detect(img_1, keypoints_1);
        detector->detect(img_2, keypoints_2);

        //-- 第二步:根据角点位置计算 BRIEF 描述子
        descriptor->compute(img_1, keypoints_1, descriptors_1);
        descriptor->compute(img_2, keypoints_2, descriptors_2);

        //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
        vector<DMatch> match;
        // BFMatcher matcher ( NORM_HAMMING );
        matcher->match(descriptors_1, descriptors_2, match);

        //-- 第四步:匹配点对筛选
        double min_dist = 10000, max_dist = 0;

        // 找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
        for (int i = 0; i < descriptors_1.rows; i++)
        {
            double dist = match[i].distance;
            if (dist < min_dist)
                min_dist = dist;
            if (dist > max_dist)
                max_dist = dist;
        }

        printf("-- Max dist : %f \n", max_dist);
        printf("-- Min dist : %f \n", min_dist);

        // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
        for (int i = 0; i < descriptors_1.rows; i++)
        {
            if (match[i].distance <= max(2 * min_dist, 30.0))
            {
                matches.push_back(match[i]);
            }
        }
    };

    Point2f pixel2cam(const Point2d &p, const Mat &K) // 将一个像素坐标 p 转换为归一化相机坐标
    {
        return Point2f(
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
    }

    void pose_estimation_3d3d(const vector<Point3f> &pts1,
                              const vector<Point3f> &pts2,
                              Mat &R, Mat &t)
    {
        Point3f p1, p2; // center of mass 质心初始化
        int N = pts1.size();
        for (int i = 0; i < N; i++)
        {
            p1 += pts1[i];
            p2 += pts2[i];
        }
        p1 = Point3f(Vec3f(p1) / N);    // p1点集质心
        p2 = Point3f(Vec3f(p2) / N);    // p2点集质心
        vector<Point3f> q1(N), q2(N); // remove the center 每个点的去质心坐标
        for (int i = 0; i < N; i++)
        {
            q1[i] = pts1[i] - p1;
            q2[i] = pts2[i] - p2;
        }

        // compute q1*q2^T p196 7.57 计算W
        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        for (int i = 0; i < N; i++)
        {
            W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
        }
        cout << "W=" << W << endl;

        // SVD on W 特征值分解
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        cout << "U=" << U << endl;
        cout << "V=" << V << endl;

        Eigen::Matrix3d R_ = U * (V.transpose());   // 求旋转
        if (R_.determinant() < 0)
        {
            R_ = -R_;
        }
        Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);  // 求平移 p198 7.54

        // convert to cv::Mat
        R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2),
             R_(1, 0), R_(1, 1), R_(1, 2),
             R_(2, 0), R_(2, 1), R_(2, 2));
        t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
    }

    void bundleAdjustment(
        const vector<Point3f> &pts1,
        const vector<Point3f> &pts2,
        Mat &R, Mat &t)
    {
        // 构建图优化，先设定g2o
        typedef g2o::BlockSolverX BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
        // 梯度下降方法，可以从GN, LM, DogLeg 中选
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer; // 图模型
        optimizer.setAlgorithm(solver); // 设置求解器
        optimizer.setVerbose(true);     // 打开调试输出

        // vertex
        VertexPose *pose = new VertexPose(); // camera pose
        pose->setId(0);
        pose->setEstimate(Sophus::SE3d());
        optimizer.addVertex(pose);

        // edges
        for (size_t i = 0; i < pts1.size(); i++)
        {
            EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(
                Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
            edge->setVertex(0, pose);
            edge->setMeasurement(Eigen::Vector3d(
                pts1[i].x, pts1[i].y, pts1[i].z));
            edge->setInformation(Eigen::Matrix3d::Identity());
            optimizer.addEdge(edge);
        }

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

        cout << endl
             << "after optimization:" << endl;
        cout << "T=\n"
             << pose->estimate().matrix() << endl;

        // convert to cv::Mat
        Eigen::Matrix3d R_ = pose->estimate().rotationMatrix();
        Eigen::Vector3d t_ = pose->estimate().translation();
        R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2),
             R_(1, 0), R_(1, 1), R_(1, 2),
             R_(2, 0), R_(2, 1), R_(2, 2));
        t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
    }
    string img1_path_;
    string img2_path_;
    string depth1_path_;
    string depth2_path_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 5)
    {
        std::cerr << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << std::endl;
        return 1;
    }

    auto node = std::make_shared<PoseEstimation>(std::string(argv[1]), std::string(argv[2]), std::string(argv[3]), std::string(argv[4]));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}