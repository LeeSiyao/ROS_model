#include <ros/ros.h>
//#include <eigen3/Eigen/Array>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;


Matrix<double, 3, 3> A;               // Fixed rows and cols. Same as Matrix3d.
Matrix<double, 3, Dynamic> B;         // Fixed rows, dynamic cols.
Matrix<double, Dynamic, Dynamic> C;   // Full dynamic. Same as MatrixXd.
Matrix<double, 3, 3, RowMajor> E;     // Row major; default is column-major.
Matrix3f P, Q, R;                     // 3x3 float matrix.
Vector3f x, y, z;                     // 3x1 float matrix.
RowVector3f a, b, c;                  // 1x3 float matrix.
VectorXd v;                           // Dynamic column vector of doubles



int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "Sawyer");

    int i=0,j=0;

    ROS_INFO("Eigen: x.size() :[%d].",x.size());
    ROS_INFO("Eigen: C.rows() :[%f].",C.rows());
    ROS_INFO("Eigen: C.cols() :[%f].",C.cols());
    ROS_INFO("Eigen: x(%d) :[%f].",i,x(i));
//    ROS_INFO("Eigen: C(%d,%d) :[%f].",i,j,C(i,j));

    // Eigen          // Matlab           // comments
//    x.size();          // length(x)        // vector size
//    C.rows();          // size(C,1)        // number of rows
//    C.cols();          // size(C,2)        // number of columns
//    x(i);              // x(i+1)           // Matlab is 1-based
//    C(i,j);            // C(i+1,j+1)       ////



}
