#pragma once
#include <string.h>
#include <iostream>
#include <fstream>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/features/shot_omp.h>
#include <pcl/keypoints/harris_3d.h>

#include <io.h>
#include <direct.h>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <pcl/features/fpfh_omp.h>

#include <pcl/features/shot_lrf.h>
#include <random>	//随机数生成
#include <boost/random.hpp>
#include <unordered_set>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <numeric>
#include <iomanip>

#include <pcl/io/obj_io.h>
//cloud2mesh
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
//内存占用监控头文件
#include <psapi.h>


using namespace std;

typedef struct {
	float x;
	float y;
	float z;
}Vertex;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef struct {
	int src_index;
	int des_index;
	pcl::PointXYZ src;
	pcl::PointXYZ des;
	Eigen::Vector3f src_norm;
	Eigen::Vector3f des_norm;
	Eigen::Matrix3f covariance_src, covariance_des;
	Eigen::Vector4f centeroid_src, centeroid_des;
	double score;
	int inlier_weight;
	float vote_score;	
}Corre_3DMatch;	//一般情况下只有源点和目标点的信息
typedef struct
{
	int index;	//当前correspondence的下标
	int degree;
	double score;
	vector<int> corre_index;	//与它有置信度的其他corre.的下标
	int true_num;
}Vote_exp;
typedef struct
{
	int index;
	double score;
}Vote;
typedef struct {
	Eigen::Matrix4d est;
	double score;
}est_score;

float MeshResolution_mr_compute(PointCloudPtr& cloud);
Eigen::MatrixXf Graph_construction(vector<Corre_3DMatch>& correspondence, float resolution, bool sc2, const string& name, const string& descriptor, float inlier_thresh);
double Distance(pcl::PointXYZ& A, pcl::PointXYZ& B);
double Square(float x);
vector<Corre_3DMatch> SDVR_top_k(vector<float> final_score, vector<Corre_3DMatch> correspondence, int k);
bool compair(const Vote& v1, const Vote& v2);
void weight_SVD(PointCloudPtr& src_pts, PointCloudPtr& des_pts, Eigen::VectorXd& weights, double weight_threshold, Eigen::Matrix4d& trans_Mat);
bool evaluation_est(Eigen::Matrix4d est, Eigen::Matrix4d gt, double re_thresh, double te_thresh, double& RE, double& TE);
double calculate_rotation_error(Eigen::Matrix3d& est, Eigen::Matrix3d& gt);
double calculate_translation_error(Eigen::Vector3d& est, Eigen::Vector3d& gt);
void Registration(string resultPath, string datasetPath, string datasetName, string descriptor);
void post_refinement(vector<Corre_3DMatch>& correspondence, PointCloudPtr& src_corr_pts, PointCloudPtr& des_corr_pts, Eigen::Matrix4d& initial, double& best_score, double inlier_thresh, int iterations, const string& metric);
double evaluation_trans(vector<Corre_3DMatch>& Match, vector<Corre_3DMatch>& correspondnece, PointCloudPtr& src_corr_pts, PointCloudPtr& des_corr_pts, double weight_thresh, Eigen::Matrix4d& trans, double metric_thresh, const string& metric, float resolution, bool instance_equal);
double evaluation_trans(vector<Corre_3DMatch>& Match, vector<Corre_3DMatch>& correspondnece, PointCloudPtr& src_corr_pts, PointCloudPtr& des_corr_pts, double weight_thresh, Eigen::Matrix4d& trans, double metric_thresh, const string& metric, float resolution, bool instance_equal);
bool SVOS_RR_RE_TE(const string& name, string src_pointcloud, string des_pointcloud, const string& corr_path, const string& label_path, const string& ov_label, const string& gt_mat, const string& folderPath, double& RE, double& TE, double& inlier_num, double& total_num, double& inlier_ratio, double& success_num, double& total_estimate, const string& descriptor, vector<double>& time_consumption);