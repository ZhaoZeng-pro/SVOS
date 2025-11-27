#pragma once
#include "head.h"

string folderPath;
bool add_overlap;
bool low_inlieratio;
bool no_logs = true;
double RE, TE, success_estimate_rate;
vector<int>scene_num;
vector<float> top100_inlier_rate;	//记录投票结果top100内点率
vector<float> top50_inlier_rate;	//记录投票结果top50内点率
vector<float> hypothsis_rate;	//记录假设生成过程中均为内点正确率
vector<float> voter_weight; //记录匹配的投票权重
float max_memory = 0.0;	//记录最大内存占用
float mean_time = 0.0;	//记录平均耗时
int count_num = 0;


int main()
{
	string resultPath = "D:/code/c++/SVOS/data/fpfh/result/";
	string datasetPath = "D:/code/c++/SVOS/data/fpfh/3dlomatch/";


	pcl::console::setVerbosityLevel(pcl::console::L_ERROR);	//让控制台只输出错误信息而忽视警告

	if (matrix_SVOS)
		cout << "SVOS matrix model" << endl;
	
	//Registration(resultPath, datasetPath, "3dmatch", "fpfh");	//3dmatch
	Registration(resultPath, datasetPath, "3dlomatch", "fpfh");	//3dlomatch

	cout << "最大内存占用为：" << max_memory << "MB"<< endl;
	cout << "场景全部耗时为：" << mean_time << "s" << endl;


}
