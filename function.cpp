#pragma once
#include "head.h"

extern bool low_inlieratio;
extern float graph_thresh;
extern vector<float> top100_inlier_rate;
extern vector<float> top50_inlier_rate;;
extern vector<float> hypothsis_rate;
extern float mean_time;
extern double RE, TE, success_estimate_rate;
extern vector<int>scene_num;
extern bool matrix_SVOS;

static string threeDMatch[8] = {
	"7-scenes-redkitchen",
	"sun3d-home_at-home_at_scan1_2013_jan_1",
	"sun3d-home_md-home_md_scan9_2012_sep_30",
	"sun3d-hotel_uc-scan3",
	"sun3d-hotel_umd-maryland_hotel1",
	"sun3d-hotel_umd-maryland_hotel3",
	"sun3d-mit_76_studyroom-76-1studyroom2",
	"sun3d-mit_lab_hj-lab_hj_tea_nov_2_2012_scan1_erika",
};

static string threeDlomatch[8] = {
	"7-scenes-redkitchen_3dlomatch",
	"sun3d-home_at-home_at_scan1_2013_jan_1_3dlomatch",
	"sun3d-home_md-home_md_scan9_2012_sep_30_3dlomatch",
	"sun3d-hotel_uc-scan3_3dlomatch",
	"sun3d-hotel_umd-maryland_hotel1_3dlomatch",
	"sun3d-hotel_umd-maryland_hotel3_3dlomatch",
	"sun3d-mit_76_studyroom-76-1studyroom2_3dlomatch",
	"sun3d-mit_lab_hj-lab_hj_tea_nov_2_2012_scan1_erika_3dlomatch",
};

float MeshResolution_mr_compute(PointCloudPtr& cloud)
{
	int i;
	//计算点云分辨率
	float mr = 0;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int>pointIdx;
	vector<float>pointDst;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ query_point;
	for (i = 0; i < cloud->points.size(); i++)
	{
		query_point = cloud->points[i];
		kdtree.nearestKSearch(query_point, 2, pointIdx, pointDst);
		float x = cloud->points[pointIdx[0]].x - cloud->points[pointIdx[1]].x;
		float y = cloud->points[pointIdx[0]].y - cloud->points[pointIdx[1]].y;
		float z = cloud->points[pointIdx[0]].z - cloud->points[pointIdx[1]].z;
		float mr_temp = sqrt(x * x + y * y + z * z);
		mr += mr_temp;
	}
	mr /= cloud->points.size();
	return mr;//approximate calculation
}
/*
resultPath: save result path
dataPath: correspondence data path
datasetName: dataset name
descriptor: descriptor name, affects graph construction
*/
void Registration(string resultPath, string datasetPath, string datasetName, string descriptor)
{
	int id = 0;
	//////////////////////////////////////////////////////////////////
	int opt;
	int digit_opind = 0;
	int option_index = 0;

	int corrected = 0;
	int total_num = 0;
	double total_re = 0;
	double total_te = 0;
	vector<double>total_success_est_rate;
	vector<int> scene_correct_num;
	vector<double>scene_re_sum;
	vector<double>scene_te_sum;

	{
		int pair_num = 1623;	//1623 or 1781 here
		if (datasetName == "3dlomatch")
		{
			pair_num = 1781;
		}
		
		//string txt_path = datasetPath + "/" + descriptor;
		const string& txt_path = datasetPath;
		string analyse_csv = resultPath + descriptor + ".csv";
		ofstream outFile;
		outFile.open(analyse_csv.c_str(), ios::out);
		outFile.setf(ios::fixed, ios::floatfield);
		outFile << "pair_name" << ',' << "corrected_or_no" << ',' << "inlier_num" << ',' << "total_num" << ',' << "inlier_ratio" << ',' << "RE" << ',' << "TE" << endl;
		vector<string>fail_pair;
		vector<double>time;
		for (int i = id; i < pair_num; i++)
		{
			time.clear();
			std::cout << "Pair " << i + 1 << "，total" << pair_num/*name_list.size()*/ << "，fail " << fail_pair.size() << endl;

			string filename = to_string(i);/*name_list[i]*/;
			string corr_path = txt_path + "/" + filename + "@corr.txt";
			string gt_mat_path = txt_path + "/" + filename + "@GTmat.txt";
			string gt_label_path = txt_path + "/" + filename + "@label.txt";
			string ov_label = "NULL";
			string folderPath = resultPath + "/";
			double re, te;
			double inlier_num, total_num;
			double inlier_ratio, success_estimate, total_estimate;

			int corrected;
			
			corrected = SVOS_RR_RE_TE(datasetName, "NULL", "NULL", corr_path, gt_label_path, ov_label, gt_mat_path, folderPath, re, te, inlier_num, total_num, inlier_ratio, success_estimate, total_estimate, descriptor, time);
			
			if (corrected)
			{
				std::cout << filename << " Success." << endl;
				RE += re;
				TE += te;
			}
			else
			{
				fail_pair.push_back(filename);
				std::cout << filename << " Fail." << endl;
			}
			outFile << filename << ',' << corrected << ',' << inlier_num << ',' << total_num << ',';
			outFile << setprecision(4) << inlier_ratio << ',' << re << ',' << te << endl;
			std::cout << endl;
		}

		outFile.close();

		outFile.open(resultPath + "/details.txt", ios::out);
		outFile.setf(ios::fixed, ios::floatfield);

		double success_num = pair_num - fail_pair.size();
		std::cout << "total:" << endl;
		outFile << "total:" << endl;
		std::cout << "\tRR:" << pair_num - fail_pair.size() << "/" << pair_num << " " << success_num / (pair_num / 1.0) << endl;
		outFile << "\tRR:" << pair_num - fail_pair.size() << "/" << pair_num << " " << success_num / (pair_num / 1.0) << endl;
		std::cout << "\tRE:" << RE / (success_num / 1.0) << endl;
		outFile << "\tRE:" << RE / (success_num / 1.0) << endl;
		std::cout << "\tTE:" << TE / (success_num / 1.0) << endl;
		outFile << "\tTE:" << TE / (success_num / 1.0) << endl;
		std::cout << "fail pairs:" << endl;
		outFile << "fail pairs:" << endl;
		for (size_t i = 0; i < fail_pair.size(); i++)
		{
			cout << "\t" << fail_pair[i] << endl;
		}
	}
}

bool SVOS_RR_RE_TE(const string& name, string src_pointcloud, string des_pointcloud, const string& corr_path, const string& label_path, const string& ov_label, const string& gt_mat, const string& folderPath, double& RE, double& TE, double& inlier_num, double& total_num, double& inlier_ratio, double& success_num, double& total_estimate, const string& descriptor, vector<double>& time_consumption)
{
	bool sc2 = true;
	bool Corr_select = false;
	bool GT_cmp_mode = false;
	int max_est_num = INT_MAX;
	bool ransc_original = false;
	bool instance_equal = true;
	string hypothsis_method = "Top50";	//max_degree OR Top50
	//string metric = "MAE++";
	string metric = "MAE++";
	
	success_num = 0;
	
	std::cout << folderPath << endl;
	string dataPath = corr_path.substr(0, corr_path.rfind("/"));
	string item_name = folderPath.substr(folderPath.rfind("/") + 1, folderPath.length());

	FILE* corr, * gt;
	corr = fopen(corr_path.c_str(), "r");
	gt = fopen(label_path.c_str(), "r");
	if (corr == NULL) {
		std::cout << " error in loading correspondence data. " << std::endl;
		std::cout << corr_path << endl;
		exit(-1);
	}
	if (gt == NULL) {
		std::cout << " error in loading ground truth label data. " << std::endl;
		std::cout << label_path << endl;
		exit(-1);
	}

	FILE* ov;
	vector<double>ov_corr_label;
	double max_corr_weight = 0;

	//PointCloudPtr Overlap_src(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr Raw_src(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr Raw_des(new pcl::PointCloud<pcl::PointXYZ>);
	float raw_des_resolution = 0;
	float raw_src_resolution = 0;
	//pcl::KdTreeFLANN<pcl::PointXYZ>kdtree_Overlap_des, kdtree_Overlap_src;

	PointCloudPtr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr cloud_des(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_src(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_des(new pcl::PointCloud<pcl::Normal>);
	vector<Corre_3DMatch>correspondence;
	vector<int>true_corre;
	inlier_num = 0;
	float resolution = 0;
	bool kitti = false;
	Eigen::Matrix4d GTmat;

	//GTMatRead(gt_mat, GTmat);
	FILE* fp = fopen(gt_mat.c_str(), "r");
	if (fp == NULL)
	{
		printf("Mat File can't open!\n");
		return -1;
	}
	fscanf(fp, "%lf %lf %lf %lf\n", &GTmat(0, 0), &GTmat(0, 1), &GTmat(0, 2), &GTmat(0, 3));
	fscanf(fp, "%lf %lf %lf %lf\n", &GTmat(1, 0), &GTmat(1, 1), &GTmat(1, 2), &GTmat(1, 3));
	fscanf(fp, "%lf %lf %lf %lf\n", &GTmat(2, 0), &GTmat(2, 1), &GTmat(2, 2), &GTmat(2, 3));
	fscanf(fp, "%lf %lf %lf %lf\n", &GTmat(3, 0), &GTmat(3, 1), &GTmat(3, 2), &GTmat(3, 3));
	fclose(fp);
	
	
	
	int idx = 0;
	while (!feof(corr))
	{
		Corre_3DMatch t;
		pcl::PointXYZ src, des;
		fscanf(corr, "%f %f %f %f %f %f\n", &src.x, &src.y, &src.z, &des.x, &des.y, &des.z);
		t.src = src;
		t.des = des;
		t.inlier_weight = 0;
		t.score = 0;
		
		correspondence.push_back(t);
		idx++;
	}
	fclose(corr);
	

	total_num = correspondence.size();
	while (!feof(gt))
	{
		int value;
		fscanf(gt, "%d\n", &value);
		true_corre.push_back(value);
		if (value == 1)
		{
			inlier_num++;
		}
	}
	fclose(gt);

	inlier_ratio = 0;
	if (inlier_num == 0)
	{
		std::cout << " NO INLIERS！ " << endl;
	}
	inlier_ratio = inlier_num / (total_num / 1.0);
	std::cout << "inliers: " << inlier_num << "\ttotal_num: " << total_num << endl;
	double RE_thresh, TE_thresh, inlier_thresh;
	if (name == "KITTI")
	{
		RE_thresh = 5;
		TE_thresh = 60;
		inlier_thresh = 0.6;
	}
	else if (name == "3dmatch" || name == "3dlomatch")
	{
		RE_thresh = 15;
		TE_thresh = 30;
		inlier_thresh = 0.1;
	}
	else if (name == "U3M") {
		inlier_thresh = 5 * resolution;
		RE_thresh = 5;
		TE_thresh = 300;
	}
	if (name == "KITTI_LC")
	{
		RE_thresh = 5;
		TE_thresh = 60;
		inlier_thresh = 0.6;
	}

	std::chrono::time_point<std::chrono::system_clock> start, end;
	std::chrono::duration<double> elapsed_time, total_time;


	start = std::chrono::system_clock::now();

	Eigen::MatrixXf Graph;
	sc2 = false;
	
	Graph = Graph_construction(correspondence, resolution, sc2, name, descriptor, inlier_thresh);
	

	/*Corres_Viewer_Score(cloud_src, cloud_des, correspondence, resolution, 1000, GTmat);
	visualization(cloud_src, cloud_des, GTmat, resolution);*/

	end = std::chrono::system_clock::now();
	elapsed_time = end - start;
	time_consumption.push_back(elapsed_time.count());
	total_time += elapsed_time;
	cout << " graph construction: " << elapsed_time.count() << endl;
	if (Graph.norm() == 0) {
		cout << "Graph is disconnected." << endl;
		return false;
	}
	start = std::chrono::system_clock::now();

	vector<int>degree(total_num, 0);
	vector<Vote_exp> pts_degree;
	for (int i = 0; i < total_num; i++)
	{
		Vote_exp t;
		t.true_num = 0;
		vector<int> corre_index;
		for (int j = 0; j < total_num; j++)
		{
			if (i != j && Graph(i, j)) {
				degree[i]++;
				corre_index.push_back(j);
				if (true_corre[j])
				{
					t.true_num++;
				}
			}
		}
		t.index = i;
		t.degree = degree[i];
		t.corre_index = corre_index;
		pts_degree.push_back(t);
	}

	////////////////////投票/////////////////
	vector<float> final_score(total_num, 0);
	if (matrix_SVOS)	//SVOS矩阵乘法形式
	{
		Eigen::MatrixXf SVOS_Matrix;
		SVOS_Matrix = Graph * Graph;
		

		for (int i = 0; i < total_num; i++)
		{
			final_score[i] = Graph.row(i) * SVOS_Matrix.col(i);
			correspondence[i].vote_score = final_score[i];
		}
	}
	else
	{
		int static_node_num = correspondence.size();

		vector<int> node_idex, node_flag;
		vector<vector<int>> node_class;
		for (int i = 0; i < static_node_num; i++)
		{
			node_idex.push_back(i);
			node_flag.push_back(0);
		}

		int ccc = 0;

		vector<float> static_score_temp(total_num, 0);
		vector<vector<int>> remain_node;
		vector<int> node_temp;

		int temp_index;
		for (int i = 0; i < total_num; i++)  //对所有中心node往外扩展静态投票集
		{
			static_score_temp.clear();
			static_score_temp.resize(total_num);
			for (int j = 0; j < pts_degree[i].corre_index.size(); j++)
			{
				temp_index = pts_degree[i].corre_index[j];
				if (static_score_temp[temp_index] != 0)
					cout << "有问题";
				if (Distance(correspondence[i].des, correspondence[temp_index].des) > inlier_thresh)	//很近的点视作同一个点
				{

					//static_score_temp[temp_index] ++; //投1票

					static_score_temp[temp_index] += Graph(i, temp_index); //投边权重票
				}
			}

			for (int j = 0; j < pts_degree[i].corre_index.size(); j++)
			{
				temp_index = pts_degree[i].corre_index[j];
				for (int k = 0; k < pts_degree[temp_index].corre_index.size(); k++)
				{

					//给投票者投票
					if (static_score_temp[pts_degree[temp_index].corre_index[k]] > 0)
					{
						if (Distance(correspondence[temp_index].des, correspondence[pts_degree[temp_index].corre_index[k]].des) > inlier_thresh)	//很近的点视作同一个点
						{

							//static_score_temp[pts_degree[temp_index].corre_index[k]]++;

							static_score_temp[pts_degree[temp_index].corre_index[k]] += Graph(temp_index, pts_degree[temp_index].corre_index[k]); //投边权重票
						}
					}
				}
			}

			for (int j = 0; j < total_num; j++)
			{
				final_score[j] += static_score_temp[j];
				correspondence[i].vote_score = final_score[i];
			}

		}
	}


	///////////////////投票结束/////////////


	//根据综合投票分数确定最终配准对
	vector<Corre_3DMatch> match_SDVR;
	int top_k;
	/*cout << "name == " << name << endl;
	cout << "descriptor == " << descriptor << endl;*/
	if (name == "U3M" || name == "KITTI")
	{
		top_k = 100;
	}
	else
	{
		//top_k = 0.2 * total_num;
		top_k = 1000;
		if (total_num < top_k)
			top_k = total_num;
	}

	//top_k = 100;

	match_SDVR = SDVR_top_k(final_score, correspondence, top_k);  //匹配对得分排序取前部分
	//voteScoreRankForCorr(correspondence);	//将匹配对根据投票分数排序
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_corr_trans(new pcl::PointCloud<pcl::PointXYZ>);	//用于验证追踪的临时数据
	pcl::PointCloud<pcl::PointXYZ> src_corr_temp;	//用于验证追踪的临时数据

	end = std::chrono::system_clock::now();
	elapsed_time = end - start;
	total_time += elapsed_time;
	time_consumption.push_back(elapsed_time.count());
	cout << "Voting time: " << elapsed_time.count() << endl;

	float flag = 0;
	src_corr_temp.clear();
	float flag_top50;
	for (int i = 0; i < match_SDVR.size(); i++)
	{
		src_corr_temp.push_back(match_SDVR[i].src);
	}
	pcl::transformPointCloud(src_corr_temp, *src_corr_trans, GTmat);
	for (int i = 0; i < src_corr_trans->size(); i++)
	{
		float dis = pow(src_corr_trans->points[i].x - match_SDVR[i].des.x, 2) + pow(src_corr_trans->points[i].y - match_SDVR[i].des.y, 2) + pow(src_corr_trans->points[i].z - match_SDVR[i].des.z, 2);
		dis = sqrt(dis);
		if (dis < inlier_thresh)
		{
			flag++;
			if (i < match_SDVR.size() / 2)
				flag_top50++;
		}
	}
	if (inlier_num <= 100)	//记录top100内点率
	{
		top100_inlier_rate.push_back(flag / inlier_num);
	}
	else
	{
		top100_inlier_rate.push_back(flag / 100);

	}
	if (inlier_num <= 50)
	{
		top50_inlier_rate.push_back(flag_top50 / inlier_num);
	}
	else
	{
		top50_inlier_rate.push_back(flag_top50 / 50);
	}

	cout << "top" << match_SDVR.size() << "内点数量验证：共有" << flag << "个内点" << endl;
	cout << "top" << match_SDVR.size() / 2 << "内点数量验证：共有" << flag_top50 << "个内点" << endl;
	flag = 0;
	src_corr_temp.clear();
	src_corr_trans->clear();

	////////////////////SDVR结束////////////////////////////////////

	/******************************************registraion***************************************************/

	RE = RE_thresh;
	TE = TE_thresh;
	Eigen::Matrix4d best_est;


	bool found = false;
	double best_score = 0;
	vector<Corre_3DMatch>selected;
	vector<int>corre_index;
	start = std::chrono::system_clock::now();
	vector<Corre_3DMatch>Group;
	vector<int>selected_index;
	Eigen::Matrix4d est_trans;
	int iteration = -1;	//迭代1000次

	/********************************在假设评估前对参与假设评估的点对进行剔除**************************************/
	PointCloudPtr src_corr_pts(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr des_corr_pts(new pcl::PointCloud<pcl::PointXYZ>);
	src_corr_pts->clear();
	des_corr_pts->clear();
	for (size_t i = 0; i < correspondence.size(); i++)
	{
		src_corr_pts->push_back(correspondence[i].src);
		des_corr_pts->push_back(correspondence[i].des);
	}
	vector<Corre_3DMatch> correspondence_filter;
	if (metric == "MAE++")
	{
		correspondence_filter = SDVR_top_k(final_score, correspondence, correspondence.size());
		int remove_count = 0;
		int all = correspondence_filter.size();

		for (int i = 0; i < all; i++)
		{
			if (i >= all - remove_count)
				break;
			for (int j = i + 1; j < all; j++)
			{
				if (j >= all - remove_count)
					break;

				/*if (Distance(correspondence_filter[i].des, correspondence_filter[j].des) == 0)
				{
					correspondence_filter.erase(correspondence_filter.begin() + j);
					j--;
					remove_count++;
				}*/
				//3dmatch && 3dlomatch
				if (name == "3dmatch" || name == "3dlomatch")
				{
					if (Distance(correspondence_filter[i].des, correspondence_filter[j].des) <= inlier_thresh && Distance(correspondence_filter[i].src, correspondence_filter[j].src) <= inlier_thresh)
					{

						correspondence_filter.erase(correspondence_filter.begin() + j);
						j--;
						remove_count++;
					}
				}
				else
				{
					if (Distance(correspondence_filter[i].des, correspondence_filter[j].des) == 0)
					{
						correspondence_filter.erase(correspondence_filter.begin() + j);
						j--;
						remove_count++;
					}
				}


			}
		}

		src_corr_temp.clear();
		src_corr_trans->clear();
		for (int i = 0; i < correspondence_filter.size(); i++)
		{
			src_corr_temp.push_back(correspondence_filter[i].src);
		}
		pcl::transformPointCloud(src_corr_temp, *src_corr_trans, GTmat);
		

		remove_count = 0;
		src_corr_pts->clear();
		des_corr_pts->clear();
		for (size_t i = 0; i < correspondence_filter.size(); i++)
		{
			src_corr_pts->push_back(correspondence_filter[i].src);
			des_corr_pts->push_back(correspondence_filter[i].des);
		}
	}
	//match_SDVR = vector<Corre_3DMatch>(correspondence_filter.begin(), correspondence_filter.begin() + 200);


	float hypo_rate = 0;
	/********************************************剔除完成*********************************************************/
	Eigen::MatrixXf Graph_after_voting;

	//Graph_after_voting = Graph_construction(match_SDVR, 5 * resolution, 0.95);

	Graph_after_voting = Graph_construction(match_SDVR, resolution, sc2, name, descriptor, inlier_thresh);
	total_num = match_SDVR.size();



	//Eigen::MatrixXf Graph_after_voting = Graph_construction(match_SDVR, resolution, sc2, name, descriptor, inlier_thresh, true);
	//临时数据

	vector<int>degree_temp(total_num, 0);	//临时数据
	vector<Vote_exp> pts_degree_temp;	//临时数据
	for (int i = 0; i < total_num; i++)
	{
		Vote_exp t;
		t.true_num = 0;
		vector<int> corre_index;
		for (int j = 0; j < total_num; j++)
		{
			if (i != j && Graph_after_voting(i, j)) {
				degree[i]++;
				corre_index.push_back(j);
				if (true_corre[j])
				{
					t.true_num++;
				}
			}
		}
		t.index = i;
		t.degree = degree_temp[i];
		t.corre_index = corre_index;
		pts_degree_temp.push_back(t);
	}


	//Eigen::MatrixXf Graph_after_voting = Graph_construction(match_SDVR, resolution, sc2, name, descriptor, inlier_thresh);
	int temp1, temp2, temp3, temp4;
	int iteration_thresh = 50000;  //迭代次数

	if (hypothsis_method == "Top50")
	{
		iteration++;
#pragma omp for
		for (int i = 0; i < match_SDVR.size(); i++)
		{

			for (int j = 0; j < pts_degree_temp[i].corre_index.size(); j++)
			{
				temp1 = pts_degree_temp[i].corre_index[j];
				if (temp1 <= i)
					continue;
				/*if (temp1 > 250 && i <= 250)
					break;*/
				for (int k = 0; k < pts_degree_temp[temp1].corre_index.size(); k++)
				{
					temp2 = pts_degree_temp[temp1].corre_index[k];
					if (temp2 <= temp1)
						continue;
					/*if (temp2 > 250 && i <= 250)
						break;*/
					if (Graph_after_voting(i, temp1) == 0 || Graph_after_voting(temp1, temp2) == 0)
					{
						cout << "这不对吧！" << endl;
					}

					if (Graph_after_voting(i, temp2) == 0)
					{
						continue;
					}

					/*if (!is_nodes_consistence(match_SDVR[i], match_SDVR[temp1], match_SDVR[temp2], 5 * resolution))
					{
						continue;
					}*/


					Group.push_back(match_SDVR[i]);
					Group.push_back(match_SDVR[temp1]);
					Group.push_back(match_SDVR[temp2]);


					double score, weight_thresh = -1;
					if (metric == "MAE++")
					{
						score = evaluation_trans(Group, correspondence_filter, src_corr_pts, des_corr_pts, weight_thresh, est_trans, inlier_thresh, metric, raw_des_resolution, instance_equal);	//是不是3dmatch的inlier_thresh有问题？
					}
					else if (metric == "MAE")
						score = evaluation_trans(Group, correspondence, src_corr_pts, des_corr_pts, weight_thresh, est_trans, inlier_thresh, metric, raw_des_resolution, instance_equal);

					est_score temp;
					temp.est = est_trans;
					temp.score = score;
					
					if (GT_cmp_mode)
					{
						//GT已知
						if (score > 0)
						{
							//评估est
							double re, te;
							bool success = evaluation_est(est_trans, GTmat, 15, 30, re, te);
							{
								success_num = success ? success_num + 1 : success_num;
								if (success && re < RE && te < TE)
								{
									RE = re;
									TE = te;
									best_est = est_trans;
									best_score = score;
									selected = Group;
									corre_index = selected_index;
									found = true;
								}
							}
						}
					}
					else {
						//GT未知

						if (score > 0)
						{
#pragma omp critical
							if (best_score < score)
							{
								best_score = score;
								best_est = est_trans;
								selected = Group;
								corre_index = selected_index;
							}

						}
					}
					Group.clear();
					Group.shrink_to_fit();
					selected_index.clear();
					selected_index.shrink_to_fit();
					iteration++;
					if (iteration > iteration_thresh)
						break;
				}
				if (iteration > iteration_thresh)
					break;
			}
			if (iteration > iteration_thresh)
				break;
		}

	}

	//假设生成的匹配对正确性验证

	end = std::chrono::system_clock::now();
	elapsed_time = end - start;
	time_consumption.push_back(elapsed_time.count());
	total_time += elapsed_time;
	std::cout << " hypothesis generation & evaluation: " << elapsed_time.count() << endl;
	Eigen::MatrixXd tmp_best;

	std::cout << "total time: " << total_time.count() << endl;
	PROCESS_MEMORY_COUNTERS pmc;
	float temp_t = 0.0;
	if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
	{
		temp_t = pmc.WorkingSetSize / 1048576;
		printf("当前进程占用内存大小为：%I64d MB\n", pmc.WorkingSetSize / 1048576);
		std::cout << "总耗时：" << total_time.count() << "s" << endl;
	}

	mean_time += total_time.count();

	
	
	if (!found)
	{
		found = evaluation_est(best_est, GTmat, RE_thresh, TE_thresh, RE, TE);
	}
	tmp_best = best_est;
	post_refinement(correspondence, src_corr_pts, des_corr_pts, best_est, best_score, inlier_thresh, 20, "MAE");

	

	std::cout << selected.size() << " " << best_score << endl;

	for (int i = 0; i < selected.size(); i++)
	{
		std::cout << selected[i].score << " ";
	}
	std::cout << endl;

	//可视化配准结果
	//sort(est_set.begin(), est_set.end(),sortByScore);

	/*visualization(cloud_src, cloud_des, best_est, resolution);
	RMSE_visualization(cloud_src, cloud_des, best_est, GTmat, resolution);
	*/

	correspondence.clear();
	correspondence.shrink_to_fit();
	ov_corr_label.clear();
	ov_corr_label.shrink_to_fit();
	true_corre.clear();
	true_corre.shrink_to_fit();
	degree.clear();
	degree.shrink_to_fit();
	pts_degree.clear();
	pts_degree.shrink_to_fit();
	selected.clear();
	selected.shrink_to_fit();
	corre_index.clear();
	corre_index.shrink_to_fit();
	src_corr_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
	des_corr_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_src.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_des.reset(new pcl::PointCloud<pcl::PointXYZ>);
	normal_src.reset(new pcl::PointCloud<pcl::Normal>);
	normal_des.reset(new pcl::PointCloud<pcl::Normal>);
	Raw_src.reset(new pcl::PointCloud<pcl::PointXYZ>);
	Raw_des.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//Overlap_src.reset(new pcl::PointCloud<pcl::PointXYZ>);

	
	
	if (found)
	{
		double new_re, new_te;
		evaluation_est(best_est, GTmat, RE_thresh, TE_thresh, new_re, new_te);
		if (new_re < RE && new_te < TE)
		{
			cout << "est_trans updated!!!" << endl;
			cout << "RE=" << new_re << " " << "TE=" << new_te << endl;
			cout << best_est << endl;
		}
		else {
			best_est = tmp_best;
			cout << "RE=" << RE << " " << "TE=" << TE << endl;
			cout << best_est << endl;
		}
		RE = new_re;
		TE = new_te;
		/*if (inlier_ratio < 0.05)
		{
			Corres_selected_visual(cloud_src, cloud_des, selected, resolution, 0.1, GTmat);
			visualization(cloud_src, cloud_des, best_est, resolution);
		}*/

		return true;
	}
	else {
		double new_re, new_te;
		found = evaluation_est(best_est, GTmat, RE_thresh, TE_thresh, new_re, new_te);
		if (found)
		{
			RE = new_re;
			TE = new_te;
			cout << "est_trans corrected!!!" << endl;
			cout << "RE=" << RE << " " << "TE=" << TE << endl;
			cout << best_est << endl;
			return true;
		}
		//Corres_selected_visual(Raw_src, Raw_des, correspondence, resolution, 0.1, GTmat);
		//Corres_selected_visual(Raw_src, Raw_des, selected, resolution, 0.1, GTmat);

		cout << "RE=" << RE << " " << "TE=" << TE << endl;
		return false;
	}
	
}

double Square(float x)
{
	return x * x;
}

//get distance between two points
double Distance(pcl::PointXYZ& A, pcl::PointXYZ& B)
{
	float result;
	result = sqrt(Square(A.x - B.x) + Square(A.y - B.y) + Square(A.z - B.z));
	return result;
}

Eigen::MatrixXf Graph_construction(vector<Corre_3DMatch>& correspondence, float resolution, bool sc2, const string& name, const string& descriptor, float inlier_thresh) {
	int size = correspondence.size();
	Eigen::MatrixXf cmp_score;
	cmp_score.resize(size, size);
	cmp_score.setZero();
	Corre_3DMatch c1, c2;
	float score, src_dis, des_dis, dis, alpha_dis;
	
	
	if (name == "3dmatch" || name == "3dlomatch")
	{
		
		for (int i = 0; i < size; i++)
		{
			c1 = correspondence[i];

			for (int j = i + 1; j < size; j++)
			{
				c2 = correspondence[j];
				src_dis = Distance(c1.src, c2.src);
				des_dis = Distance(c1.des, c2.des);

				
				dis = abs(src_dis - des_dis);

				{
					alpha_dis = 10 * resolution;  //MAC中的
					if (alpha_dis == 0)
						alpha_dis = 2 * inlier_thresh;
					//alpha_dis = 2 * inlier_thresh;  
					score = exp(-dis * dis / (2 * alpha_dis * alpha_dis));  //每个点0.1偏差 0.995 每个点0.2偏差0.98 每个点0.3偏差 0.955 每个点0.4偏差 0.92 每个点0.5偏差 0.88

					//score = (min(src_dis, des_dis) / max(src_dis, des_dis));    //另一种距离兼容分数

					//score = 1 - (dis * dis) / (inlier_thresh * inlier_thresh);
					if (name == "3dmatch" && descriptor == "fcgf")
					{
						//cout << "fcgf+3dmatch" << endl;

						score = (score < 0.99) ? 0 : score;
					}
					else if (name == "3dmatch" && descriptor == "fpfh") {
						//cout << "3dmatch and fpfh" << endl;
						score = (score < 0.99) ? 0 : score;
					}
					
					else if (name == "3dlomatch" && descriptor == "fpfh") {
						//3dlomatch
						score = (score < 0.995) ? 0 : score; //MAC: 3dlomatch 0.99, 3dmatch fcgf 0.999 fpfh 0.995 
					}
					else if (descriptor == "geotrans" && name == "3dmatch")
					{
						score = (score < 0.9999) ? 0 : score;
					}
					else if (descriptor == "geotrans" && name == "3dlomatch")
					{
						score = (score < 0.9999) ? 0 : score;
					}
					else if (name == "3dmatch" && descriptor == "predator")
					{
						score = (score < 0.9999) ? 0 : score;
					}
					else if (name == "3dlomatch" && descriptor == "predator")
					{
						score = (score < 0.999) ? 0 : score;
					}
					
					else {
						//3dlomatch
						score = (score < 0.999) ? 0 : score; 
					}
				}
				cmp_score(i, j) = score;
				cmp_score(j, i) = score;
			}


		}

	}
	else {
		
		for (int i = 0; i < size; i++)
		{
			c1 = correspondence[i];
			for (int j = i + 1; j < size; j++)
			{
				c2 = correspondence[j];
				src_dis = Distance(c1.src, c2.src);
				des_dis = Distance(c1.des, c2.des);
				dis = abs(src_dis - des_dis);
				alpha_dis = 2 * inlier_thresh;
				score = exp(-dis * dis / (2 * alpha_dis * alpha_dis));
				score = (score < 0.999) ? 0 : score;

				cmp_score(i, j) = score;
				cmp_score(j, i) = score;
			}
		}
	}
	if (sc2)
	{
		//Eigen::setNbThreads(6);
		//cout << "construct SOG" << endl;
		cmp_score = cmp_score.cwiseProduct(cmp_score * cmp_score);
	}
	return cmp_score;
}

/*
功能：对一系列拥有得分的SDVR匹配对进行排序，选出top-k个匹配对，返回top-k个匹配对
输入：
输出：
时间：2024/1/13
编辑人：曾钊
*/
vector<Corre_3DMatch> SDVR_top_k(vector<float> final_score, vector<Corre_3DMatch> correspondence, int k)
{
	vector<Vote> index_score;
	Vote temp;
	for (int i = 0; i < final_score.size(); i++)
	{
		temp.index = i;
		temp.score = final_score[i];
		index_score.push_back(temp);
	}

	sort(index_score.begin(), index_score.end(), compair);  //能排序成功吗？

	vector<Corre_3DMatch> match_top_k;
	Corre_3DMatch match_temp;
	for (int i = 0; i < k; i++)
	{
		match_temp = correspondence[index_score[i].index];
		//match_temp.score = index_score[i].score;
		match_top_k.push_back(match_temp);
	}

	return match_top_k;
}

bool compair(const Vote& v1, const Vote& v2)
{
	return v1.score > v2.score;
}

double evaluation_trans(vector<Corre_3DMatch>& Match, vector<Corre_3DMatch>& correspondnece, PointCloudPtr& src_corr_pts, PointCloudPtr& des_corr_pts, double weight_thresh, Eigen::Matrix4d& trans, double metric_thresh, const string& metric, float resolution, bool instance_equal)
{
	PointCloudPtr src_pts(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudPtr des_pts(new pcl::PointCloud<pcl::PointXYZ>);
	vector<double> weights;
	for (auto& i : Match)
	{

		if (i.score >= weight_thresh)
		{
			src_pts->push_back(i.src);
			des_pts->push_back(i.des);
			weights.push_back(i.score);
		}
		else
		{
			return -1;
		}
	}
	if (weights.size() < 3)
	{
		return 0;
	}
	Eigen::VectorXd weight_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(weights.data(), weights.size());
	weights.clear();
	weights.shrink_to_fit();
	weight_vec /= weight_vec.maxCoeff();
	if (instance_equal)
		weight_vec.setOnes(); // 2023.2.23 

	weight_SVD(src_pts, des_pts, weight_vec, 0, trans);
	PointCloudPtr src_trans(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*src_corr_pts, *src_trans, trans);
	//Eigen::Matrix4f trans_f = trans.cast<float>();
	//Eigen::Matrix3f R = trans_f.topLeftCorner(3, 3);
	double score = 0.0;
	int inlier = 0;
	int corr_num = src_corr_pts->points.size();
	float n1, n2;
	for (int i = 0; i < corr_num; i++)
	{

		double dist = Distance(src_trans->points[i], des_corr_pts->points[i]);
		double w = 1;
		if (dist < metric_thresh)
		{
			inlier++;
			if (metric == "inlier")
			{
				score += 1 * w;//correspondence[i].inlier_weight;
			}
			else if (metric == "MAE" || metric == "MAE++")
			{
				score += (metric_thresh - dist) * w / metric_thresh;
			}
			else if (metric == "MSE")
			{
				score += pow((metric_thresh - dist), 2) * w / pow(metric_thresh, 2);
			}
		}
	}

	src_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
	des_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
	src_trans.reset(new pcl::PointCloud<pcl::PointXYZ>);
	return score;

}

void weight_SVD(PointCloudPtr& src_pts, PointCloudPtr& des_pts, Eigen::VectorXd& weights, double weight_threshold, Eigen::Matrix4d& trans_Mat) {
	for (size_t i = 0; i < weights.size(); i++)
	{
		weights(i) = (weights(i) < weight_threshold) ? 0 : weights(i);
	}

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> weight;
	Eigen::VectorXd ones = weights;
	ones.setOnes();
	weight = (weights * ones.transpose());
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Identity = weight;

	Identity.setIdentity();
	weight = (weights * ones.transpose()).cwiseProduct(Identity);
	pcl::ConstCloudIterator<pcl::PointXYZ> src_it(*src_pts);
	pcl::ConstCloudIterator<pcl::PointXYZ> des_it(*des_pts);

	src_it.reset(); des_it.reset();
	Eigen::Matrix<double, 4, 1> centroid_src, centroid_des;
	pcl::compute3DCentroid(src_it, centroid_src);
	pcl::compute3DCentroid(des_it, centroid_des);

	src_it.reset(); des_it.reset();
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> src_demean, des_demean;
	pcl::demeanPointCloud(src_it, centroid_src, src_demean);
	pcl::demeanPointCloud(des_it, centroid_des, des_demean);

	Eigen::Matrix<double, 3, 3> H = (src_demean * weight * des_demean.transpose()).topLeftCorner(3, 3);

	// Compute the Singular Value Decomposition
	Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<double, 3, 3> u = svd.matrixU();
	Eigen::Matrix<double, 3, 3> v = svd.matrixV();

	// Compute R = V * U'
	if (u.determinant() * v.determinant() < 0)
	{
		for (int x = 0; x < 3; ++x)
			v(x, 2) *= -1;
	}

	Eigen::Matrix<double, 3, 3> R = v * u.transpose();

	// Return the correct transformation
	Eigen::Matrix<double, 4, 4> Trans;
	Trans.setIdentity();
	Trans.topLeftCorner(3, 3) = R;
	const Eigen::Matrix<double, 3, 1> Rc(R * centroid_src.head(3));
	Trans.block(0, 3, 3, 1) = centroid_des.head(3) - Rc;
	trans_Mat = Trans;
}


bool evaluation_est(Eigen::Matrix4d est, Eigen::Matrix4d gt, double re_thresh, double te_thresh, double& RE, double& TE) {
	Eigen::Matrix3d rotation_est, rotation_gt;
	Eigen::Vector3d translation_est, translation_gt;
	rotation_est = est.topLeftCorner(3, 3);
	rotation_gt = gt.topLeftCorner(3, 3);
	translation_est = est.block(0, 3, 3, 1);
	translation_gt = gt.block(0, 3, 3, 1);

	RE = calculate_rotation_error(rotation_est, rotation_gt);
	TE = calculate_translation_error(translation_est, translation_gt);
	if (0 <= RE && RE <= re_thresh && 0 <= TE && TE <= te_thresh)
	{
		return true;
	}
	return false;
}

double calculate_rotation_error(Eigen::Matrix3d& est, Eigen::Matrix3d& gt) {
	double tr = (est.transpose() * gt).trace();
	return acos(min(max((tr - 1.0) / 2.0, -1.0), 1.0)) * 180.0 / M_PI;
}
double calculate_translation_error(Eigen::Vector3d& est, Eigen::Vector3d& gt) {
	Eigen::Vector3d t = est - gt;
	return sqrt(t.dot(t)) * 100;
}

void post_refinement(vector<Corre_3DMatch>& correspondence, PointCloudPtr& src_corr_pts, PointCloudPtr& des_corr_pts, Eigen::Matrix4d& initial, double& best_score, double inlier_thresh, int iterations, const string& metric) 
{
	int pointNum = src_corr_pts->points.size();
	double pre_score = best_score;
	for (int i = 0; i < iterations; i++)
	{
		double score = 0;
		Eigen::VectorXd weights, weight_pred;
		weights.resize(pointNum);
		weights.setZero();
		vector<int> pred_inlier_index;
		PointCloudPtr trans(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*src_corr_pts, *trans, initial);
		for (int j = 0; j < pointNum; j++)
		{
			double dist = Distance(trans->points[j], des_corr_pts->points[j]);
			double w = 1;
			
			if (dist < inlier_thresh)
			{
				pred_inlier_index.push_back(j);
				weights[j] = 1 / (1 + pow(dist / inlier_thresh, 2));
				if (metric == "inlier")
				{
					score += 1 * w;
				}
				else if (metric == "MAE" || metric == "MAE++")
				{
					score += (inlier_thresh - dist) * w / inlier_thresh;
				}
				else if (metric == "MSE")
				{
					score += pow((inlier_thresh - dist), 2) * w / pow(inlier_thresh, 2);
				}
			}
		}
		if (score < pre_score) {
			break;
		}
		else {
			pre_score = score;
			//估计pred_inlier
			PointCloudPtr pred_src_pts(new pcl::PointCloud<pcl::PointXYZ>);
			PointCloudPtr pred_des_pts(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*src_corr_pts, pred_inlier_index, *pred_src_pts);
			pcl::copyPointCloud(*des_corr_pts, pred_inlier_index, *pred_des_pts);
			weight_pred.resize(pred_inlier_index.size());
			for (int k = 0; k < pred_inlier_index.size(); k++)
			{
				weight_pred[k] = weights[pred_inlier_index[k]];
			}
			//weighted_svd
			weight_SVD(pred_src_pts, pred_des_pts, weight_pred, 0, initial);
			pred_src_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
			pred_des_pts.reset(new pcl::PointCloud<pcl::PointXYZ>);
		}
		pred_inlier_index.clear();
		trans.reset(new pcl::PointCloud<pcl::PointXYZ>);
	}
	best_score = pre_score;
}
