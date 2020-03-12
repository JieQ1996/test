#include "include/getLP.h"
pcl::PointCloud<pcl::PointXYZ> cloud_line;
pcl::PointCloud<pcl::PointXYZ> Points_plane_out;

PclGetLP::PclGetLP(ros::NodeHandle &nh){
	sub_point_cloud_ = nh.subscribe("/velodyne_points",2,&PclGetLP::point_feature,this);
	
	pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points",2);  // 输出过滤点
    pub_feature_points_plane_ = nh.advertise<sensor_msgs::PointCloud2>("/feature_points_plane",2);    // 输出平面
	pub_feature_points_line_ = nh.advertise<sensor_msgs::PointCloud2>("/feature_points_line",2);    // 输出直线
	//ros::Rate loop_rate(0.5);   //循环速率1s/次
	ros::spin();
}

PclGetLP::~PclGetLP(){}

void PclGetLP::Spin(){
}

//直线提取（基于法线）
int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float re,float reforn)  //输入点云、边界估计半径、法线估计半径 为分辨率的十倍
{ 
 
	pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>); //存储边界点
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud)); 
	normEst.setRadiusSearch(reforn); //设置法线估计的半径
	normEst.compute(*normals); //将法线估计结果保存至normals
	//输出法线的个数
	//std:cout<<"reforn: "<<reforn<<std::endl;
	//std::cerr << "normals: " << normals->size() << std::endl;
 
	boundEst.setInputCloud(cloud); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setRadiusSearch(re); //设置边界估计所需要的半径
	boundEst.setAngleThreshold(M_PI/4); //边界估计时的角度阈值
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.compute(boundaries); //将边界估计结果保存在boundaries
 
	//输出边界点的个数
	std::cerr << "boundaries: " <<boundaries.points.size() << std::endl;
 
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int i = 0; i < cloud->points.size(); i++) 
	{ 
		
		if(boundaries[i].boundary_point > 0) 
		{ 
			cloud_boundary->push_back(cloud->points[i]); 
		} 
	} 
	
	cloud_line = *cloud_boundary;
	return 0; 
} 

//估计平面
int estimatePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float dis)//输入点云 、距离阈值
{
	//创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(dis);         //设置误差容忍范围，也就是我说过的阈值
	
	 // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int)cloud->points.size();
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > Points_plane_out;//输出点云

    // While 30% of the original cloud is still there
    while (cloud->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_in);
		if(cloud_in->points.size() > 2000){
	        //Points_plane_out .push_back(cloud_in);
			Points_plane_out = *cloud_in;
			break;
		}

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_out);
        cloud.swap(cloud_out);//更新
		//CloudIN = cloud_out;
        i++;
    }
	

	
	return 0;
}

void PclGetLP::point_feature(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
	
	pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
	pcl::fromROSMsg(*in_cloud_ptr, laserCloudIn);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr CloudIN(new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::fromROSMsg(*in_cloud_ptr, *CloudIN);
	
	// 创建滤波器对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (CloudIN);
    sor.setMeanK (100);//寻找每个点的50个最近邻点
    sor.setStddevMulThresh (1.0);//一个点的最近邻距离超过全局平均距离的一个标准差以上，就会舍弃
    sor.filter (*cloud_filtered);
    //std::cout<<"CloudIN: "<<CloudIN->points.size()<<std::endl;
    std::cout<<"cloud_filtered: "<<cloud_filtered->points.size()<<std::endl;
	std::cout<<"CloudIN: "<<CloudIN->points.size()<<std::endl;
 
	estimateBorders(cloud_filtered,1,1);
	estimatePlane(cloud_filtered,0.05);
	
	
	sensor_msgs::PointCloud2 PlaneOut;
  	pcl::toROSMsg(Points_plane_out, PlaneOut);
 	PlaneOut.header= in_cloud_ptr->header;
	pub_feature_points_plane_.publish(PlaneOut);
	
 	sensor_msgs::PointCloud2 LineOut;
  	pcl::toROSMsg(cloud_line, LineOut);
  	LineOut.header= in_cloud_ptr->header;
  	pub_feature_points_line_.publish(LineOut);
	
	sensor_msgs::PointCloud2 PointsCloudIN;
  	pcl::toROSMsg(laserCloudIn, PointsCloudIN);
  	PointsCloudIN.header= in_cloud_ptr->header;
  	pub_filtered_points_.publish(PointsCloudIN);
	/* //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
    seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-设置目标几何形状
    seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    seg.setDistanceThreshold(0.05);         //设置误差容忍范围，也就是我说过的阈值
	
	 // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int)CloudIN->points.size();
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > Points_plane_out;//输出点云
	pcl::PointCloud<pcl::PointXYZ> Points_plane_out;
    // While 30% of the original cloud is still there
    while (CloudIN->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud(CloudIN);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(CloudIN);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_in);
		if(cloud_in->points.size() > 2000){
	        //Points_plane_out .push_back(cloud_in);
			Points_plane_out = *cloud_in;
			break;
		}

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_out);
        CloudIN.swap(cloud_out);//更新
		//CloudIN = cloud_out;
        i++;
    }
	
	sensor_msgs::PointCloud2 PlaneOut;
  	pcl::toROSMsg(Points_plane_out, PlaneOut);
 	PlaneOut.header= in_cloud_ptr->header;
	pub_feature_points_plane_.publish(PlaneOut);*/
	

	
}
	
