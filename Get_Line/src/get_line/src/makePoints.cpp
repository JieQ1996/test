#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
 

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/point_types.h>
 
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
 
#include <pcl/point_cloud.h>

 
 
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
 
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
 
#include <pcl/io/ply_io.h>
 
#include <pcl/filters/statistical_outlier_removal.h>



#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <cmath>

#include<Eigen/Core>
#include<Eigen/Geometry>
float cloudCurvature[15000];
int cloudSortInd[15000];
int cloudNeighborPicked[15000];
int cloudLabel[15000];
const double scanPeriod = 0.1;
const int N_SCANS = 16;
const int N_PLANES = 6;
std::vector<int> scanStartInd(N_SCANS, 0);
std::vector<int> scanEndInd(N_SCANS, 0);



float transform[6] = {M_PI/18,M_PI/18,M_PI/18,0,0,0.5};
float transformSum[6] = {0};
typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZI PointType1;
PointType pointSel,pointOri,ave_normal,ave_tran_normal,ave_tran_tran_normal,pp;
PointType1 point,pointSell;
std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_PLANES);
void Transform(PointType const * const pi, PointType * const po)
{

  float rx = transform[0];
  float ry = transform[1];
  float rz = transform[2];
  float tx = transform[3];
  float ty = transform[4];
  float tz = transform[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
}

void TransformToLast(PointType const * const pi, PointType * const po)
{

  float rx = transformSum[0];
  float ry = transformSum[1];
  float rz = transformSum[2];
  float tx = transformSum[3];
  float ty = transformSum[4];
  float tz = transformSum[5];

  float x1 = cos(ry) * (pi->x) - sin(ry) * (pi->z);
  float y1 = pi->y;
  float z1 = sin(ry) * (pi->x) + cos(ry) * (pi->z);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(rz) * x2 + sin(rz) * y2;
  po->y = -sin(rz) * x1 + cos(rz) * y1;
  po->z = z2;
	/*
  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  */
}
void TransformToLast1(PointType1 const * const pi, PointType1 * const po)
{

  float rx = transformSum[0];
  float ry = transformSum[1];
  float rz = transformSum[2];
  float tx = transformSum[3];
  float ty = transformSum[4];
  float tz = transformSum[5];

  float x1 = cos(ry) * (pi->x) - sin(ry) * (pi->z);
  float y1 = pi->y;
  float z1 = sin(ry) * (pi->x) + cos(ry) * (pi->z);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(rz) * x2 + sin(rz) * y2;
  po->y = -sin(rz) * x1 + cos(rz) * y1;
  po->z = z2;
	/*
  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  */
}
//int user_data;
//Initialize the viewer including the backgroundcolor,coordinate axis,and others.
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
	//set the backgroundcolor:R,G,B
     viewer.setBackgroundColor (0, 0, 0);//背景为黑色
 
  /*  pcl::PointXYZ o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 5, "sphere",0);*/
	//viewer.addLine(o,"line",0);
    /*std::cout << "i only run once" << std::endl;*/
}
 
//void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
//{
//    static unsigned count = 0;
//    std::stringstream ss;
//    ss << "Once per viewer loop: " << count++;
//    viewer.removeShape ("text", 0);
//    viewer.addText (ss.str(), 200, 300, "text", 0);
//
//    //FIXME: possible race condition here:
//    user_data++;
//}
 
//Accomplish loading a PCDFile,creating a viewer,and show the cloud at the viewer.

//Create a point-cloud object,and generate a PCD File to save the point cloud object.
bool saveThePointCloud()
{
	 pcl::PointCloud<pcl::PointXYZ> Cloud;      //定义点云对象
	    // 创建点云
	Cloud.width=900;
	Cloud.height=16;
	Cloud.is_dense=false;
	Cloud.points.resize(Cloud.width*Cloud.height);
	srand(unsigned(int(NULL)));
	/*for(size_t i=0;i<Cloud.points.size();++i)
	{
		Cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		Cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		Cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
			std::cout<<i<<",";
	}
	int num = Cloud.points.size();
	std::cout<<"num="<<num<<std::endl;
	std::cout<<Cloud.points[14401].x<<std::endl;
	std::cout<<Cloud.points[0].x<<std::endl;*/
	int cw = Cloud.width;
	int ch = Cloud.height;
	for(size_t scan = 0;scan < Cloud.height;++scan)
	{
		//std::cout<<scan<<std::endl;
		//std::cout<<900*scan<<std::endl;
		for(size_t i=0;i<Cloud.width;++i)
		{
			//std::cout<<(i+900*scan)<<",";
			Cloud.points[i+cw*scan].y = 0.1*scan;
			if(i<(cw/9))
			{
				Cloud.points[i+cw*scan].x = i*(9.0f/cw);
				Cloud.points[i+cw*scan].z = 2.0f;
			}
			else if(i<(cw/3))
			{
				Cloud.points[i+cw*scan].x = 1.0f;
				Cloud.points[i+cw*scan].z = 2.0f-(9.0f/cw)*(i-cw/9);
			}
			else if(i<(2*cw/3))
			{
				Cloud.points[i+cw*scan].x = i*(9.0f/cw)-2.0f;
				Cloud.points[i+cw*scan].z = 0.0f;
			}
			else if(i<(8*cw/9))
			{
				Cloud.points[i+cw*scan].x = 4.0f;
				Cloud.points[i+cw*scan].z = (9.0f/cw)*(i-2*cw/3);
			}
			else 
			{
				Cloud.points[i+cw*scan].x = i*(9.0f/cw)-4.0f;
				Cloud.points[i+cw*scan].z = 2.0f;
			}		
			
	}
	}
	for(size_t j = 0;j < Cloud.points.size();j++)
	{
		Cloud.points[j].x = Cloud.points[j].x - 2.5;
		Cloud.points[j].y = Cloud.points[j].y - 0.75;
	}
	for(size_t j = 0;j < Cloud.points.size();j++)
	{
		Cloud.points[j].x = Cloud.points[j].x * 14;
		Cloud.points[j].y = Cloud.points[j].y * 14;
		Cloud.points[j].z = Cloud.points[j].z * 14;
	}
 	  pcl::io::savePCDFile("pointCloudValueFile.pcd",Cloud);
	  pcl::io::savePCDFileASCII("pointCloudFile.pcd",Cloud);
	  return true;
}
//水平分辨率0.2，垂直分辨率2
bool saveThePointCloud_PLUS()
{
	 pcl::PointCloud<pcl::PointXYZ> Cloud;      //定义点云对象
	    // 创建点云
	//每米5个点加三个随机点
	Cloud.width=600;
	Cloud.height=16;
	Cloud.is_dense=false;
	Cloud.points.resize(Cloud.width*Cloud.height);
	srand(unsigned(int(NULL)));
	/*for(size_t i=0;i<Cloud.points.size();++i)
	{
		Cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		Cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		Cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
			std::cout<<i<<",";
	}
	int num = Cloud.points.size();
	std::cout<<"num="<<num<<std::endl;
	std::cout<<Cloud.points[14401].x<<std::endl;
	std::cout<<Cloud.points[0].x<<std::endl;*/
	int cw = Cloud.width;
	int ch = Cloud.height;
	int fbl_x = 0.2;
	int fbl_y = 0.4;
	for(size_t scan = 0;scan < Cloud.height;++scan)
	{
		//std::cout<<scan<<std::endl;
		//std::cout<<900*scan<<std::endl;
		for(size_t i=0;i<Cloud.width;++i)
		{
			//std::cout<<(i+900*scan)<<",";
			Cloud.points[i+cw*scan].y = fbl_y * scan;
			if(i < 25)
			{
				Cloud.points[i+cw*scan].x = i*fbl_x;
				Cloud.points[i+cw*scan].z = 20.0f;
			}
			else if(i < 125)
			{
				Cloud.points[i+cw*scan].x = 25 * fbl_x;
				Cloud.points[i+cw*scan].z = 20.0f - fbl_x * (i-25);
			}
			else if(i < 275)
			{
				Cloud.points[i+cw*scan].x = (i - 100) * fbl_x;
				Cloud.points[i+cw*scan].z = 0.0f;
			}
			else if(i < 375)
			{
				Cloud.points[i+cw*scan].x = 175 * fbl_x;
				Cloud.points[i+cw*scan].z = fbl_x * (i-275);
			}
			else //if (i < 400)
			{
				Cloud.points[i+cw*scan].x = (i -200) * fbl_x ;
				Cloud.points[i+cw*scan].z = 20.0f;
			}		
			/*else
			{
				Cloud.points[i+cw*scan].x = 40.0f * rand();
				Cloud.points[i+cw*scan].z = 20.0f * rand();
			}*/
		}
	}
	/*int cloud_num =  Cloud.points.size();
	std::cout<<cloud_num; 
	for(size_t j = 0;j < Cloud.points.size();j++)
	{
		Cloud.points[j].x = Cloud.points[j].x - 20.0f;
		Cloud.points[j].y = Cloud.points[j].y - 10.5f;
	}*/
 	  pcl::io::savePCDFile("pointCloudValueFile.pcd",Cloud);
	  pcl::io::savePCDFileASCII("pointCloudFile.pcd",Cloud);
	  return true;
}
int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float re,float reforn) 
{ 
 
	pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>); 
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud)); 
	normEst.setRadiusSearch(reforn); //设置法线估计的半径 上下两线距离
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
	//std::cerr << "boundaries: " <<boundaries.points.size() << std::endl;
 
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int i = 0; i < cloud->points.size(); i++) 
	{ 
		
		if(boundaries[i].boundary_point > 0) 
		{ 
			cloud_boundary->push_back(cloud->points[i]); 
		} 
	} 
 
	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("点云库PCL从入门到精通案例"));
	
	int v1(0); 
	MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); 
	MView->setBackgroundColor (0.3, 0.3, 0.3, v1); 
	MView->addText ("Raw point clouds", 10, 10, "v1_text", v1); 
	int v2(0); 
	MView->createViewPort (0.5, 0.0, 1, 1.0, v2); 
	MView->setBackgroundColor (0.5, 0.5, 0.5, v2); 
	MView->addText ("Boudary point clouds", 10, 10, "v2_text", v2); 
 
	MView->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud",v1);
	MView->addPointCloud<pcl::PointXYZ> (cloud_boundary, "cloud_boundary",v2);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "sample cloud",v1);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "cloud_boundary",v2);
	MView->addCoordinateSystem (1.0);
	MView->initCameraParameters ();
 
	MView->spin();
 
	return 0; 
} 
int estimateL(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr &cor_out)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cor_points(new pcl::PointCloud<pcl::PointXYZI>);//边缘点
	pcl::PointCloud<pcl::PointXYZI>::Ptr cor_less_points(new pcl::PointCloud<pcl::PointXYZI>);//边缘点
	
	int cloudSize = cloud->points.size();
	std::vector<pcl::PointCloud<PointType1> > laserCloudScans(N_SCANS);
  	for (int i = 0; i < cloudSize; i++) {
    	point.x = cloud->points[i].x;
    	point.y = cloud->points[i].y;
    	point.z = cloud->points[i].z;
		
		int scanID;
		scanID = i / 900;
		point.intensity = scanID ;

    	laserCloudScans[scanID].push_back(point);
  	}

 	pcl::PointCloud<PointType1>::Ptr laserCloud(new pcl::PointCloud<PointType1>());
  	for (int i = 0; i < N_SCANS; i++) {
   		*laserCloud += laserCloudScans[i];
  	}
  	int scanCount = -1;
	//计算曲率
	for (int i = 5; i < cloudSize - 5; i++) {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;
    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
			
    if (int(laserCloud->points[i].intensity) != scanCount) {
      scanCount = int(laserCloud->points[i].intensity);

      if (scanCount > 0 && scanCount < N_SCANS) {
        scanStartInd[scanCount] = i + 5;
        scanEndInd[scanCount - 1] = i - 5;
      }
    }
  }
  scanStartInd[0] = 5;
  scanEndInd.back() = cloudSize - 5;
	/*
	for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    if (diff > 0.1) {

      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);

      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }*/
  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;

  for (int i = 0; i < N_SCANS; i++) {
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 4; j++) {
      int sp = (scanStartInd[i] * (4 - j)  + scanEndInd[i] * j) / 4;
      int ep = (scanStartInd[i] * (3 - j)  + scanEndInd[i] * (j + 1)) / 4 - 1;

      for (int k = sp + 1; k <= ep; k++) {
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > 0.1) {
        
          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cor_points->push_back(laserCloud->points[ind]);
            cor_less_points->push_back(laserCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cor_less_points->push_back(laserCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }
	}
  }
	for(int i = 0;i < cor_points->points.size();i++)
	{
		*cor_out = *cor_points; 
		//std::cout<<"cor"<<i<<"="<<cor_points->points[i]<<std::endl;
	}
	//std::cout<<"cor_num="<<	cor_points->points.size()<<"less_cor_num="<<cor_less_points->points.size();
	
}
int estimateLine(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float yaw)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);//原始输出点云
	//pcl::PointCloud<pcl::PointXYZ>::Ptr tran_cloud_out(new pcl::PointCloud<pcl::PointXYZ>);//旋转原始输出点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr tran_cloud (new pcl::PointCloud<pcl::PointXYZ>);//旋转点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr line_out(new pcl::PointCloud<pcl::PointXYZI>);//边缘点
	pcl::PointCloud<pcl::PointXYZI>::Ptr tran_line_out(new pcl::PointCloud<pcl::PointXYZI>);//边缘点
	pcl::PointCloud<pcl::PointXYZI>::Ptr tran_tran_line_out(new pcl::PointCloud<pcl::PointXYZI>);//旋转边缘点
		//旋转点云
	for(int k = 0;k < cloud->points.size();k++)
	{
		Transform(&cloud->points[k],&pointSel);
		tran_cloud->push_back(pointSel);
	}
	
	estimateL(cloud,line_out);
	estimateL(tran_cloud,tran_line_out);

	PointType LineStart,LineEnd,TranLineStart,TranLineEnd,Vec,TranVec;
	LineStart.x = line_out->points[0].x;
	LineStart.y = line_out->points[0].y;
	LineStart.z = line_out->points[0].z;

	LineEnd.x = line_out->points[60].x;
	LineEnd.y = line_out->points[60].y;
	LineEnd.z = line_out->points[60].z;
	
	Vec.x = LineEnd.x - LineStart.x;
	Vec.y = LineEnd.y - LineStart.y;
	Vec.z = LineEnd.z - LineStart.z;
	float Vec_dis;
	Vec_dis = sqrt(Vec.x * Vec.x + Vec.y * Vec.y + Vec.z * Vec.z);
	Vec.x = Vec.x / Vec_dis;
	Vec.y = Vec.y / Vec_dis;
	Vec.z = Vec.z / Vec_dis;
	std::cout<<"raw line vec"<<Vec<<std::endl;
	
	TranLineStart.x = tran_line_out->points[0].x;
	TranLineStart.y = tran_line_out->points[0].y;
	TranLineStart.z = tran_line_out->points[0].z;
	
	TranLineEnd.x = tran_line_out->points[60].x;
	TranLineEnd.y = tran_line_out->points[60].y;
	TranLineEnd.z = tran_line_out->points[60].z;
	
	TranVec.x = TranLineEnd.x - TranLineStart.x;
	TranVec.y = TranLineEnd.y - TranLineStart.y;
	TranVec.z = TranLineEnd.z - TranLineStart.z;
	float Tran_Vec_dis;
	Tran_Vec_dis = sqrt(TranVec.x * TranVec.x + TranVec.y * TranVec.y + TranVec.z * TranVec.z);
	TranVec.x = TranVec.x / Tran_Vec_dis;
	TranVec.y = TranVec.y / Tran_Vec_dis;
	TranVec.z = TranVec.z / Tran_Vec_dis;
	std::cout<<"tran line vec"<<TranVec<<std::endl;
	
	float alpha = 0;//旋转角度
	float guiyi;
	PointType trancenter;//旋转轴
	alpha = acos(TranVec.x * Vec.x + TranVec.y * Vec.y + TranVec.z * Vec.z);
	trancenter.x = Vec.y * TranVec.z - TranVec.y * Vec.z ;
	trancenter.y = -(Vec.x * TranVec.z - TranVec.x * Vec.z);
	trancenter.z = Vec.x * TranVec.y - TranVec.x * Vec.y ;
	guiyi = sqrt(trancenter.x * trancenter.x + trancenter.y * trancenter.y + trancenter.z * trancenter.z);
	trancenter.x = trancenter.x / guiyi;
	trancenter.y = trancenter.y / guiyi;
	trancenter.z = trancenter.z / guiyi;
	std::cout<<"旋转轴"<<trancenter<<"; ";
	std::cout<<"旋转角度"<<alpha<<std::endl;
	//旋转向量转化为欧拉角
	Eigen::AngleAxisd rotation_vector (alpha,Eigen::Vector3d (trancenter.x,trancenter.y,trancenter.z));
	Eigen::Vector3d eulerAngle=rotation_vector.matrix().eulerAngles(2,1,0);//2,1,0 yaw pitch roll ZYX (XYZ roll pitch yaw)
	transformSum[0] = 0;//eulerAngle(2);
	transformSum[1] = 0;//eulerAngle(1);
	transformSum[2] = eulerAngle(0);
	yaw = transformSum[2];
	std::cout<<"roll="<<transformSum[0]<<"; yaw="<<transformSum[2]<<"; pitch="<<transformSum[1]<<std::endl;
	
	
	for(int k = 0;k < tran_line_out->points.size();k++)
	{
		TransformToLast1(&tran_line_out->points[k],&pointSell);
		tran_tran_line_out->push_back(pointSell);
	}
	
	PointType tts,tte,ttc;
	tts.x = tran_tran_line_out->points[0].x;
	tts.y = tran_tran_line_out->points[0].y;
	tts.z = tran_tran_line_out->points[0].z;
	
	tte.x = tran_tran_line_out->points[60].x;
	tte.y = tran_tran_line_out->points[60].y;
	tte.z = tran_tran_line_out->points[60].z;
	
	ttc.x = tte.x - tts.x;
	ttc.y = tte.y - tts.y;
	ttc.z = tte.z - tts.z;
	float ttvd;
	ttvd = sqrt(ttc.x * ttc.x + ttc.y * ttc.y + ttc.z * ttc.z);
	ttc.x = ttc.x / ttvd;
	ttc.y = ttc.y / ttvd;
	ttc.z = ttc.z / ttvd;
	std::cout<<"tran tran line vec"<<ttc<<std::endl;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("get_line"));
	int v1(0); 
	MView->createViewPort (0.0, 0.5, 0.5, 1.0, v1); 
	MView->setBackgroundColor (0.3, 0.3, 0.3, v1); 
	MView->addText ("Raw point clouds", 10, 10, "v1_text", v1); 
	int v2(0); 
	MView->createViewPort (0.5, 0.5, 1, 1.0, v2); 
	MView->setBackgroundColor (0.5, 0.5, 0.5, v2); 
	MView->addText ("Tran Raw point clouds", 10, 10, "v2_text", v2); 
 	int v3(0); 
	MView->createViewPort (0.0, 0.0, 0.5, 0.5, v3); 
	MView->setBackgroundColor (0.5, 0.5, 0.5, v3); 
	MView->addText ("line point clouds", 10, 10, "v3_text", v3); 
	int v4(0); 
	MView->createViewPort (0.5, 0.0, 1, 0.5, v4); 
	MView->setBackgroundColor (0.3, 0.3, 0.3, v4); 
	MView->addText ("Tran line point clouds", 10, 10, "v4_text", v4);
	
	MView->addPointCloud<pcl::PointXYZ> (cloud, "cloud",v1);
	MView->addPointCloud<pcl::PointXYZ> (tran_cloud, "tran_cloud",v2);//
	MView->addPointCloud<pcl::PointXYZI> (line_out, "plane",v3);
	MView->addPointCloud<pcl::PointXYZI> (tran_line_out, "tran_plane",v4);//tran_plane
	//MView.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(plane,normals,10,0.1,"plane",v2);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "cloud",v1);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5,0,0.4, "tran_cloud",v2);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "line",v3);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0.5,0.4, "tran_line",v4);
	MView->addCoordinateSystem (7.0);
	MView->initCameraParameters ();
 
	MView->spin();//无限循环
}
int estimateP(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float dis,pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_out,int xuhao)
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

	//原始点云平面提取
    int i = 0, nr_points = (int)cloud->points.size();
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > Points_plane_out;//输出点云
	//std::cout<<nr_points<<std::endl;
    // While 30% of the original cloud is still there
    while (cloud->points.size() > 0.1 * nr_points && i < xuhao)
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
		if(cloud_in->points.size() > 200){
			*plane_out = *cloud_in;
			/*
			int nn = cloud_in->points.size();
			for(int k = 0;k < nn;k++)
			{
				pp.x = cloud_in->points[k].x;
				pp.y = cloud_in->points[k].y;
				pp.y = cloud_in->points[k].z;
				laserCloudScans[i].push_back(pp);
			}
			*/
		}

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_out);
        cloud.swap(cloud_out);//更新
        i++;
    }
	/*
	int x = 0;
	int max = laserCloudScans[0].points.size();
	for(int j = 0;j < i;j++)
	{
		std::cout<<laserCloudScans[j].points.size()<<std::endl;
		if(max < laserCloudScans[j].points.size())
		{
			max = laserCloudScans[j].points.size();
			x = j;
		}
	}
	*plane_out = laserCloudScans[0];
	//清空laserCloudScans[];
	std::cout<<plane_out->points.size()<<std::endl;
	*/
	return 0;
}
int estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane,float re,pcl::PointCloud<pcl::Normal>::Ptr &normals_out)//re = 0.11
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr nor(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	norm.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(plane)); 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm.setSearchMethod (tree);
	norm.setRadiusSearch(re); //设置法线估计的半径 上下两线距离
	norm.compute(*nor); //将法线估计结果保存至normals
	*normals_out = *nor;
	return 0;
}
int unite_direction( PointType * const po,pcl::PointCloud<pcl::PointXYZ>::Ptr &plane,pcl::PointCloud<pcl::Normal>::Ptr &normals,PointType * const ave_nor)//po:方向点
{
	float pd = -1;//正数为正方向，负数为反方向
	int nor_num = normals->points.size();
	float sum_nor[3] = {0};
	PointType pointCha;
	for(size_t i = 0;i <plane->points.size();i++)
	{	
		pointCha.x = po->x - plane->points[i].x;
		pointCha.y = po->y - plane->points[i].y;
		pointCha.z = po->z - plane->points[i].z;
		pd = (pointCha.x * normals->points[i].normal_x + pointCha.y * normals->points[i].normal_y + pointCha.z * normals->points[i].normal_z);
		if(pd < 0){
			normals->points[i].normal_x = -normals->points[i].normal_x;
			normals->points[i].normal_y = -normals->points[i].normal_y;
			normals->points[i].normal_z = -normals->points[i].normal_z;
			//std::cout<<"nor"<<i<<"="<<normals->points[i]<<std::endl;//.x<<","<<normals->points[i].y<<","<<normals->points[i].z<<std::endl;		
		}
		sum_nor[0] += normals->points[i].normal_x;
		sum_nor[1] += normals->points[i].normal_y;
		sum_nor[2] += normals->points[i].normal_z;
		sum_nor[3] += plane->points[i].z;
	}
	ave_nor->x = sum_nor[0] / nor_num;
	ave_nor->y = sum_nor[1] / nor_num;
	ave_nor->z = sum_nor[2] / nor_num;
	return 0;
}
int estimatePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float dis,float roll,float pitch,float h)//输入点云 、距离阈值
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_out (new pcl::PointCloud<pcl::PointXYZ>);//原始输出点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out (new pcl::PointCloud<pcl::PointXYZ>);//平面输出点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out_sec(new pcl::PointCloud<pcl::PointXYZ>);//平面输出点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr  tran_plane_out(new pcl::PointCloud<pcl::PointXYZ>);//旋转平面输出点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr  tran_plane_out_sec(new pcl::PointCloud<pcl::PointXYZ>);//旋转平面输出点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr  tran_points_out(new pcl::PointCloud<pcl::PointXYZ>);//旋转原始输出点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr tran_cloud (new pcl::PointCloud<pcl::PointXYZ>);//旋转点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr tran_tran_plane (new pcl::PointCloud<pcl::PointXYZ>);//旋转回去的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cor_points(new pcl::PointCloud<pcl::PointXYZ>);//边缘点
	pcl::PointCloud<pcl::PointXYZ>::Ptr tran_cor_points(new pcl::PointCloud<pcl::PointXYZ>);//旋转边缘点
	//旋转点云
	for(int k = 0;k < cloud->points.size();k++)
	{
		Transform(&cloud->points[k],&pointSel);
		tran_cloud->push_back(pointSel);
	}
	
	//保存输出点云	
	*points_out = *cloud;
	*tran_points_out = *tran_cloud;
	
	//平面提取
	estimateP(cloud,dis,plane_out,1);
	estimateP(cloud,dis,plane_out_sec,1);
	estimateP(tran_cloud,dis,tran_plane_out,1);
	estimateP(tran_cloud,dis,tran_plane_out_sec,1);
	
			/****************平面匹配******************/
	
 	//法线计算
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::Normal>::Ptr tran_normals(new pcl::PointCloud<pcl::Normal>);
	estimateNormals(plane_out,1.54,normals);
	estimateNormals(tran_plane_out,1.54,tran_normals);
	
	/* 平面提取
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

	//原始点云平面提取
    int i = 0, nr_points = (int)cloud->points.size();
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > Points_plane_out;//输出点云
	//std::cout<<nr_points<<std::endl;
    // While 30% of the original cloud is still there
    while (cloud->points.size() > 0.1 * nr_points)
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
		if(cloud_in->points.size() > 4000){
			*plane_out = *cloud_in;
		}

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_out);
        cloud.swap(cloud_out);//更新
        i++;
    }
	//std::cout<<i<<std::endl;
	
	//创建ling一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr tran_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr tran_inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::SACSegmentation<pcl::PointXYZ> tran_seg;     // 创建一个分割器
    tran_seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    tran_seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory-设置目标几何形状
    tran_seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法
    tran_seg.setDistanceThreshold(dis);         //设置误差容忍范围，也就是我说过的阈值
	
	 // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> tran_extract;
	//旋转点云平面提取
	int l = 0, nr_tran_points = (int)tran_cloud->points.size();
	//std::cout<<"tran_points_nums"<<nr_tran_points<<std::endl;
    // While 30% of the original cloud is still there
    while (tran_cloud->points.size() > 0.1 * nr_tran_points)
    {
        // Segment the largest planar component from the remaining tran_cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr  tran_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr  tran_cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        tran_seg.setInputCloud(tran_cloud);
        tran_seg.segment(*tran_inliers, *tran_coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        tran_extract.setInputCloud(tran_cloud);
        tran_extract.setIndices(tran_inliers);
        tran_extract.setNegative(false);
        tran_extract.filter(*tran_cloud_in);
		if(tran_cloud_in->points.size() > 4000){
			*tran_plane_out = *tran_cloud_in;
		}

        // Create the filtering object
        tran_extract.setNegative(true);
        tran_extract.filter(*tran_cloud_out);
        tran_cloud.swap(tran_cloud_out);//更新
        l++;
    }	
	*/	
	
	/*
	for(int j = 0;j < plane_out->points.size();j++)
	{
		Transform(&plane_out->points[j],&pointSel);
		tran_plane_out->push_back(pointSel);
	}
	*/

	/*法线计算
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	norm.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(plane_out)); 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm.setSearchMethod (tree);
	norm.setRadiusSearch(0.11); //设置法线估计的半径 上下两线距离
	norm.compute(*normals); //将法线估计结果保存至normals
	
	pcl::PointCloud<pcl::Normal>::Ptr tran_normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	norm.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(tran_plane_out)); 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tran_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm.setSearchMethod (tran_tree);
	norm.setRadiusSearch(0.11); //设置法线估计的半径 上下两线距离
	norm.compute(*tran_normals); //将法线估计结果保存至normals
	*/
	
	/*
		//提取边缘点
	pcl::PointCloud<pcl::Normal>::Ptr points_out_normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	norm.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(points_out)); 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr points_out_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm.setSearchMethod (points_out_tree);
	norm.setRadiusSearch(0.06); //设置法线估计的半径 上下两线距离
	norm.compute(*points_out_normals); //将法线估计结果保存至normals
	
	for(int i = 0;i < points_out->points.size();i++){
		if(points_out_normals->points[i].curvature > 0.0001){		
			std::cout<<"nor_c"<<i<<"="<<points_out_normals->points[i].curvature<<std::endl;
			cor_points->push_back(points_out->points[i]);
		}
	}
	*/
					//***************法线定向************//
	
	//pd=n*(p1-p2)>0
	pointOri.x = 0;//p1_x
	pointOri.y = 0;//p1_y
	pointOri.z = 40;//p1_z
	unite_direction(&pointOri,plane_out,normals,&ave_normal);
	unite_direction(&pointOri,tran_plane_out,tran_normals,&ave_tran_normal);
	std::cout<<"ave_nor"<<ave_normal<<std::endl;
	std::cout<<"ave_tran_nor"<<ave_tran_normal<<std::endl;
	/*法线定向及求平均
	float pd = -1;//正数为正方向，负数为反方向
	int nor_num = normals->points.size();
	int tran_nor_num = tran_normals->points.size();
	float sum_nor[4] = {0};
	float sum_tran_nor[4] = {0};
	float ave_H,ave_tran_H;
	for(size_t i = 0;i <plane_out->points.size();i++)
	{	
		pointCha.x = pointOri.x - plane_out->points[i].x;
		pointCha.y = pointOri.y - plane_out->points[i].y;
		pointCha.z = pointOri.z - plane_out->points[i].z;
		pd = (pointCha.x * normals->points[i].normal_x + pointCha.y * normals->points[i].normal_y + pointCha.z * normals->points[i].normal_z);
		if(pd < 0){
			normals->points[i].normal_x = -normals->points[i].normal_x;
			normals->points[i].normal_y = -normals->points[i].normal_y;
			normals->points[i].normal_z = -normals->points[i].normal_z;
			//std::cout<<"nor"<<i<<"="<<normals->points[i]<<std::endl;//.x<<","<<normals->points[i].y<<","<<normals->points[i].z<<std::endl;		
		}
		sum_nor[0] += normals->points[i].normal_x;
		sum_nor[1] += normals->points[i].normal_y;
		sum_nor[2] += normals->points[i].normal_z;
		sum_nor[3] += plane_out->points[i].z;
	}
	ave_normal.x = sum_nor[0] / nor_num;
	ave_normal.y = sum_nor[1] / nor_num;
	ave_normal.z = sum_nor[2] / nor_num;
	ave_H = sum_nor[3] / nor_num;
	std::cout<<"ave_nor"<<ave_normal<<std::endl;
		
	for(size_t i = 0;i <tran_plane_out->points.size();i++)
	{
		pointCha.x = pointOri.x - tran_plane_out->points[i].x;
		pointCha.y = pointOri.y - tran_plane_out->points[i].y;
		pointCha.z = pointOri.z - tran_plane_out->points[i].z;
		pd = (pointCha.x * tran_normals->points[i].normal_x + pointCha.y * tran_normals->points[i].normal_y + pointCha.z * tran_normals->points[i].normal_z);
		if(pd < 0){
			tran_normals->points[i].normal_x = -tran_normals->points[i].normal_x;
			tran_normals->points[i].normal_y = -tran_normals->points[i].normal_y;
			tran_normals->points[i].normal_z = -tran_normals->points[i].normal_z;
			//std::cout<<"nor"<<i<<"="<<tran_normals->points[i]<<std::endl;//.x<<","<<tran_normals->points[i].y<<","<<tran_normals->points[i].z<<std::endl;		
		}
		sum_tran_nor[0] += tran_normals->points[i].normal_x;
		sum_tran_nor[1] += tran_normals->points[i].normal_y;
		sum_tran_nor[2] += tran_normals->points[i].normal_z;
		sum_tran_nor[3] += tran_plane_out->points[i].z;
	}
	ave_tran_normal.x = sum_tran_nor[0] / tran_nor_num;
	ave_tran_normal.y = sum_tran_nor[1] / tran_nor_num;
	ave_tran_normal.z = sum_tran_nor[2] / tran_nor_num;
	std::cout<<"ave_tran_nor"<<ave_tran_normal<<std::endl;
	*/
					//*****************计算旋转量*****************//
	//罗德里格公式
	//旋转轴为两向量叉乘的单位向量
	//旋转角度为点乘
	float alpha = 0;//旋转角度
	float guiyi;
	PointType trancenter;//旋转轴
	alpha = acos(ave_tran_normal.x * ave_normal.x + ave_tran_normal.y * ave_normal.y + ave_tran_normal.z * ave_normal.z);
	trancenter.x = ave_normal.y * ave_tran_normal.z - ave_tran_normal.y * ave_normal.z ;
	trancenter.y = -(ave_normal.x * ave_tran_normal.z - ave_tran_normal.x * ave_normal.z);
	trancenter.z = ave_normal.x * ave_tran_normal.y - ave_tran_normal.x * ave_normal.y ;
	guiyi = sqrt(trancenter.x * trancenter.x + trancenter.y * trancenter.y + trancenter.z * trancenter.z);
	trancenter.x = trancenter.x / guiyi;
	trancenter.y = trancenter.y / guiyi;
	trancenter.z = trancenter.z / guiyi;
	std::cout<<"旋转轴"<<trancenter<<"; ";
	std::cout<<"旋转角度"<<alpha<<std::endl;
	//旋转向量转化为欧拉角
	Eigen::AngleAxisd rotation_vector (alpha,Eigen::Vector3d (trancenter.x,trancenter.y,trancenter.z));
	Eigen::Vector3d eulerAngle=rotation_vector.matrix().eulerAngles(2,1,0);//2,1,0 yaw pitch roll ZYX (XYZ roll pitch yaw)
	

	
	transformSum[0] = eulerAngle(2);
	transformSum[1] = eulerAngle(1);
	//transformSum[2] = 0;//eulerAngle(0);
	std::cout<<"roll="<<transformSum[0]<<"; yaw="<<transformSum[2]<<"; pitch="<<transformSum[1]<<std::endl;
	roll = transformSum[0];
	pitch = transformSum[1];
	//局部坐标转到全局坐标
	for(int k = 0;k < tran_plane_out->points.size();k++)
	{
		TransformToLast(&tran_plane_out->points[k],&pointSel);
		tran_tran_plane->push_back(pointSel);
	}
	
	pcl::PointCloud<pcl::Normal>::Ptr tran_tran_normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	estimateNormals(tran_tran_plane,1.54,tran_tran_normals);
	unite_direction(&pointOri,tran_tran_plane,tran_tran_normals,&ave_tran_tran_normal);
	std::cout<<"ave_tran_tran_nor"<<ave_tran_tran_normal<<std::endl;
	
	int plane_number = plane_out->points.size();
	float distance = 0;
	for(int i = 0;i < plane_number;i++)
	{
		distance += plane_out->points[i].z;
	}
	float ave_dis = distance / plane_number;
	
	int tran_plane_number = tran_tran_plane->points.size();
	float tran_distance = 0;
	for(int i = 0;i < tran_plane_number;i++)
	{
		tran_distance += tran_tran_plane->points[i].z;
	}
	float tran_ave_dis = tran_distance / tran_plane_number;
	
	h = tran_ave_dis - ave_dis;
	std::cout<<"dis="<<ave_dis<<";"<<"tran_dis="<<tran_ave_dis<<std::endl;
	
	/*
	pcl::PointCloud<pcl::Normal>::Ptr tran_tran_normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	norm.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(tran_tran_plane)); 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tran_tran_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm.setSearchMethod (tran_tran_tree);
	norm.setRadiusSearch(0.11); //设置法线估计的半径 上下两线距离
	norm.compute(*tran_tran_normals); //将法线估计结果保存至normals
	*/
	//for(int i = 0;i < tran_tran_normals->points.size();i = i + 40){
	//	std::cout<<"tran_tran_nor"<<i<<"="<<tran_tran_normals->points[i]<<";    ";
	//	std::cout<<"tran_tran_plane_H"<<i<<"="<<tran_tran_plane->points[i].z<<std::endl;
	//}
	
	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("get_plane"));
	
	int v1(0); 
	MView->createViewPort (0.0, 0.5, 0.5, 1.0, v1); 
	MView->setBackgroundColor (0.3, 0.3, 0.3, v1); 
	MView->addText ("Raw point clouds", 10, 10, "v1_text", v1); 
	int v2(0); 
	MView->createViewPort (0.5, 0.5, 1, 1.0, v2); 
	MView->setBackgroundColor (0.5, 0.5, 0.5, v2); 
	MView->addText ("Tran Raw point clouds", 10, 10, "v2_text", v2); 
 	int v3(0); 
	MView->createViewPort (0.0, 0.0, 0.5, 0.5, v3); 
	MView->setBackgroundColor (0.5, 0.5, 0.5, v3); 
	MView->addText ("plane point clouds", 10, 10, "v3_text", v3); 
	int v4(0); 
	MView->createViewPort (0.5, 0.0, 1, 0.5, v4); 
	MView->setBackgroundColor (0.3, 0.3, 0.3, v4); 
	MView->addText ("Tran plane point clouds", 10, 10, "v4_text", v4);
	
	MView->addPointCloud<pcl::PointXYZ> (points_out, "cloud",v1);
	MView->addPointCloud<pcl::PointXYZ> (tran_points_out, "tran_cloud",v2);//
	MView->addPointCloud<pcl::PointXYZ> (plane_out, "plane",v3);
	MView->addPointCloud<pcl::PointXYZ> (tran_tran_plane, "tran_plane",v4);//tran_plane
	//MView.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(plane,normals,10,0.1,"plane",v2);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "cloud",v1);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5,0,0.4, "tran_cloud",v2);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "plane",v3);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0.5,0.4, "tran_plane",v4);
	MView->addCoordinateSystem (14.0);
	MView->initCameraParameters ();
 
	MView->spin();//无限循环
	
	pcl::visualization::PCLVisualizer viewer("plane normals");
	viewer.setBackgroundColor (0.3, 0.3, 0.3);
	//viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(plane,normals,1,0.2);
	viewer.addCoordinateSystem(14.0);
	viewer.addPointCloud(points_out,"points");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "points");

	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(plane_out,normals,10,3,"normals");
	while (!viewer.wasStopped ())
	{
	     viewer.spinOnce ();
	}
	

	pcl::visualization::PCLVisualizer tran_viewer("tran plane normals");
	tran_viewer.setBackgroundColor (0.3, 0.3, 0.3);
	//viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(plane,normals,1,0.2);
	tran_viewer.addCoordinateSystem(14.0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(tran_points_out, 199,21,133);//0-255  设置成绿色
	//viewer.addPointCloud<pcl::PointXYZ>(Cloud, singleColor, "sample");//显示点云，其中fildColor为颜色显示
	tran_viewer.addPointCloud(tran_points_out,singleColor,"tran_points");
	tran_viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(tran_plane_out,tran_normals,10,3,"tran_normals");
	while (!tran_viewer.wasStopped ())
	{
	     tran_viewer.spinOnce ();
	}

	//	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(Cloud, "z");//按照z字段进行渲染
	//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(Cloud, 0,255,0);//0-255  设置成绿色
	//	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(Cloud);//随机给点云生成颜色
	return 0;
}
void showTheCloud()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//load 
    pcl::io::loadPCDFile ("pointCloudFile.pcd", *cloud); 
 	float r,y,p,h;
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (100);//寻找每个点的50个最近邻点
    sor.setStddevMulThresh (1.0);//一个点的最近邻距离超过全局平均距离的一个标准差以上，就会舍弃
    sor.filter (*cloud_filtered);*/
//	estimateBorders(cloud,1,1);
	estimateLine(cloud,y);
 	estimatePlane(cloud,0.0015,r,p,h);
	std::cout<<"roll="<<r<<"; yaw="<<y<<"; pitch="<<p<<"; height="<<h<<std::endl;
	
    ///pcl::visualization::CloudViewer viewer("Cloud Viewer qj");
 
    //blocks until the cloud is actually rendered
    ///viewer.showCloud(cloud);
 
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
 
    //This will only get called once
    ///viewer.runOnVisualizationThreadOnce (viewerOneOff);
 
    //This will get called once per visualization iteration
     //viewer.runOnVisualizationThread (viewerPsycho);
    ///while (!viewer.wasStopped ())
    ///{
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        //user_data++;
   ///}
}
int main ()
{
	if(saveThePointCloud())
	{
		showTheCloud();
	}
	else
	{
		std::cout<<"Failed"<<endl;
	}
    return 0;
}