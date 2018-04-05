#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
//ICP library
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
//PCA library
#include <pcl/common/pca.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
//TF library
#include <tf/transform_broadcaster.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
ros::Publisher point_map_publisher;
ros::Publisher point_scene_publisher;
ros::Publisher path_vis_publisher;
void registration_vis(PointCloudXYZRGB::Ptr scene, PointCloudXYZRGB::Ptr map);
Eigen::Matrix4f initial_alignment(PointCloudXYZRGB::Ptr scene, PointCloudXYZRGB::Ptr map);
Eigen::Matrix4f icp_mapping(PointCloudXYZRGB::Ptr scene, PointCloudXYZRGB::Ptr map);
//Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();


int main(int argc, char** argv){
	//load  pointcloud
	PointCloudXYZRGB::Ptr scene (new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr initial_transform_scene (new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr map (new PointCloudXYZRGB);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/michael/Downloads/PointCloud_v2/scene.pcd", *scene);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/michael/Downloads/PointCloud_v2/map.pcd", *map);
	//initialize ros node
	ros::init (argc, argv, "hw4_node");
	ros::NodeHandle nh("~");
	point_map_publisher = nh.advertise<sensor_msgs::PointCloud2>("map_point",1);
	point_scene_publisher = nh.advertise<sensor_msgs::PointCloud2>("scene_point",1);
	path_vis_publisher = nh.advertise<visualization_msgs::Marker>("PCA", 10);
	//Initial transform.
	Eigen::Matrix4f initial_transform_matrix = initial_alignment(scene, map);

	pcl::transformPointCloud (*scene, *initial_transform_scene, initial_transform_matrix);
	Eigen::Matrix4f icp_matrix = icp_mapping(initial_transform_scene, map);
	icp_matrix = icp_matrix*initial_transform_matrix;
	printf("Transform matrix:\n");
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			printf("%lf ", icp_matrix(i,j));
		}
		printf("\n");
	}

	//TF
	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform tf_matrix;
	tf_matrix.setOrigin(tf::Vector3(icp_matrix(0,3), icp_matrix(1,3), icp_matrix(2,3)) );
	tf_matrix.setBasis( tf::Matrix3x3(   icp_matrix(0,0),icp_matrix(0,1),icp_matrix(0,2),icp_matrix(1,0),icp_matrix(1,1),icp_matrix(1,2),icp_matrix(2,0),icp_matrix(2,1),icp_matrix(2,2)) );


	while (1){
		tf_broadcaster.sendTransform(tf::StampedTransform(tf_matrix,ros::Time::now(),"/map", "/scene"));
		registration_vis(scene,map);
	}
}
Eigen::Matrix4f initial_alignment(PointCloudXYZRGB::Ptr scene, PointCloudXYZRGB::Ptr map){
	Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();

	Eigen::Vector4f centroid_map, centroid_scene;
    pcl::compute3DCentroid (*map, centroid_map);
    pcl::compute3DCentroid (*scene, centroid_scene);
	printf("Scene size: %d, Map size: %d\n",scene->points.size(),map->points.size());
	printf("Centroid_scene: %lf %lf %lf, Centroid_map: %lf %lf %lf\n",centroid_scene[0],centroid_scene[1],centroid_scene[2],centroid_map[0],centroid_map[1],centroid_map[2]);
	//Initial transition
	pcl::compute3DCentroid (*map, centroid_map);
    pcl::compute3DCentroid (*scene, centroid_scene);
	printf("Centroid_scene: %lf %lf %lf, Centroid_map: %lf %lf %lf\n",centroid_scene[0],centroid_scene[1],centroid_scene[2],centroid_map[0],centroid_map[1],centroid_map[2]);
	initial_transform(0,3) = centroid_map[0] - centroid_scene[0];
	initial_transform(1,3) = centroid_map[1] - centroid_scene[1];
	initial_transform(2,3) = centroid_map[2] - centroid_scene[2];

	//Intial Rotation PCA for point cloud 
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::PCA<pcl::PointXYZRGB> cpca = new pcl::PCA<pcl::PointXYZRGB>;
	Eigen::Matrix3f pca_vector(3,3);
	Eigen::Vector3f pca_values(3);
	cpca.setInputCloud(map);
	pca_vector = cpca.getEigenVectors();
	pca_values = cpca.getEigenValues();
	printf ("Map\n");
	for(int i=0;i<3;i++){
		printf("Eigen value: %lf Eigen vector:",pca_values[i]);
		for(int j=0;j<3;j++){
			printf("%lf ", pca_vector(i,j));
		}
		printf("\n");
	}
	double theta_map = atan2(pca_vector(1,1),pca_vector(1,0));

	Eigen::Matrix3f pca_vector2(3,3);
	Eigen::Vector3f pca_values2(3);
	cpca.setInputCloud(scene);
	pca_vector2 = cpca.getEigenVectors();
	pca_values2 = cpca.getEigenValues();
	printf ("Scene\n");
	for(int i=0;i<3;i++){
		printf("Eigen value: %lf Eigen vector:",pca_values2[i]);
		for(int j=0;j<3;j++){
			printf("%lf ", pca_vector2(i,j));
		}
		printf("\n");
	}

	double theta_scene = atan2(-pca_vector2(0,1),-pca_vector2(0,0));
	printf("theta_map:%lf theta_scene:%lf\n",theta_map,theta_scene);
	double yaw = theta_map - theta_scene;

	initial_transform(0,0) = 0.7;//cos(yaw);
	initial_transform(0,1) = 0.7;//-sin(yaw);
	initial_transform(1,0) = -0.7;//sin(yaw);
	initial_transform(1,1) = 0.7;//cos(yaw);
	printf("Initial matrix:\n");
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			printf("%lf ", initial_transform(i,j));
		}
		printf("\n");
	}
	return initial_transform;
	/*
	//Visualization pca
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
	line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1; 
    line_list.scale.y = 0.1;
    line_list.color.b=1.0;
    line_list.color.a=1.0;
    for (int i=0;i<3;i++){
    	geometry_msgs::Point p1; 
	    p1.x = centroid_map[0];
	    p1.y = centroid_map[1];
	    p1.z = centroid_map[2];
	    geometry_msgs::Point p2;
	    p2.x = centroid_map[0]+pca_vector(i,0)*10;
	    p2.y = centroid_map[1]+pca_vector(i,1)*10;
	    p2.z = centroid_map[2]+pca_vector(i,2)*10;
	    // The line list needs two points for each line
	    line_list.points.push_back(p1);
	    line_list.points.push_back(p2);
    }
    path_vis_publisher.publish(line_list);
    */
	//Initial alignment

}


Eigen::Matrix4f icp_mapping(PointCloudXYZRGB::Ptr scene, PointCloudXYZRGB::Ptr map){
	
	//ICP

	float sample_rate = 0.08;
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	PointCloudXYZRGB::Ptr down_sample_map =map;//(new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr cloud_reg (new PointCloudXYZRGB);
	Eigen::Matrix4f transformation;
	Eigen::Matrix4f output_transform = Eigen::Matrix4f::Identity();
	sor.setInputCloud (map);
  	sor.setLeafSize (sample_rate, sample_rate, sample_rate);
	sor.filter (*down_sample_map);
	pcl::copyPointCloud (*scene,*cloud_reg);
	/*for (int iteration=0;iteration< 20;iteration++){
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);				
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; 
		tree1->setInputCloud(cloud_reg);																				
		tree2->setInputCloud(down_sample_map);																			
		icp.setSearchMethodSource(tree1);																			//
		icp.setSearchMethodTarget(tree2);																			//
		icp.setInputSource(cloud_reg);				// Set align model													//		
		icp.setInputTarget(down_sample_map);		// Set align target													//
		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)													//
		icp.setMaxCorrespondenceDistance(10);																		
		icp.setTransformationEpsilon(1e-9);	// Set the transformation epsilon (criterion 1)																//
		icp.setEuclideanFitnessEpsilon(1);	// Set the euclidean distance difference epsilon (criterion 2)																	//
		icp.setMaximumIterations(500);			// Set the maximum number of iterations (criterion 3)																	//
		 icp.setRANSACOutlierRejectionThreshold (0.05);
		icp.align(*cloud_reg);	
		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;																					
		transformation = icp.getFinalTransformation(); 
		output_transform = output_transform * transformation;
	}*/
	
	for (int iteration=0;iteration< 20;iteration++){
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);				
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; 
		tree1->setInputCloud(cloud_reg);																				
		tree2->setInputCloud(down_sample_map);																			
		icp.setSearchMethodSource(tree1);																			//
		icp.setSearchMethodTarget(tree2);																			//
		icp.setInputSource(cloud_reg);				// Set align model													//		
		icp.setInputTarget(down_sample_map);		// Set align target													//
		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)													//
		icp.setMaxCorrespondenceDistance(5);																		
		icp.setTransformationEpsilon(1e-9);	// Set the transformation epsilon (criterion 1)																//
		icp.setEuclideanFitnessEpsilon(1);	// Set the euclidean distance difference epsilon (criterion 2)																	//
		icp.setMaximumIterations(100);			// Set the maximum number of iterations (criterion 3)																	//
		 icp.setRANSACOutlierRejectionThreshold (0.07);
		icp.align(*cloud_reg);	
		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;																					
		transformation = icp.getFinalTransformation(); 
		output_transform = output_transform * transformation;
	}

	
	return output_transform;
}


void registration_vis(PointCloudXYZRGB::Ptr scene, PointCloudXYZRGB::Ptr map){

	float sample_rate = 0.5;
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (map);
  	sor.setLeafSize (sample_rate, sample_rate, sample_rate);
	sor.filter (*map);
	//printf("size: %d",scene->points.size());
	for (int i=0;i<scene->points.size();i++){
		scene->points[i].r=255.0;
		scene->points[i].g=0.0;
		scene->points[i].b=0.0;
	}
	for (int i=0;i<map->points.size();i++){
		map->points[i].r=0.0;
		map->points[i].g=255.0;
		map->points[i].b=0.0;
	}
	sensor_msgs::PointCloud2 vis_scene_point, vis_map_point;
	pcl::toROSMsg(*scene,vis_scene_point);
	pcl::toROSMsg(*map,vis_map_point);
	vis_scene_point.header.frame_id = "/scene";
	vis_map_point.header.frame_id = "/map";
	point_map_publisher.publish(vis_map_point);
	point_scene_publisher.publish(vis_scene_point);




	usleep(100);
}

