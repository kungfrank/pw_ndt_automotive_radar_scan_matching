#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_p2d.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
//#include <ndt_generic/pcl_utils.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std::chrono_literals;


int main() {

		// Loading first scan of room.
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/joinet/Downloads/radar_pc_0.pcd", *target_cloud) == -1)
		{
		  PCL_ERROR ("Couldn't read file <target>.pcd \n");
		  return (-1);
		}
		std::cout << "Loaded " << target_cloud->size () << " data points from <target>.pcd" << std::endl;
		// Loading second scan of room from new perspective.
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/joinet/Downloads/radar_pc_5.pcd", *input_cloud) == -1)
		{
		  PCL_ERROR ("Couldn't read file <input>.pcd \n");
		  return (-1);
		}
		std::cout << "Loaded " << input_cloud->size () << " data points from <input>.pcd" << std::endl;

///--- ROS D2D NDT ---///
/*
		//// make pc become 3D ////
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*target_cloud, *cloud_a);
		for(int k=1; k<10; k++){
			for(int i=0 ; i<cloud_a->size(); i++){
					cloud_a->points[i].z = k;
			}
			*target_cloud = *cloud_a + *target_cloud;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*input_cloud, *cloud_b);
		for(int k=1; k<10; k++){
			for(int i=0 ; i<cloud_b->size(); i++){
					cloud_b->points[i].z = k;
			}
			*input_cloud = *cloud_b + *input_cloud;
		}
		//// make pc become 3D ////
*/

    double __res[] = {3};
    std::vector<double> resolutions (__res, __res+sizeof(__res)/sizeof(double));
    lslgeneric::NDTMatcherD2D_2D matcherD2D (false,false,resolutions); //,false,false
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
    //... load information into cloud1 and cloud2 ...
    bool ret = matcherD2D.match(*target_cloud,*input_cloud,T);
    //we now have T, which minimizes |cloud1 - cloud2*T| 
    /////////////////////perception_oru::transformPointCloudInPlace<pcl::PointXYZ>(T,cloud2);
		std::cout << "ret: " << ret << std::endl;
		std::cout << "T: " << T(0,0) << T(0,1) << T(0,2) << " " << T(0,3) << std::endl;
		std::cout << "T: " << T(1,0) << T(1,1) << T(1,2) << " " << T(1,3) << std::endl;
		std::cout << "T: " << T(2,0) << T(2,1) << T(2,2) << " " << T(2,3) << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud (*input_cloud, *output_cloud, T.matrix());


///--- PCL P2D NDT ---///
/*
		// Initializing Normal Distributions Transform (NDT).
		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
		// Setting scale dependent NDT parameters
		// Setting minimum transformation difference for termination condition.
		ndt.setTransformationEpsilon (0.01);
		// Setting maximum step size for More-Thuente line search.
		ndt.setStepSize (0.1);
		//Setting Resolution of NDT grid structure (VoxelGridCovariance).
		ndt.setResolution (1.0);
		// Setting max number of registration iterations.
		ndt.setMaximumIterations (35);
		// Setting point cloud to be aligned.
		ndt.setInputSource (input_cloud);
		// Setting point cloud to be aligned to.
		ndt.setInputTarget (target_cloud);

		// Set initial alignment estimate found using robot odometry.
		Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
		Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
		Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

		// Calculating required rigid transform to align the input cloud to the target cloud.
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		ndt.align (*output_cloud, init_guess);

		std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
		          << " score: " << ndt.getFitnessScore () << std::endl;

		// Transforming unfiltered, input cloud using found transform.
		pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
*/

		// Initializing point cloud visualizer
		pcl::visualization::PCLVisualizer::Ptr
		viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer_final->setBackgroundColor (0, 0, 0);
		// Coloring and visualizing target cloud (red).
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		target_color (target_cloud, 255, 0, 0);
		viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
		viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		                                                1, "target cloud");
		// Coloring and visualizing transformed input cloud (green).
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		output_color (output_cloud, 0, 255, 0);
		viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
		viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		                                                1, "output cloud");

		// Coloring and visualizing transformed input cloud (blue).
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		input_color (output_cloud, 0, 0, 255);
		viewer_final->addPointCloud<pcl::PointXYZ> (input_cloud, input_color, "input cloud");
		viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		                                                1, "input cloud");
		// Starting visualizer
		viewer_final->addCoordinateSystem (1.0, "global");
		viewer_final->initCameraParameters ();

		// Wait until visualizer window is closed.
		while (!viewer_final->wasStopped ())
		{
		  viewer_final->spinOnce (100);
		  std::this_thread::sleep_for(100ms);
		}

		return (0);

}
