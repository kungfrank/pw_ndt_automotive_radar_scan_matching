#include <ndt_gpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <iostream>
#include <thread>
#include <pthread.h>

#include <chrono>

//using namespace std::literals::chrono_literals;

static std::shared_ptr<gpu::GNormalDistributionsTransform> anh_gpu_ndt_ptr =
    std::make_shared<gpu::GNormalDistributionsTransform>();

int main() {

    //// Loading Scans ////
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/joinet/Downloads/room_scan1.pcd", *target_cloud) == -1)
		{
		  PCL_ERROR ("Couldn't read file <target>.pcd \n");
		  return (-1);
		}
		std::cout << "Loaded " << target_cloud->size () << " data points from <target>.pcd" << std::endl;
		// Loading second scan of room from new perspective.
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/joinet/Downloads/room_scan2.pcd", *input_cloud) == -1)
		{
		  PCL_ERROR ("Couldn't read file <input>.pcd \n");
		  return (-1);
		}
		std::cout << "Loaded " << input_cloud->size () << " data points from <input>.pcd" << std::endl;

    // Set initial alignment estimate found using robot odometry
    /*.
    Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    */
    Eigen::Matrix4f init_guess(Eigen::Matrix4f::Identity());

    std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start,
        getFitnessScore_end;
    static bool has_converged;
    static int iteration = 0;
    static double fitness_score = 0.0;
    static double trans_probability = 0.0;
    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());


    ///--- NDT GPU ---///
    anh_gpu_ndt_ptr->setInputSource(input_cloud);
    anh_gpu_ndt_ptr->setInputTarget(target_cloud);

    anh_gpu_ndt_ptr->setResolution(1.0);
    anh_gpu_ndt_ptr->setMaximumIterations(35);
    anh_gpu_ndt_ptr->setStepSize(0.1);
    anh_gpu_ndt_ptr->setTransformationEpsilon(0.01);

    align_start = std::chrono::system_clock::now();
    anh_gpu_ndt_ptr->align(init_guess);
    align_end = std::chrono::system_clock::now();

    has_converged = anh_gpu_ndt_ptr->hasConverged();
    t = anh_gpu_ndt_ptr->getFinalTransformation();
    iteration = anh_gpu_ndt_ptr->getFinalNumIteration();

    getFitnessScore_start = std::chrono::system_clock::now();
    fitness_score = anh_gpu_ndt_ptr->getFitnessScore();
    getFitnessScore_end = std::chrono::system_clock::now();

    trans_probability = anh_gpu_ndt_ptr->getTransformationProbability();

    cout << "has_converged: " << has_converged << " iteration: " << iteration << " fitness_score: " << fitness_score << endl;
    cout << "t: \n" << t << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*input_cloud, *output_cloud, t);

    double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
    double getFitnessScore_time = std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() / 1000.0;

    cout << "align_time: " << align_time << " getFitnessScore_time: " << getFitnessScore_time << endl;

    ///--- PCL P2D NDT ---///
/*
		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
		ndt.setTransformationEpsilon (0.01);
		ndt.setStepSize (0.1);
		ndt.setResolution (1.0);
		ndt.setMaximumIterations (35);
		ndt.setInputSource (input_cloud);
		ndt.setInputTarget (target_cloud);

		// Calculating required rigid transform to align the input cloud to the target cloud.
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    align_start = std::chrono::system_clock::now();
		ndt.align (*output_cloud, init_guess);
    align_end = std::chrono::system_clock::now();

    has_converged = ndt.hasConverged();
    t = ndt.getFinalTransformation();
    iteration = ndt.getFinalNumIteration();

    getFitnessScore_start = std::chrono::system_clock::now();
    fitness_score = ndt.getFitnessScore();
    getFitnessScore_end = std::chrono::system_clock::now();

    trans_probability = anh_gpu_ndt_ptr->getTransformationProbability();

    cout << "has_converged: " << has_converged << " iteration: " << iteration << " fitness_score: " << fitness_score << endl;
    cout << "t: \n" << t << endl;

    pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

    double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
    double getFitnessScore_time = std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() / 1000.0;

    cout << "align_time: " << align_time << " getFitnessScore_time: " << getFitnessScore_time << endl;
*/

    ///--- Vizualization ---///

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
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		return (0);

}
