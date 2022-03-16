


void ICP_COV(pcl::PointCloud<pcl::PointXYZ>::Ptr& source_pc, pcl::PointCloud<pcl::PointXYZ>::Ptr& target_pc, Eigen::Matrix4f& transform, Eigen::MatrixXd& cov)
{

		std::cout << "source_pc->points.size(): " << source_pc->points.size() << std::endl;
		std::cout << "target_pc->points.size(): " << target_pc->points.size() << std::endl;

    double Tx = transform(0,3);
    double Ty = transform(1,3);
    double Tz = transform(2,3);
    double roll  = atan2f(transform(2,1), transform(2,2));
    double pitch = asinf(-transform(2,0));
    double yaw   = atan2f(transform(1,0), transform(0,0));

		pcl::PointCloud<pcl::PointXYZ>::Ptr source_pc_align (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*source_pc, *source_pc_align, transform);

    pcl::Correspondences correspondeces_reciprocal_shot;
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;

    corr_est.setInputSource(source_pc_align);
    corr_est.setInputTarget(target_pc);
    corr_est.determineReciprocalCorrespondences(correspondeces_reciprocal_shot);
		std::cout << "No. of Reciprocal Correspondences : " << correspondeces_reciprocal_shot.size() << std::endl;

    std::vector<int> source_idx;
    std::vector<int> target_idx;

    pcl::Correspondence temp;
    for (int i = 0; i < correspondeces_reciprocal_shot.size(); i++)
    {
        temp = correspondeces_reciprocal_shot[i];
        source_idx.push_back(temp.index_query);
        target_idx.push_back(temp.index_match);
    }

    pcl::PointCloud<pcl::PointXYZ> source_pi;
    pcl::PointCloud<pcl::PointXYZ> target_qi;

    pcl::copyPointCloud(*source_pc, source_idx, source_pi);
    pcl::copyPointCloud(*target_pc, target_idx, target_qi);

		// Y = M * X
		Eigen::MatrixXd Y_mat(source_pi.points.size()*2, 1);
		Eigen::MatrixXd M_mat(source_pi.points.size()*2, 3);
		Eigen::MatrixXd X_mat(3, 1);

		X_mat(0,0) = Tx;
		X_mat(1,0) = Ty;
		X_mat(2,0) = yaw;

    for (size_t s = 0; s < source_pi.points.size(); ++s )
    {
        double pix = source_pi.points[s].x;
        double piy = source_pi.points[s].y;
        double piz = source_pi.points[s].z;
        double qix = target_qi.points[s].x;
        double qiy = target_qi.points[s].y;
        double qiz = target_qi.points[s].z;

				Y_mat(2*s, 0) = qix - pix;
				Y_mat(2*s+1, 0) = qiy - piy;
				M_mat(2*s, 0) = 1;	M_mat(2*s, 1) = 0;	M_mat(2*s, 2) = -piy;
				M_mat(2*s+1, 0) = 0;	M_mat(2*s+1, 1) = 1;	M_mat(2*s+1, 2) = pix;
		}

		Eigen::MatrixXd cov_mat;
		Eigen::MatrixXd Err_mat = M_mat * X_mat - Y_mat; // MX-Y ( 2N x 1 )
		Eigen::MatrixXd cov_mat_scalar = ((Err_mat.transpose()*Err_mat) / (source_pi.points.size()*2 - 3));
		cov_mat = cov_mat_scalar(0,0)*(M_mat.transpose()*M_mat).inverse();

    std::cout << "cov_mat: \n" << cov_mat <<std::endl;
}


