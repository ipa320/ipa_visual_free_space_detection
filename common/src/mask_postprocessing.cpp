#ifndef POSTPROCESSING_CPP
#define POSTPROCESSING_CPP
#include <mask_postprocessing.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

//look up all 3D points belonging to free space of 2D mask
void MaskPostprocessing::compute_3D_FreeSpace_PointCloud(
		const cv::Mat &r_mask_32F,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_points,
		pcl::PointCloud<pcl::PointXYZ> &freeSpace_3D_pointCloud) {
	long unsigned int time = m_ptr_common->get_time_ms64();
	int width_i = r_mask_32F.cols;
	int height_i = r_mask_32F.rows;

	if (input_points->empty()) {
		ROS_ERROR("input_points EMPTY");
	}
	freeSpace_3D_pointCloud = pcl::PointCloud<pcl::PointXYZ>();

	int start_Y = 0;
	if (m_ptr_common->isExtractRoiB()) {
		start_Y = height_i;
		height_i *= 2;
	}
	for (int v = start_Y; v < height_i; ++v) {
		for (int u = 0; u < width_i; ++u) {
			//get image index of (u,v)
			int index = v * width_i + u;
			//look up 3D coordinate
			pcl::PointXYZ point3D = input_points->at(index);

			//free space with check for NaN
			if (point3D.x == point3D.x
					&& r_mask_32F.at<float>(v - start_Y, u) == 1) {
				freeSpace_3D_pointCloud.push_back(point3D);
			}
		}
	}
	if (m_ptr_common->isPrintTimingsB()) {
		ROS_INFO("compute 3D free space points: %lu ms",
				m_ptr_common->get_time_ms64() - time);
	}
	if (m_ptr_common->isDebugB()) {
		std::cout << "3D free space points -> "
				<< freeSpace_3D_pointCloud.size() << "\n";
	}
}

void MaskPostprocessing::concaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &output) {

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
			new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(input));

	pcl::RandomSampleConsensus < pcl::PointXYZ > ransac(model_p);
	ransac.setDistanceThreshold(.01);
	ransac.computeModel();
	ransac.getInliers(inliers);

	printf("inliers -> %d\n", inliers.at(0));

	// copies all inliers of the model computed to another PointCloud
	//pcl::copyPointCloud < pcl::PointXYZ > (*cloud, inliers, *final);
	pcl::copyPointCloud < pcl::PointXYZ > (*input, inliers, *output);

	/*
	 pcl::ConcaveHull < pcl::PointXYZ > chull;
	 chull.setInputCloud(input);
	 chull.setAlpha(0.1);
	 chull.reconstruct(*output);
	 */
}

//convert color free space mask 8UC3 into binary free space mask 32FC1: if color of region is NOT black, then freespace, else obstacle
cv::Mat MaskPostprocessing::create_binary_segment_mask(
		const cv::Mat &r_mask_8UC3) {
	long unsigned int time = m_ptr_common->get_time_ms64();
	int width_i = r_mask_8UC3.cols;
	int height_i = r_mask_8UC3.rows;
	cv::Mat binaryMask = cv::Mat::zeros(r_mask_8UC3.size(), CV_32FC1);
	int borderWidth_i = 0;
	for (int row = borderWidth_i; row < height_i - borderWidth_i; ++row) {
		for (int col = borderWidth_i; col < width_i - borderWidth_i; ++col) {
			if (r_mask_8UC3.at<cv::Vec3b>(row, col) != cv::Vec3b(0, 0, 0)) {
				binaryMask.at<float>(row, col) = 1.0f;
			}
		}
	}
	if (m_ptr_common->isPrintTimingsB()) {
		ROS_INFO("create binary free space mask: %lu ms",
				m_ptr_common->get_time_ms64() - time);
	}
	return binaryMask;
}

#endif
