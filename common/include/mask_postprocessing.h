#ifndef POSTPROCESSING_H
#define POSTPROCESSING_H

class MaskPostprocessing {
private:
	FreespaceCommon* m_ptr_common;

public:
	MaskPostprocessing(FreespaceCommon* &r_ptr_common) :
			m_ptr_common(r_ptr_common) {
	}
	~MaskPostprocessing() {
	}

	void concaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
			pcl::PointCloud<pcl::PointXYZ>::Ptr &output);
	cv::Mat create_binary_segment_mask(const cv::Mat &r_mask_8UC3);
	void compute_3D_FreeSpace_PointCloud(const cv::Mat &r_mask_32F,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_points,
			pcl::PointCloud<pcl::PointXYZ> &freeSpace_3D_pointCloud);
};
#endif
