// ROS message includes
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

/* protected region user include files on begin */
#include <queue>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <common.cpp>
#include <lineDetection.cpp>
#include <sorting.cpp>
#include <matching.cpp>
#include <detect_Jimenez.cpp>
#include <detect_UVdisp.cpp>
#include <mask_postprocessing.cpp>
/* protected region user include files end */

class free_space_detection_node_config
{
public:
    bool m_debug_b;
    bool m_extractROI_b;
    bool m_printTimings_b;
    bool m_writeResults_b;
    double m_sigma_gaussian_filter_d;
    double m_minH_thresh_d;
    double m_minI_thresh_d;
    double m_felz_sigma_d;
    int m_felz_k_i;
    int m_felz_minSize_i;
    double m_segmentClassification_thresh_d;
    int m_morphSize_i;
    double m_Vdisp_thresh_d;
    double m_minValidDisp_thresh_d;
    double m_resolution_H_d;
    double m_resolution_I_d;
    double m_Udisp_thresh_d;
    double m_maxDelay_d;
    double m_alpha_d;
    double m_maxH_thresh_d;
    double m_maxI_thresh_d;
    bool m_priorityAccuracy_b;
    int m_minIntensity_i;
};

class free_space_detection_node_data
{
// autogenerated: don't touch this class
public:
    //input data
    sensor_msgs::CameraInfo in_camera_rgb_camera_info;
    sensor_msgs::Image in_camera_rgb_image_color;
    stereo_msgs::DisparityImage in_camera_depth_disparity;
    sensor_msgs::PointCloud2 in_camera_depth_points;
    sensor_msgs::CameraInfo in_camera_depth_camera_info;
    //output data
    sensor_msgs::Image out_segment_freespace_mask;
    bool out_segment_freespace_mask_active;
    sensor_msgs::Image out_colorHistogram_freespace_mask;
    bool out_colorHistogram_freespace_mask_active;
    sensor_msgs::Image out_disparity_freespace_mask;
    bool out_disparity_freespace_mask_active;
    sensor_msgs::Image out_final_freespace_mask;
    bool out_final_freespace_mask_active;
    sensor_msgs::PointCloud2 out_freeSpacePoints;
    bool out_freeSpacePoints_active;
};

class free_space_detection_node_impl
{
    /* protected region user member variables on begin */
	FreespaceCommon* m_ptr_common;
	Jimenez* m_ptr_jimenez;
	UVDisp* m_ptr_uvdisp;
	MaskPostprocessing* m_ptr_postprocessing;

	cv::Mat m_disparity_freespace_mask_32F, m_colorHist_freespace_mask_32F,
			m_segment_freespace_mask_8U, m_final_freespace_mask_32F;

	sensor_msgs::PointCloud2* m_ptr_orgPointCloud_sensorMsgs;
	pcl::PointCloud<pcl::PointXYZ>* m_ptr_orgPointCloud;
	pcl::PointCloud<pcl::PointXYZ>* m_ptr_freeSpacePoints;

	std::stringstream m_debug_info_ss;
	std::stringstream m_timing_info_ss;
	/* protected region user member variables end */

public:
    free_space_detection_node_impl() 
    {
        /* protected region user constructor on begin */
		m_ptr_freeSpacePoints = NULL;
		m_ptr_orgPointCloud_sensorMsgs = NULL;

		m_ptr_common = new FreespaceCommon();
		m_ptr_jimenez = new Jimenez(m_ptr_common);
		m_ptr_uvdisp = new UVDisp(m_ptr_common);
		m_ptr_postprocessing = new MaskPostprocessing(m_ptr_common);

		m_ptr_orgPointCloud = new pcl::PointCloud<pcl::PointXYZ>();
		/* protected region user constructor end */
    }

    void configure(free_space_detection_node_config) 
    {
        /* protected region user configure on begin */
		/* protected region user configure end */
    }

    void update(free_space_detection_node_data &data, free_space_detection_node_config config)
    {
        /* protected region user update on begin */
    	printf("update\n");
		m_debug_info_ss.clear();
		m_debug_info_ss.str("");
		m_timing_info_ss.clear();
		m_timing_info_ss.str("");

		if (m_ptr_common != NULL) {
			configure_common(config);
		}
		m_segment_freespace_mask_8U = cv::Mat::zeros(
				m_segment_freespace_mask_8U.rows,
				m_segment_freespace_mask_8U.cols,
				m_segment_freespace_mask_8U.type());
		m_colorHist_freespace_mask_32F = cv::Mat::zeros(
				m_colorHist_freespace_mask_32F.rows,
				m_colorHist_freespace_mask_32F.cols,
				m_colorHist_freespace_mask_32F.type());
		m_final_freespace_mask_32F = cv::Mat::zeros(
				m_final_freespace_mask_32F.rows,
				m_final_freespace_mask_32F.cols,
				m_final_freespace_mask_32F.type());


		m_debug_info_ss << "frame " << m_ptr_common->getFrameNr() << "\n";
		m_timing_info_ss << "frame " << m_ptr_common->getFrameNr() << ": ";

		//call main function
		bool freespace_avaiable_b = compute(data);

		if (m_ptr_common->isDebugB()) {
			printf("DEBUG INFO:\n%s", m_debug_info_ss.str().c_str());
		}

		if (freespace_avaiable_b) {
			if (m_ptr_common->isDebugB()) {
				show();
			}
			if (m_ptr_common->isPrintTimingsB()) {
				printf("%s\n", m_timing_info_ss.str().c_str());
			}
			m_ptr_common->update();

			/*****************************************
			 * fill result data
			 * ***************************************/
			data.out_disparity_freespace_mask =
					*(m_ptr_common->Mat_2_SensorMessage(
							m_disparity_freespace_mask_32F));
			data.out_colorHistogram_freespace_mask =
					*(m_ptr_common->Mat_2_SensorMessage(
							m_colorHist_freespace_mask_32F));
			data.out_segment_freespace_mask =
					*(m_ptr_common->Mat_2_SensorMessage(
							m_segment_freespace_mask_8U));
			data.out_final_freespace_mask = *(m_ptr_common->Mat_2_SensorMessage(
					m_final_freespace_mask_32F));

			if (m_ptr_freeSpacePoints != NULL) {
				pcl::toROSMsg(*m_ptr_freeSpacePoints, data.out_freeSpacePoints);
				data.out_freeSpacePoints.header =
						m_ptr_orgPointCloud_sensorMsgs->header;
			}
		}
		/* protected region user update end */
    }


    /* protected region user additional functions on begin */
	~free_space_detection_node_impl() {
		if (m_ptr_jimenez != NULL) {
			delete m_ptr_jimenez;
			m_ptr_jimenez = NULL;
		}
		if (m_ptr_uvdisp != NULL) {
			delete m_ptr_uvdisp;
			m_ptr_uvdisp = NULL;
		}
		if (m_ptr_common != NULL) {
			delete m_ptr_common;
			m_ptr_common = NULL;
		}
		if (m_ptr_postprocessing != NULL) {
			delete m_ptr_postprocessing;
			m_ptr_postprocessing = NULL;
		}
		if (m_ptr_freeSpacePoints != NULL) {
			delete m_ptr_freeSpacePoints;
			m_ptr_freeSpacePoints = NULL;
		}
		if (m_ptr_orgPointCloud != NULL) {
			delete m_ptr_orgPointCloud;
			m_ptr_orgPointCloud = NULL;
		}
		if (m_ptr_orgPointCloud_sensorMsgs == NULL) {
			delete m_ptr_orgPointCloud_sensorMsgs;
			m_ptr_orgPointCloud_sensorMsgs = NULL;
		}
	}

	void configure_common(free_space_detection_node_config config) {
		m_ptr_common->setDebugB(config.m_debug_b);
		m_ptr_common->setExtractRoiB(config.m_extractROI_b);
		m_ptr_common->setPrintTimingsB(config.m_printTimings_b);
		m_ptr_common->setWriteResultsB(config.m_writeResults_b);
		m_ptr_common->setMaxDelayD(config.m_maxDelay_d);

		m_ptr_common->setMorphSizeI(config.m_morphSize_i);

		m_ptr_common->setSigmaGaussianFilterD(config.m_sigma_gaussian_filter_d);
		m_ptr_common->setMinHThreshD(config.m_minH_thresh_d);
		m_ptr_common->setMinIThreshD(config.m_minI_thresh_d);
		m_ptr_common->setMaxHThreshD(config.m_maxH_thresh_d);
		m_ptr_common->setMaxIThreshD(config.m_maxI_thresh_d);
		m_ptr_common->setResolutionHD(config.m_resolution_H_d);
		m_ptr_common->setResolutionID(config.m_resolution_I_d);
		m_ptr_common->setAlphaD(config.m_alpha_d);

		//Parameters of graph based segmentation of Felzenszwalb Huttenlocher
		m_ptr_common->setFelzKI(config.m_felz_k_i);
		m_ptr_common->setFelzSigmaD(config.m_felz_sigma_d);
		m_ptr_common->setFelzMinSizeI(config.m_felz_minSize_i);

		//threshold for segment classification
		m_ptr_common->setSegmentClassificationThreshD(
				config.m_segmentClassification_thresh_d);
		m_ptr_common->setPriorityAccuracyB(config.m_priorityAccuracy_b);
		m_ptr_common->setMinIntensityI(config.m_minIntensity_i);

		//U V disparity
		m_ptr_common->setVdispThreshD(config.m_Vdisp_thresh_d);
		m_ptr_common->setUdispThreshD(config.m_Udisp_thresh_d);
		m_ptr_common->setMinValidDispThreshD(config.m_minValidDisp_thresh_d);
	}

	bool compute(free_space_detection_node_data &data) {
		stereo_msgs::DisparityImage dispImg = data.in_camera_depth_disparity;
		sensor_msgs::Image colorImg = data.in_camera_rgb_image_color;
		sensor_msgs::PointCloud2 pointCloud = data.in_camera_depth_points;

		bool color_avail_b = (colorImg.height != 0);
		bool disp_avail_b = (dispImg.image.height != 0);
		bool points_avail_b = !(pointCloud.data).empty();

		long unsigned int total_time = m_ptr_common->get_time_ms64();
		long unsigned int time = 0;

		cv::Mat colorImg_8U, dispImg_32F;

		//shall the graph based segmentation be computed?
		bool completeSegmentation_b = m_ptr_common->isPriorityAccuracyB();
		//test ENTROPY -> good quality?
		bool good_color_Quality_b = false;
		bool good_disp_Quality_b = false;
		//is any form of 2D free space mask available?
		bool freespace_available_b = false;

		if (!disp_avail_b) {
			printf("no disparity image available\n");
			return false;
		}

		//disp image available
		//m_debug_info_ss << "received disparity image\n";
		dispImg_32F = m_ptr_common->SensorMessage_2_Mat(dispImg.image);

		//extract disparity Region of Interest (disp ROI)
		if (m_ptr_common->isExtractRoiB()) {
			(m_ptr_common->extractROI(dispImg_32F)).copyTo(dispImg_32F);
		}
		//check quality of disparity image
		good_disp_Quality_b = m_ptr_uvdisp->isGoodQuality(dispImg_32F);

		if (!good_disp_Quality_b) {
			m_debug_info_ss
					<< "disparity image has bad quality -> no free space detection possible!\n";
			return false;
		}

		//disp has good quality
		m_debug_info_ss << "disparity image has good quality\n";

		//compute disparity mask, still some unknown pixels due to invalid disparities
		time = m_ptr_common->get_time_ms64();
		m_ptr_uvdisp->detect(dispImg_32F, m_disparity_freespace_mask_32F);
		m_timing_info_ss << "disp detect "
				<< m_ptr_common->get_time_ms64() - time << " ms, ";

		m_disparity_freespace_mask_32F.copyTo(m_final_freespace_mask_32F);
		freespace_available_b = true;

		if (color_avail_b) {
			m_debug_info_ss << "received color image\n";

			//check if time stamps similar
			ros::Time disp_time = dispImg.header.stamp;
			ros::Time color_time = colorImg.header.stamp;
			if (disp_time > color_time) {
				swap(color_time, disp_time);
			}
			bool timeDelay_ok_b = (color_time - disp_time).toNSec()
					< m_ptr_common->getMaxDelayD() * 1000000.0f;
			if (!timeDelay_ok_b) {
				m_debug_info_ss << "time stamp drifts too much!\n";
				m_ptr_common->applyMorphing(m_final_freespace_mask_32F,
						m_final_freespace_mask_32F);
			} else {
				//m_debug_info_ss << "color image has similar time stamp\n";
				colorImg_8U = m_ptr_common->SensorMessage_2_Mat(colorImg);

				//extract color Region of Interest (color ROI)
				if (m_ptr_common->isExtractRoiB()) {
					(m_ptr_common->extractROI(colorImg_8U)).copyTo(colorImg_8U);
				}
				m_ptr_jimenez->init(colorImg_8U);

				//check quality of color image
				good_color_Quality_b = m_ptr_jimenez->isGoodQuality();

				if (m_ptr_common->isDebugB()) {
					m_ptr_common->show("Color_Image", colorImg_8U);
				}

				if (!good_color_Quality_b) {
					m_debug_info_ss << "color image has bad quality\n";
					m_ptr_common->applyMorphing(m_final_freespace_mask_32F,
							m_final_freespace_mask_32F);
				} else {
					//if good color image quality, no complete segmentation and similar time stamp:
					//compute histograms based on already classified disparity values!
					if (!completeSegmentation_b) {
						m_debug_info_ss
								<< "compute histograms based on already classified pixels and handle invalid disparities with additional hue and intensity information of histograms\n";
						m_ptr_jimenez->createPixelMask(
								m_colorHist_freespace_mask_32F,
								m_disparity_freespace_mask_32F);
						m_ptr_common->applyMorphing(
								m_colorHist_freespace_mask_32F,
								m_colorHist_freespace_mask_32F);
						m_ptr_uvdisp->handleInvalidValues(
								m_final_freespace_mask_32F,
								m_colorHist_freespace_mask_32F);
					} else {
						m_debug_info_ss << "complete segmentation\n";

						time = m_ptr_common->get_time_ms64();
						m_ptr_jimenez->createSegmentMask(
								m_disparity_freespace_mask_32F,
								m_segment_freespace_mask_8U);
						m_timing_info_ss << "createSegmentMask "
								<< m_ptr_common->get_time_ms64() - time
								<< " ms, ";

						m_final_freespace_mask_32F =
								m_ptr_postprocessing->create_binary_segment_mask(
										m_segment_freespace_mask_8U);
					}
				}
			}
		}
		m_timing_info_ss << "2D-mask "
				<< m_ptr_common->get_time_ms64() - total_time << " ms, ";

		//3D Info for free space
		if (points_avail_b && freespace_available_b) {
			if (m_ptr_orgPointCloud_sensorMsgs == NULL) {
				m_ptr_orgPointCloud_sensorMsgs = new sensor_msgs::PointCloud2();
			}
			if (m_ptr_freeSpacePoints == NULL) {
				m_ptr_freeSpacePoints = new pcl::PointCloud<pcl::PointXYZ>();
			}
			*m_ptr_orgPointCloud_sensorMsgs = pointCloud;
			pcl::moveFromROSMsg(pointCloud, *m_ptr_orgPointCloud);

			m_ptr_freeSpacePoints->clear();
			pcl::PointCloud<pcl::PointXYZ>::Ptr orgPoints_shared =
					m_ptr_orgPointCloud->makeShared();

			m_ptr_postprocessing->compute_3D_FreeSpace_PointCloud(
					m_final_freespace_mask_32F, orgPoints_shared,
					*m_ptr_freeSpacePoints);

			/*
			pcl::PointCloud<pcl::PointXYZ>::Ptr freespace_points =
					m_ptr_freeSpacePoints->makeShared();
			pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull = new pcl::PointCloud<pcl::PointXYZ>();

			if(!freespace_points->empty()){
				m_ptr_postprocessing->concaveHull(freespace_points, convexHull);
			}*/
			//*m_ptr_freeSpacePoints = *convexHull;
		}
		m_timing_info_ss << "incl. 3Dpoints "
				<< m_ptr_common->get_time_ms64() - total_time << " ms\n";

		//ToDo final eval
		//writeResults();

		//update frame number only if disparity image received
		return freespace_available_b;
	}

	void show() {
		m_ptr_common->show("FreeSpace_Segments", m_segment_freespace_mask_8U);
		m_ptr_common->show("Final_FreeSpace_mask",
				m_ptr_common->convertImage_grey_Float2Uchar(
						m_final_freespace_mask_32F));
		m_ptr_common->show("Disparity_freeSpace_mask",
				m_ptr_common->convertImage_grey_Float2Uchar(
						m_disparity_freespace_mask_32F));
		m_ptr_common->show("Color_Information",
				m_ptr_common->convertImage_grey_Float2Uchar(
						m_colorHist_freespace_mask_32F));
	}
	void writeResults() {
		m_ptr_common->writeToPNG("FreeSpace_Segments",
				m_segment_freespace_mask_8U);
		m_ptr_common->writeToPNG("Final_FreeSpace_mask",
				m_ptr_common->convertImage_grey_Float2Uchar(
						m_final_freespace_mask_32F));
		m_ptr_common->writeToPNG("Disparity_freeSpace_mask",
				m_ptr_common->convertImage_grey_Float2Uchar(
						m_disparity_freespace_mask_32F));
		m_ptr_common->writeToPNG("Color_Information",
				m_ptr_common->convertImage_grey_Float2Uchar(
						m_colorHist_freespace_mask_32F));
	}
	/* protected region user additional functions end */
};
