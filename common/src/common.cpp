#ifndef FREESPACE_COMMON_CPP
#define FREESPACE_COMMON_CPP
#include <common.h>

FreespaceCommon::FreespaceCommon() :
		m_debug_b(true), m_printTimings_b(false), m_extractROI_b(true), m_writeResults_b(
				false), m_frameNr_i(0), m_orgImgWidth_i(0), m_orgImgHeight_i(0), m_roi_offsetX_i(
				0), m_roi_offsetY_i(0), m_morphSize_i(2), m_morphElement_i(
				cv::MORPH_ELLIPSE), m_morphOperation_i(cv::MORPH_CLOSE) {
	m_path_str = "/home/frm-ms/Manu/free_space_detection/results/";
}

void FreespaceCommon::configure(int width_i, int height_i) {
	m_orgImgWidth_i = width_i;
	m_orgImgHeight_i = height_i;

	if (m_extractROI_b) {
		m_roi_offsetX_i = 0;
		m_roi_offsetY_i = (int) height_i / 2.0f;

		m_roiImgWidth_i = width_i;
		m_roiImgHeight_i = m_orgImgHeight_i - m_roi_offsetY_i;
	}
}

void FreespaceCommon::update() {
	++m_frameNr_i;
}

sensor_msgs::Image::Ptr FreespaceCommon::Mat_2_SensorMessage(
		const cv::Mat &r_mat) {
	cv_bridge::CvImage out_msg;
	//ToDo -> add header with timestamp and frameNumber
	//out_msg.header   = in_msg->header;
	std::string enc;
	switch (r_mat.type()) {
	case CV_8UC1:
		enc = "8UC1";
		break;
	case CV_8UC3:
		enc = "8UC3";
		break;
	case CV_32FC1:
		enc = "32FC1";
		break;
	case CV_32FC3:
		enc = "32FC3";
		break;
	case CV_64FC1:
		enc = "64FC1";
		break;
	case CV_64FC3:
		enc = "64FC3";
		break;
	}
	out_msg.encoding = enc;
	out_msg.image = r_mat;
	return out_msg.toImageMsg();
}

cv::Mat FreespaceCommon::SensorMessage_2_Mat(
		const sensor_msgs::Image &r_sensorImage) {
	cv::Mat cvMat;
	try {
		cvMat =
				cv_bridge::toCvCopy(r_sensorImage, r_sensorImage.encoding)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("failed to get image: %s", e.what());
	}
	return cvMat;
}
cv::Mat FreespaceCommon::SensorMessage_2_Mat(
		const sensor_msgs::Image::Ptr &r_sensorImage) {
	return SensorMessage_2_Mat(*r_sensorImage);
}

std::string FreespaceCommon::frame_number_to_string(int frame_number_i) {
	std::stringstream result;
	int total_digits = 5;

	int sample = frame_number_i;
	int leadingZeros = total_digits-1;

	while (sample > 9) {
		sample /= 10;
		--leadingZeros;
	}

	for (int i = 0; i < leadingZeros; ++i) {
		result << "0";
	}
	result << frame_number_i;
	return result.str();
}

/**
 * @function get_time_ms64
 * gets unix time stamp in ms
 *
 * Return:
 * @return unix time stamp in ms
 */
long unsigned int FreespaceCommon::get_time_ms64() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long unsigned int ret = tv.tv_usec;
	/* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
	ret /= 1000;
	/* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
	ret += (tv.tv_sec * 1000);
	return ret;
}

void FreespaceCommon::applyMorphing(const cv::Mat &r_inputImg_32F,
		cv::Mat &r_outputImg_32F) {
	cv::Mat element = getStructuringElement(m_morphElement_i,
			cv::Size(2 * m_morphSize_i + 1, 2 * m_morphSize_i + 1),
			cv::Point(m_morphSize_i, m_morphSize_i));
	cv::morphologyEx(r_inputImg_32F, r_outputImg_32F, m_morphOperation_i,
			element);
}

std::string FreespaceCommon::createFileName(const std::string &name,
		std::string extension) {
	std::stringstream fileName;
	fileName << m_path_str << frame_number_to_string(m_frameNr_i) << "_"
			<< name << extension;
	//printf("fileName -> %s\n", fileName.str().c_str());
	return fileName.str();
}

cv::Mat FreespaceCommon::mergeChannels2Image(std::vector<cv::Mat> channels) {
	cv::Mat result = cv::Mat(channels[0].rows, channels[0].cols, CV_8UC3);
	cv::Mat in[] = { channels[0], channels[1], channels[2] };
	int from_to[] = { 0, 0, 1, 1, 2, 2 };
	mixChannels(in, 3, &result, 1, from_to, 3);
	return result;
}

void FreespaceCommon::show(const std::string &r_name_str, const cv::Mat &img) {
	if (img.cols != 0 && img.rows != 0) {
		cv::imshow(r_name_str, img);
		if (cv::waitKey(10) >= 0)
			cv::destroyWindow(r_name_str);
	}
}
void FreespaceCommon::show(const std::string &r_name_str, const IplImage *dst) {
	if (dst->width != 0 && dst->height != 0) {
		show(r_name_str, cv::Mat(dst));
	}
}
void FreespaceCommon::writeToPNG(const std::string &r_name_str,
		const cv::Mat &r_img) {
	if (r_img.cols != 0 && r_img.rows != 0) {
		cv::imwrite(createFileName(r_name_str, ".png"), r_img);
	}
}
void FreespaceCommon::writeToPPM(const std::string &r_name_str,
		image<rgb>* &r_ptr_img_rgb) {
	if (r_ptr_img_rgb->width() != 0 && r_ptr_img_rgb->height() != 0) {
		savePPM(r_ptr_img_rgb, createFileName(r_name_str, ".ppm").c_str());
	}
}
void FreespaceCommon::writeToPNG(const std::string &r_name_str,
		const IplImage *img) {
	writeToPNG(r_name_str, cv::Mat(img));
}

void FreespaceCommon::writeToPCD(const std::string &r_name_str,
		const pcl::PointCloud<pcl::PointXYZ> & r_cloud) {
	pcl::io::savePCDFileASCII(createFileName(r_name_str, ".pcd"), r_cloud);
}

void FreespaceCommon::writeToPCD(const std::string &r_name_str,
		sensor_msgs::PointCloud2 & r_cloud) {
	pcl::PointCloud < pcl::PointXYZ > points;
	pcl::moveFromROSMsg(r_cloud, points);

	writeToPCD(r_name_str, points);

	cv::FileStorage file(createFileName(r_name_str, "_header.xml"),
			cv::FileStorage::WRITE);

	file << "FrameId" << r_cloud.header.frame_id;
	file << "Seq" << (int) r_cloud.header.seq;
	file << "Stamp_Sec" << (double) r_cloud.header.stamp.sec;
	file << "Stamp_NanoSec" << (double) r_cloud.header.stamp.nsec;

	file.release();
}

void FreespaceCommon::writeToXML(const std::string &r_name_str,
		const cv::Mat & r_image) {
	cv::FileStorage file(createFileName(r_name_str, ".xml"),
			cv::FileStorage::WRITE);
	file << "Image" << r_image;
	file.release();
}

cv::Mat FreespaceCommon::convertImage_bgr_Uchar2Float(
		const cv::Mat &r_BGRimg_8U) {
	cv::Mat result = cv::Mat(r_BGRimg_8U.rows, r_BGRimg_8U.cols, CV_32FC3);
	r_BGRimg_8U.convertTo(result, CV_32FC3, 1.0 / 255.0);
	return result;
}
cv::Mat FreespaceCommon::convertImage_bgr_Float2Uchar(
		const cv::Mat &r_BGRimg_32F) {
	cv::Mat result = cv::Mat(r_BGRimg_32F.rows, r_BGRimg_32F.cols, CV_8UC3);
	r_BGRimg_32F.convertTo(result, CV_8UC3, 1.0 * 255.0);
	return result;
}
cv::Mat FreespaceCommon::convertImage_grey_Uchar2Float(
		const cv::Mat &r_img_8U) {
	cv::Mat result = cv::Mat(r_img_8U.rows, r_img_8U.cols, CV_32FC1);
	r_img_8U.convertTo(result, CV_32FC1, 1.0 / 255.0);
	return result;
}
cv::Mat FreespaceCommon::convertImage_grey_Float2Uchar(
		const cv::Mat &r_img_32F) {
	cv::Mat result = cv::Mat(r_img_32F.rows, r_img_32F.cols, CV_8UC1);
	r_img_32F.convertTo(result, CV_8UC1, 1.0 * 255.0);
	return result;
}
cv::Mat FreespaceCommon::convert_hueImage_Float2Uchar(
		const cv::Mat &r_HUEimg_32F) {
	cv::Mat result = cv::Mat(r_HUEimg_32F.rows, r_HUEimg_32F.cols, CV_8UC1);
	r_HUEimg_32F.convertTo(result, CV_8UC1, 1.0f / 2.0f);
	return result;
}

cv::Mat FreespaceCommon::luv32F_2_luv8U(const cv::Mat &r_LUVimg_32F) {
	//split channels
	std::vector<cv::Mat> channels_vec(3);
	cv::split(r_LUVimg_32F, channels_vec);
	//convert each channel
	channels_vec[0].convertTo(channels_vec[0], CV_8UC1, 1.0 * 255.0f / 100.0);
	channels_vec[1].convertTo(channels_vec[1], CV_8UC1, 255.0 / 354.0,
			(134.0) * 255.0 / 354.0);
	channels_vec[2].convertTo(channels_vec[2], CV_8UC1, 255.0 / 256.0,
			(140.0) * 255.0 / 256.0);
	//merge to single image again
	return mergeChannels2Image(channels_vec);
}
#endif
