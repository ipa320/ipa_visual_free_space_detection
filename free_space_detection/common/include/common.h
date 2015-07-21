#ifndef FREESPACE_COMMON_H
#define FREESPACE_COMMON_H

#include <felzenszwalb/segment-image.h>
#include <felzenszwalb/image.h>
#include <felzenszwalb/pnmfile.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>

class common_config {
public:
	bool m_debug_b;
	bool m_timings_b;
	bool m_writeResults_b;
	bool m_extractROI_b;
	double m_maxDelay_d;
	int m_morphSize_i;

	common_config() {
		common_config(false, false, false, true, 450.0, 4);
	}
	common_config(bool debug_b, bool timings_b, bool writeResults_b,
			bool extractROI_b, double maxDelay_d, int morphSize_i) :
			m_debug_b(debug_b), m_timings_b(timings_b), m_writeResults_b(
					writeResults_b), m_extractROI_b(extractROI_b), m_maxDelay_d(
					maxDelay_d), m_morphSize_i(morphSize_i) {
	}
};

class FreespaceCommon {
private:
	int m_frameNr_i;

	std::string m_path_str;

	bool m_debug_b;
	bool m_printTimings_b;
	bool m_writeResults_b;
	bool m_extractROI_b;

	double m_sigma_gaussian_filter_d;
	double m_minH_thresh_d;
	double m_minI_thresh_d;
	double m_maxH_thresh_d;
	double m_maxI_thresh_d;
	double m_resolution_H_d;
	double m_resolution_I_d;

	double m_felz_sigma_d;
	int m_felz_k_i;
	int m_felz_minSize_i;
	double m_segmentClassification_thresh_d;
	double m_alpha_d;

	int m_morphSize_i;
	int m_morphElement_i;
	int m_morphOperation_i;

	double m_Vdisp_thresh_d;
	double m_Udisp_thresh_d;
	double m_minValidDisp_thresh_d;

	double m_maxDelay_d;
	bool m_priorityAccuracy_b;
	int m_minIntensity_i;

	//computed
	int m_orgImgWidth_i;
	int m_orgImgHeight_i;
	int m_roi_offsetX_i;
	int m_roi_offsetY_i;
	int m_roiImgWidth_i;
	int m_roiImgHeight_i;

public:
	FreespaceCommon();
	~FreespaceCommon() {
	}

	void configure(int width_i, int height_i);
	void update();

	cv::Mat extractROI(cv::Mat &r_img) {
		cv::Mat tmp;
		r_img(cv::Rect(0, r_img.rows / 2.0f, r_img.cols, r_img.rows / 2.0f)).copyTo(
				tmp);
		return tmp;
	}

	sensor_msgs::Image::Ptr Mat_2_SensorMessage(const cv::Mat &img_32F);
	cv::Mat SensorMessage_2_Mat(const sensor_msgs::Image &img_32F);
	cv::Mat SensorMessage_2_Mat(const sensor_msgs::Image::Ptr &r_sensorImage);

	void applyMorphing(const cv::Mat &r_inputImg_32F, cv::Mat &r_outputImg_32F);

	std::string frame_number_to_string(int frame_number_i);

	std::string createFileName(const std::string &name, std::string extension);
	cv::Mat mergeChannels2Image(std::vector<cv::Mat> channels);
	long unsigned int get_time_ms64();
	float getDistance(cv::Point2f p1, cv::Point2f p2);

	void show(const std::string &r_name_str, const cv::Mat &r_img);
	void show(const std::string &r_name_str, const IplImage *dst);
	void writeToPNG(const std::string &r_name_str, const cv::Mat &r_img);
	void writeToPPM(const std::string &r_name_str, image<rgb>* &r_ptr_img_rgb);
	void writeToPNG(const std::string &r_name_str, const IplImage *img);
	void writeToPCD(const std::string &r_name_str,
			const pcl::PointCloud<pcl::PointXYZ> & r_cloud);
	void writeToXML(const std::string &r_name_str,
			const cv::Mat & r_disp_32FC1);

	void readPCDFile(const std::string &r_name_str,
			sensor_msgs::PointCloud2 & r_cloud);
	void writeToPCD(const std::string &r_name_str,
			sensor_msgs::PointCloud2 & r_cloud);

	void readPCDFile(const std::string &r_name_str,
			pcl::PointCloud<pcl::PointXYZ> & r_cloud);
	void readPNGFile(const std::string &r_name_str, cv::Mat &r_img);
	void readXMLFile(const std::string &r_name_str, cv::Mat &r_disp_32FC1);

	cv::Mat convertImage_bgr_Uchar2Float(const cv::Mat &r_BGRimg_8U);
	cv::Mat convertImage_bgr_Float2Uchar(const cv::Mat &r_BGRimg_32F);
	cv::Mat convertImage_grey_Uchar2Float(const cv::Mat &r_img_8U);
	cv::Mat convertImage_grey_Float2Uchar(const cv::Mat &r_img_32F);
	cv::Mat convert_hueImage_Float2Uchar(const cv::Mat &r_HUEimg_32F);
	cv::Mat bgr32F_2_bgr8U(const cv::Mat &r_BGRimg_32F);
	cv::Mat luv32F_2_luv8U(const cv::Mat &r_LUVimg_32F);

	//Getter and Setter

	double getSegmentClassificationThreshD() const {
		return m_segmentClassification_thresh_d;
	}

	void setSegmentClassificationThreshD(double algo1FelzThreshD) {
		m_segmentClassification_thresh_d = algo1FelzThreshD;
	}

	double getAlphaD() const {
		return m_alpha_d;
	}

	void setAlphaD(double alphaD) {
		m_alpha_d = alphaD;
	}

	bool isDebugB() const {
		return m_debug_b;
	}

	void setDebugB(bool debugB) {
		m_debug_b = debugB;
	}

	bool isExtractRoiB() const {
		return m_extractROI_b;
	}

	void setExtractRoiB(bool extractRoiB) {
		m_extractROI_b = extractRoiB;
	}

	int getFelzKI() const {
		return m_felz_k_i;
	}

	void setFelzKI(int felzKI) {
		m_felz_k_i = felzKI;
	}

	int getFelzMinSizeI() const {
		return m_felz_minSize_i;
	}

	void setFelzMinSizeI(int felzMinSizeI) {
		m_felz_minSize_i = felzMinSizeI;
	}

	double getFelzSigmaD() const {
		return m_felz_sigma_d;
	}

	void setFelzSigmaD(double felzSigmaD) {
		m_felz_sigma_d = felzSigmaD;
	}

	int getFrameNr() const {
		return m_frameNr_i;
	}

	double getMaxDelayD() const {
		return m_maxDelay_d;
	}

	void setMaxDelayD(double maxDelayD) {
		m_maxDelay_d = maxDelayD;
	}

	double getMaxHThreshD() const {
		return m_maxH_thresh_d;
	}

	void setMaxHThreshD(double maxHThreshD) {
		m_maxH_thresh_d = maxHThreshD;
	}

	double getMaxIThreshD() const {
		return m_maxI_thresh_d;
	}

	void setMaxIThreshD(double maxIThreshD) {
		m_maxI_thresh_d = maxIThreshD;
	}

	double getMinHThreshD() const {
		return m_minH_thresh_d;
	}

	void setMinHThreshD(double minHThreshD) {
		m_minH_thresh_d = minHThreshD;
	}

	double getMinIThreshD() const {
		return m_minI_thresh_d;
	}

	void setMinIThreshD(double minIThreshD) {
		m_minI_thresh_d = minIThreshD;
	}

	double getMinValidDispThreshD() const {
		return m_minValidDisp_thresh_d;
	}

	void setMinValidDispThreshD(double minValidDispThreshD) {
		m_minValidDisp_thresh_d = minValidDispThreshD;
	}

	int getMorphElementI() const {
		return m_morphElement_i;
	}

	void setMorphElementI(int morphElementI) {
		m_morphElement_i = morphElementI;
	}

	int getMorphOperationI() const {
		return m_morphOperation_i;
	}

	void setMorphOperationI(int morphOperationI) {
		m_morphOperation_i = morphOperationI;
	}

	int getMorphSizeI() const {
		return m_morphSize_i;
	}

	void setMorphSizeI(int morphSizeI) {
		m_morphSize_i = morphSizeI;
	}

	int getOrgImgHeightI() const {
		return m_orgImgHeight_i;
	}

	void setOrgImgHeightI(int orgImgHeightI) {
		m_orgImgHeight_i = orgImgHeightI;
	}

	int getOrgImgWidthI() const {
		return m_orgImgWidth_i;
	}

	void setOrgImgWidthI(int orgImgWidthI) {
		m_orgImgWidth_i = orgImgWidthI;
	}

	const std::string& getPathStr() const {
		return m_path_str;
	}

	void setPathStr(const std::string& pathStr) {
		m_path_str = pathStr;
	}

	bool isPrintTimingsB() const {
		return m_printTimings_b;
	}

	void setPrintTimingsB(bool printTimingsB) {
		m_printTimings_b = printTimingsB;
	}

	bool isPriorityAccuracyB() const {
		return m_priorityAccuracy_b;
	}

	void setPriorityAccuracyB(bool priorityAccuracyB) {
		m_priorityAccuracy_b = priorityAccuracyB;
	}

	double getResolutionHD() const {
		return m_resolution_H_d;
	}

	void setResolutionHD(double resolutionHD) {
		m_resolution_H_d = resolutionHD;
	}

	double getResolutionID() const {
		return m_resolution_I_d;
	}

	void setResolutionID(double resolutionID) {
		m_resolution_I_d = resolutionID;
	}

	int getRoiOffsetXI() const {
		return m_roi_offsetX_i;
	}

	void setRoiOffsetXI(int roiOffsetXI) {
		m_roi_offsetX_i = roiOffsetXI;
	}

	int getRoiOffsetYI() const {
		return m_roi_offsetY_i;
	}

	void setRoiOffsetYI(int roiOffsetYI) {
		m_roi_offsetY_i = roiOffsetYI;
	}

	int getRoiImgHeightI() const {
		return m_roiImgHeight_i;
	}

	void setRoiImgHeightI(int roiImgHeightI) {
		m_roiImgHeight_i = roiImgHeightI;
	}

	int getRoiImgWidthI() const {
		return m_roiImgWidth_i;
	}

	void setRoiImgWidthI(int roiImgWidthI) {
		m_roiImgWidth_i = roiImgWidthI;
	}

	double getSigmaGaussianFilterD() const {
		return m_sigma_gaussian_filter_d;
	}

	void setSigmaGaussianFilterD(double sigmaGaussianFilterD) {
		m_sigma_gaussian_filter_d = sigmaGaussianFilterD;
	}

	double getUdispThreshD() const {
		return m_Udisp_thresh_d;
	}

	void setUdispThreshD(double udispThreshD) {
		m_Udisp_thresh_d = udispThreshD;
	}

	double getVdispThreshD() const {
		return m_Vdisp_thresh_d;
	}

	void setVdispThreshD(double vdispThreshD) {
		m_Vdisp_thresh_d = vdispThreshD;
	}

	bool isWriteResultsB() const {
		return m_writeResults_b;
	}

	void setWriteResultsB(bool writeResultsB) {
		m_writeResults_b = writeResultsB;
	}

	int getMinIntensityI() const {
		return m_minIntensity_i;
	}

	void setMinIntensityI(int minIntensityI) {
		m_minIntensity_i = minIntensityI;
	}
};

#endif
