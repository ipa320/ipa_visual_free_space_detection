#ifndef UV_DISP_H
#define UV_DISP_H
#include <disp_image.cpp>

class UVDisp {
private:
	cv::Mat m_orgDisp_resultImg_8U, m_UDisp_resultImg_8U, m_VDisp_resultImg_8U;
	cv::Mat m_V_hough_1_resultImg_8U, m_V_hough_2_resultImg_8U,
			m_U_hough_1_resultImg_8U;
	cv::Mat m_U_hough_2_resultImg_8U;

	FreespaceCommon* m_ptr_common;
	LineDetection* m_ptr_lineDetection;
	Matching* m_ptr_matching;

public:
	UVDisp(FreespaceCommon* &r_ptr_common);
	~UVDisp();

	bool isGoodQuality(const cv::Mat &r_image);
	void detect(const cv::Mat &inputImg_32F, cv::Mat &disparity_mask_32F);
	void handleInvalidValues(cv::Mat &r_free_space_Mask_32F,
			const cv::Mat &r_hs_segmentation_mask_32F);

	void assignLines(DisparityImage &r_orgDispImg_32F,
			std::vector<Line> clustered_U_vec,
			std::vector<Line> clustered_V_vec);
	void findNearestLine(cv::Point &p, const std::vector<Line> &r_lines_vec,
			float m_thresh_f, int &r_bestIndex_i, float &r_minDist_f);
	void classify(DisparityImage &r_orgDispImg_32F, Line &r_groundLine,
			cv::Mat &r_freeSpace_32F);

	void show();
	void writeResults();

	const cv::Mat& getOrgDispResultImg8U() const {
		return m_orgDisp_resultImg_8U;
	}

	const cv::Mat& getUHough1ResultImg8U() const {
		return m_U_hough_1_resultImg_8U;
	}

	const cv::Mat& getUHough2ResultImg8U() const {
		return m_U_hough_2_resultImg_8U;
	}

	const cv::Mat& getUDispResultImg8U() const {
		return m_UDisp_resultImg_8U;
	}

	const cv::Mat& getVHough1ResultImg8U() const {
		return m_V_hough_1_resultImg_8U;
	}

	const cv::Mat& getVHough2ResultImg8U() const {
		return m_V_hough_2_resultImg_8U;
	}

	const cv::Mat& getVDispResultImg8U() const {
		return m_VDisp_resultImg_8U;
	}
};
#endif
