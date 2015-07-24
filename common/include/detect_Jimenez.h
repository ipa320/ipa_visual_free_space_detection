#ifndef JIMENEZ_H
#define JIMENEZ_H

#include <histograms.cpp>
#include <felzenszwalb/segment-image.h>

using namespace cv;
using namespace std;

class Jimenez {
private:
	FreespaceCommon* m_ptr_common;
	Hist* m_ptr_hist;

	//fix parameter
	static constexpr float m_maxH_f = 360.0f;
	static constexpr float m_maxI_f = 255.0f;

	int m_imgWidth_i;
	int m_imgHeight_i;

	cv::Mat* m_ptr_inputImg_32F;
	cv::Mat* m_ptr_hueImg_32F;
	cv::Mat* m_ptr_intensityImg_32F;
	cv::Mat* m_ptr_luvImg_32F;

	//Timings
	long unsigned int m_computeHistograms_time, m_createPixelMask_time;

	//called once per frame
	cv::Mat createTrapez(const cv::Mat &r_img_32F);
	cv::Mat createReferenceArea(const cv::Mat &r_img_32F);

	void createIntensityImage(const cv::Mat & r_inputImg,
			cv::Mat & r_outputImg);
	void createHueImage(const cv::Mat & r_inputImg, cv::Mat & r_outputImg);
	void createLUVImage(const cv::Mat & r_inputImg, cv::Mat & r_outputImg);

	void classifyPixel(const cv::Mat &r_hueImg_32F,
			const cv::Mat &r_intensityImg_32F, const cv::Mat &r_hist_H_mat,
			const cv::Mat &r_hist_I_mat, cv::Mat &r_mask_32F, float thresh_H_f,
			float thresh_I_f);
public:
	Jimenez(FreespaceCommon* &c);
	~Jimenez();
	void init(const cv::Mat &r_inputImg_8U);

	bool isGoodQuality();
	void createPixelMask(cv::Mat &r_UVmask_32F,
			cv::Mat r_freeSpace_pixel_mask_32F);
	void createSegmentMask(cv::Mat &r_pixel_mask_32F,
			cv::Mat &r_segment_mask_8U);

	void show();
	void writeResults();
	void writeTimings();
};
#endif
