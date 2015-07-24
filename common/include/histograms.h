#ifndef HIST_H
#define HIST_H

class Hist {
private:
	cv::Mat compute(const cv::Mat &r_src_32F, const cv::Mat &r_mask_8U,
			int size[], const float* range[]);

public:
	Hist() {
	}
	~Hist() {
	}
	cv::Mat compute(const cv::Mat &r_src_32F, const cv::Mat &r_mask_8U,
			float resolution_f, float max_f);
	cv::Mat createFilterKernel();
	float calcEntropy(const cv::Mat &r_hist_32F);
	float calc_relative_Entropy(const cv::Mat &r_hist_32F, float resolution_f);
	void filter(cv::Mat &r_hist, const cv::Mat &r_kernel);
	void fill_histogram_visualization(const cv::Mat &r_hist, cv::Mat &r_dst,
			float resolution_f, float max_f, float histThresh_f);
};
#endif
