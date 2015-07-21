#include <histograms.h>

#ifndef HIST_CPP
#define HIST_CPP

//	void calcHist(	const Mat* images, int nimages, const int* channels, InputArray mask, OutputArray hist,
//					int dims, const int* histSize, const float** ranges,
//					bool uniform=true, bool accumulate=false )
//	ranges 		- Array of the dims arrays of the histogram bin boundaries in each dimension
cv::Mat Hist::compute(const cv::Mat &r_src_32F, const cv::Mat &r_mask_8U,
		int nr_bins[], const float* range[]) {
	int nimages_i = 1;
	int dim_i = 1;
	const int channel_i = 0;
	bool uniform_b = true;
	cv::Mat hist;

	cv::calcHist(&r_src_32F, nimages_i, &channel_i, r_mask_8U, hist, dim_i,
			nr_bins, range, uniform_b, false);

	//normalize to 0...1
	cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	return hist;
}
cv::Mat Hist::compute(const cv::Mat &r_src_32F, const cv::Mat &r_mask_8U,
		float resolution_f, float max_f) {
	int nr_bins[] = { (int) MIN(max_f / (resolution_f * 100.0f), max_f) };

	const float range[] = { 0, max_f };
	const float* ranges[] = { range };
	return compute(r_src_32F, r_mask_8U, nr_bins, ranges);
}
//creates float 1/3 Box filter
cv::Mat Hist::createFilterKernel() {
	cv::Mat result = cv::Mat::zeros(3, 1, CV_32FC1);
	for (int i = 0; i < 3; ++i) {
		result.at<float>(i) = 1.0f / 3.0f;
	}
	return result;
}
void Hist::filter(cv::Mat &r_hist, const cv::Mat &r_kernel) {
	cv::filter2D(r_hist, r_hist, -1, r_kernel);
}
float Hist::calcEntropy(const cv::Mat &r_hist_32F) {
	//entropy H(X) = - sum_1_to_N ( p_i*log(p_i) )
	//p_i = number of samples with same value  / total number of samples
	//total number of samples
	int number_samples = (int) (cv::sum(r_hist_32F)[0]);
	//printf("total number of samples = %i\n", histogram_length);

	float entropy_f = 0;
	for (int i = 0; i < r_hist_32F.cols; ++i) {
		float p_f = r_hist_32F.at<float>(i) / (float) number_samples;//Anzahl von wert / gesamtzahl samples
		if (p_f != 0) {
			entropy_f += (p_f * log2(p_f));
		}
	}
	return -entropy_f;
}

//calculate relative entropy (divided by max entropy)
float Hist::calc_relative_Entropy(const cv::Mat &r_hist_32F,
		float resolution_f) {
	int number_samples = (int) (cv::sum(r_hist_32F)[0]);
	cv::Mat tmpH = cv::Mat::ones(1, number_samples, CV_32FC1);
	float e_max = calcEntropy(tmpH);
	return calcEntropy(r_hist_32F) / e_max;
}

//compute histogram visualization images
void Hist::fill_histogram_visualization(const cv::Mat &r_hist_32F,
		cv::Mat &r_dst, float resolution_f, float max_f, float histThresh_f) {
	r_dst = cv::Mat::zeros(r_dst.rows, r_dst.cols, CV_8UC1);
	int nr_bins[] = { (int) MIN(max_f / (resolution_f * 100.0f), max_f) };
	int hist_height = r_dst.rows;
	int hist_width = r_dst.cols;
	int bin_w = cvRound((double) hist_width / nr_bins[0]);

	cv::Mat tmpHist_32F;
	cv::normalize(r_hist_32F, tmpHist_32F, 0, hist_height, cv::NORM_MINMAX, -1,
			cv::Mat());
	for (int i = 1; i < nr_bins[0]; ++i) {
		cv::line(r_dst,
				cv::Point(bin_w * (i - 1),
						hist_height - cvRound(tmpHist_32F.at<float>(i - 1))),
				cv::Point(bin_w * (i),
						hist_height - cvRound(tmpHist_32F.at<float>(i))),
				cv::Scalar(255, 0, 0), 2, 8, 0);
	}
	std::stringstream textBufferThresh;
	textBufferThresh << "Th " << histThresh_f;

	//scale values
	histThresh_f *= hist_height;

	//histogram threshold
	cv::line(r_dst, cv::Point(0, hist_height - cvRound(histThresh_f)),
			cv::Point(hist_width - 1, hist_height - cvRound(histThresh_f)),
			cv::Scalar(255, 0, 0), 2, 8, 0);
	cv::putText(r_dst, textBufferThresh.str().c_str(),
			cv::Point(20, hist_height - cvRound(histThresh_f) - 5), 1, 1.0f,
			cv::Scalar(255, 0, 0), 1, 5, false);
}

#endif
