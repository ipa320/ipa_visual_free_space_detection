#ifndef JIMENEZ_CPP
#define JIMENEZ_CPP
#include <detect_Jimenez.h>

Jimenez::Jimenez(FreespaceCommon* &r_ptr_common) :
		m_ptr_hist(NULL), m_ptr_inputImg_32F(NULL), m_ptr_hueImg_32F(NULL), m_ptr_intensityImg_32F(
				NULL), m_ptr_luvImg_32F(NULL), m_ptr_common(r_ptr_common) {
}
Jimenez::~Jimenez() {
	if (m_ptr_hist != NULL) {
		delete m_ptr_hist;
		m_ptr_hist = NULL;
	}
	if (m_ptr_inputImg_32F != NULL) {
		delete m_ptr_inputImg_32F;
		m_ptr_inputImg_32F = NULL;
	}
	if (m_ptr_hueImg_32F != NULL) {
		delete m_ptr_hueImg_32F;
		m_ptr_hueImg_32F = NULL;
	}
	if (m_ptr_intensityImg_32F != NULL) {
		delete m_ptr_intensityImg_32F;
		m_ptr_intensityImg_32F = NULL;

	}
	if (m_ptr_luvImg_32F != NULL) {
		delete m_ptr_luvImg_32F;
		m_ptr_luvImg_32F = NULL;
	}
}

//--> values in range 0 ... 255
void Jimenez::createIntensityImage(const cv::Mat & r_inputImg,
		cv::Mat & r_outputImg) {
	for (int row = 0; row < r_inputImg.rows; ++row) {
		for (int col = 0; col < r_inputImg.cols; ++col) {
			r_outputImg.at<float>(row, col) = (r_inputImg.at<cv::Vec3f>(row,
					col)[0] + r_inputImg.at<cv::Vec3f>(row, col)[1]
					+ r_inputImg.at<cv::Vec3f>(row, col)[2]) / 3.0f * 255.0f;
		}
	}
}

//2.) 	transform input image to HSV color space and get Hue channel
//		convert to 8-bit images:	L <- L*255, S <- S*255, H <- H/2
//range -> 0 ... 360
void Jimenez::createHueImage(const cv::Mat & r_inputImg,
		cv::Mat & r_outputImg) {
	cv::Mat hsv_32FC3;
	cvtColor(r_inputImg, hsv_32FC3, CV_BGR2HSV);
	std::vector<cv::Mat> hsv_channels_vec(3);
	cv::split(hsv_32FC3, hsv_channels_vec);
	hsv_channels_vec[0].copyTo(r_outputImg);
}

// 6.) 	transform input image to L*u*v color space
// 		0 <= L <= 100	-134 <= u <= 220	-140 <= v <= 122
//		convert to 8-bit images:	L <- L*255/100, u <- (u + 134)*255/354, v <- (v + 140)*255/256
void Jimenez::createLUVImage(const cv::Mat & r_inputImg,
		cv::Mat & r_outputImg) {
	cvtColor(r_inputImg, r_outputImg, CV_BGR2Luv);
}

void Jimenez::init(const cv::Mat &r_inputImg_8U) {
	m_imgWidth_i = r_inputImg_8U.cols;
	m_imgHeight_i = r_inputImg_8U.rows;

	//prepare
	if (m_ptr_inputImg_32F == NULL) {
		m_ptr_inputImg_32F = new cv::Mat(m_imgHeight_i, m_imgWidth_i, CV_32FC3);
	}
	if (m_ptr_hueImg_32F == NULL) {
		m_ptr_hueImg_32F = new cv::Mat(m_imgHeight_i, m_imgWidth_i, CV_32FC1);
	}
	if (m_ptr_intensityImg_32F == NULL) {
		m_ptr_intensityImg_32F = new cv::Mat(m_imgHeight_i, m_imgWidth_i,
				CV_32FC1);
	}
	if (m_ptr_luvImg_32F == NULL) {
		m_ptr_luvImg_32F = new cv::Mat(m_imgHeight_i, m_imgWidth_i, CV_32FC3);
	}
	if (m_ptr_hist == NULL) {
		m_ptr_hist = new Hist();
	}

	//convert 8 bit input image to 32 float image
	*m_ptr_inputImg_32F = m_ptr_common->convertImage_bgr_Uchar2Float(
			r_inputImg_8U);

	//apply gaussian filter
	cv::GaussianBlur(*m_ptr_inputImg_32F, *m_ptr_inputImg_32F, cv::Size(0, 0),
			m_ptr_common->getSigmaGaussianFilterD());

	//create luv, hue and intensity image
	createHueImage(*m_ptr_inputImg_32F, *m_ptr_hueImg_32F);
	createIntensityImage(*m_ptr_inputImg_32F, *m_ptr_intensityImg_32F);
	createLUVImage(*m_ptr_inputImg_32F, *m_ptr_luvImg_32F);

	if (m_ptr_common->isDebugB()) {
		show();
	}
	if (m_ptr_common->isWriteResultsB()) {
		writeResults();
	}

}

//if UV disp free space mask is empty --> ALGO 1
//if ALGO 1 && first frame -> create reference area ELSE use existing mask
//else ALGO 4: compute histograms using UV disparity mask
//m_ptr_hueImg_32F already 0 ... 360, m_ptr_intensityImg_32F has range 0 ... 255
//compute histograms of channels H and I of HSI color space for reference area / UV disp free space mask
void Jimenez::createPixelMask(cv::Mat &r_freeSpace_pixel_mask_32F,
		cv::Mat r_input_UVdisp_free_space_mask_32F = cv::Mat()) {
	m_createPixelMask_time = m_ptr_common->get_time_ms64();
	//create mask of reference pixel
	cv::Mat reference_mask_8U = m_ptr_common->convertImage_grey_Float2Uchar(
			r_input_UVdisp_free_space_mask_32F);

	//compute H and I histograms
	cv::Mat hist_H_32F, hist_I_32F;
	hist_H_32F = m_ptr_hist->compute(*m_ptr_hueImg_32F, reference_mask_8U,
			m_ptr_common->getResolutionHD(), m_maxH_f);
	hist_I_32F = m_ptr_hist->compute(*m_ptr_intensityImg_32F, reference_mask_8U,
			m_ptr_common->getResolutionID(), m_maxI_f);

	//apply low pass filter to histograms
	cv::Mat kernel = m_ptr_hist->createFilterKernel();
	m_ptr_hist->filter(hist_H_32F, kernel);
	m_ptr_hist->filter(hist_I_32F, kernel);

	//compute entropy
	float entropy_H = m_ptr_hist->calc_relative_Entropy(hist_H_32F,
			(float) m_ptr_common->getResolutionHD());
	float entropy_I = m_ptr_hist->calc_relative_Entropy(hist_I_32F,
			(float) m_ptr_common->getResolutionID());

	double alpha_d = m_ptr_common->getAlphaD();
	double minH_d = m_ptr_common->getMinHThreshD();
	double minI_d = m_ptr_common->getMinIThreshD();
	double maxH_d = m_ptr_common->getMaxHThreshD();
	double maxI_d = m_ptr_common->getMaxIThreshD();

	//compute entropy based threshold
	float thresh_entr_H_f = minH_d
			+ alpha_d * (maxI_d - minI_d) * (1.0f - entropy_H);
	float thresh_entr_I_f = minI_d
			+ alpha_d * (maxI_d - minI_d) * (1.0f - entropy_I);

	//HI segmentation (pixel classification)
	classifyPixel(*m_ptr_hueImg_32F, *m_ptr_intensityImg_32F, hist_H_32F,
			hist_I_32F, r_freeSpace_pixel_mask_32F, thresh_entr_H_f,
			thresh_entr_I_f);

	m_createPixelMask_time = m_ptr_common->get_time_ms64()
			- m_createPixelMask_time;
}

//check image quality: if "dark" frame (mean > minIntensity) and low entropy --> reject dark frames as no good quality
bool Jimenez::isGoodQuality() {
	//compute intensity histogram over complete input image and apply low pass filter
	cv::Mat hist_I_32F = m_ptr_hist->compute(*m_ptr_intensityImg_32F,
			hist_I_32F, m_ptr_common->getResolutionID(), m_maxI_f);
	cv::Mat kernel = m_ptr_hist->createFilterKernel();
	m_ptr_hist->filter(hist_I_32F, kernel);

	//compute entropy and mean intensity value of input image
	float entropy_f = m_ptr_hist->calc_relative_Entropy(hist_I_32F,
			(float) m_ptr_common->getResolutionID());
	float mean_greyValue_f = mean(*m_ptr_intensityImg_32F)[0];

	if (m_ptr_common->isDebugB()) {
		printf(
				"computeImageQuality -> total image entropy = %f, mean grey = %f\n",
				entropy_f, mean_greyValue_f);
	}
	return entropy_f < 0.1f
			&& mean_greyValue_f > m_ptr_common->getMinIntensityI();
}

//range 0 ... 255 , in intensityImg_32F ---*255---> between 0 ... 255
//get hue and intensity value of current pixel, between 0 ... 360
//white == free, black == obstacle
void Jimenez::classifyPixel(const cv::Mat &r_hueImg_32F,
		const cv::Mat &r_intensityImg_32F, const cv::Mat &r_hist_H_mat,
		const cv::Mat &r_hist_I_mat, cv::Mat &r_mask_32F, float thresh_H_f,
		float thresh_I_f) {
	//clear mask
	r_mask_32F = cv::Mat::zeros(m_imgHeight_i, m_imgWidth_i, CV_32FC1);

	for (int row = 0; row < m_imgHeight_i; ++row) {
		for (int col = 0; col < m_imgWidth_i; ++col) {
			int h_idx_i = r_hueImg_32F.at<float>(row, col)
					/ (m_maxH_f / r_hist_H_mat.rows);
			int i_idx_i = r_intensityImg_32F.at<float>(row, col)
					/ (m_maxI_f / r_hist_I_mat.rows);

			//check range
			if (h_idx_i < 0) {
				h_idx_i = 0;
			}
			if (h_idx_i >= r_hist_H_mat.rows) {
				h_idx_i = r_hist_H_mat.rows - 1;
			}
			if (i_idx_i < 0) {
				i_idx_i = 0;
			}
			if (i_idx_i >= r_hist_I_mat.rows) {
				i_idx_i = r_hist_I_mat.rows - 1;
			}

			//classify
			if (r_hist_H_mat.at<float>(h_idx_i) >= thresh_H_f
					|| r_hist_I_mat.at<float>(i_idx_i) >= thresh_I_f) {
				r_mask_32F.at<float>(row, col) = 1;
			}
		}
	}
}

//use L as R, U as G, V as B component
void Jimenez::createSegmentMask(cv::Mat &r_pixel_mask_32F,
		cv::Mat &r_segment_mask_8U) {
	int* ptr_components_i;
	cv::Mat image2Segment_8U = m_ptr_common->luv32F_2_luv8U(*m_ptr_luvImg_32F);

	//prepare
	image<rgb> *ptr_segmentImg_rgb = new image<rgb>(m_imgWidth_i, m_imgHeight_i,
			1);
	for (int row = 0; row < m_imgHeight_i; ++row) {
		for (int col = 0; col < m_imgWidth_i; ++col) {
			rgb* ptr = imPtr(ptr_segmentImg_rgb, col, row);
			ptr->b = image2Segment_8U.at<cv::Vec3b>(row, col)[0];
			ptr->g = image2Segment_8U.at<cv::Vec3b>(row, col)[1];
			ptr->r = image2Segment_8U.at<cv::Vec3b>(row, col)[2];
		}
	}

	//segment image
	if (ptr_segmentImg_rgb != NULL) {
		image<rgb> *ptr_resultImg_rgb;
		ptr_resultImg_rgb = NULL;

		std::unordered_map<uint32_t, Region> * u_classified = segment_image(
				ptr_segmentImg_rgb, ptr_resultImg_rgb, r_pixel_mask_32F,
				*m_ptr_intensityImg_32F, m_ptr_common->getFelzSigmaD(),
				m_ptr_common->getFelzKI(), m_ptr_common->getFelzMinSizeI(),
				m_ptr_common->getSegmentClassificationThreshD(),
				ptr_components_i, false, true);

		r_segment_mask_8U = cv::Mat::zeros(ptr_resultImg_rgb->height(),
				ptr_resultImg_rgb->width(), CV_8UC3);
		if (ptr_resultImg_rgb->width() == ptr_segmentImg_rgb->width()) {
			for (int row = 0; row < ptr_segmentImg_rgb->height(); ++row) {
				for (int col = 0; col < ptr_segmentImg_rgb->width(); ++col) {
					r_segment_mask_8U.at<cv::Vec3b>(row, col)[0] =
							imRef(ptr_resultImg_rgb, col, row).b;
					r_segment_mask_8U.at<cv::Vec3b>(row, col)[1] =
							imRef(ptr_resultImg_rgb, col, row).g;
					r_segment_mask_8U.at<cv::Vec3b>(row, col)[2] =
							imRef(ptr_resultImg_rgb, col, row).r;
				}
			}
		}

		//clean up
		if (ptr_segmentImg_rgb != NULL) {
			delete ptr_segmentImg_rgb;
		}
		if (ptr_resultImg_rgb != NULL) {
			delete ptr_resultImg_rgb;
		}
		if (u_classified != NULL) {
			delete u_classified;
		}
	}
}

void Jimenez::show() {
	m_ptr_common->show("hue_image",
			m_ptr_common->convertImage_grey_Float2Uchar(*m_ptr_hueImg_32F));
	m_ptr_common->show("intensityImg_32F",
			m_ptr_common->convertImage_grey_Float2Uchar(
					*m_ptr_intensityImg_32F));
	m_ptr_common->show("luvImg_32F",
			m_ptr_common->luv32F_2_luv8U(*m_ptr_luvImg_32F));
}
void Jimenez::writeResults() {
	m_ptr_common->writeToPNG("hue_image",
			m_ptr_common->convertImage_grey_Float2Uchar(*m_ptr_hueImg_32F));
	m_ptr_common->writeToPNG("intensityImg_32F",
			m_ptr_common->convertImage_grey_Float2Uchar(
					*m_ptr_intensityImg_32F));
	m_ptr_common->writeToPNG("luvImg_32F",
			m_ptr_common->luv32F_2_luv8U(*m_ptr_luvImg_32F));
}
void Jimenez::writeTimings() {
	ROS_INFO("compute histograms: %lu ms", m_computeHistograms_time);
	ROS_INFO("compute histograms: %lu ms", m_createPixelMask_time);
}
#endif
