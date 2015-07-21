#ifndef DEPTH_IMAGE_CPP
#define DEPTH_IMAGE_CPP
#include <disp_image.h>

DisparityImage::DisparityImage() {
}
DisparityImage::DisparityImage(cv::Mat image) :
		cv::Mat(image) {
	m_min_f = 1000.0f;
	m_max_f = -1000.0f;
	m_resolution_f = 0.07f;

	//set minimum and maximum
	for (int i = 0; i < rows; ++i) {
		float* Di = ptr<float>(i);
		for (int j = 0; j < cols; ++j) {
			if (Di[j] < m_min_f) {
				m_min_f = Di[j];
			}
			if (Di[j] > m_max_f) {
				m_max_f = Di[j];
			}
		}
	}
}

std::vector<DisparityImage> DisparityImage::create_UV_dispImages() {
	std::vector<DisparityImage> img_vec = std::vector<DisparityImage>();
	int orgImgWidth_i = cols;
	int orgImgHeight_i = rows;

	int u_dispImgWidth_i = orgImgWidth_i;
	int u_dispImgHeight_i =
			(int) ((m_max_f - m_min_f) * (1.0f / m_resolution_f));
	int v_dispImgWidth_i = (int) ((m_max_f - m_min_f) * (1.0f / m_resolution_f));
	int v_dispImgHeight_i = orgImgHeight_i;

	cv::Mat u_dispImg_32F = cv::Mat(u_dispImgHeight_i, u_dispImgWidth_i,
			CV_32F);
	cv::Mat v_dispImg_32F = cv::Mat(v_dispImgHeight_i, v_dispImgWidth_i,
			CV_32F);

	u_dispImg_32F = cv::Mat::zeros(u_dispImg_32F.rows, u_dispImg_32F.cols,
			CV_32FC1);
	v_dispImg_32F = cv::Mat::zeros(v_dispImg_32F.rows, v_dispImg_32F.cols,
			CV_32FC1);

	for (int row = 0; row < orgImgHeight_i; ++row) {
		for (int col = 0; col < orgImgWidth_i; ++col) {
			float disp_f = at<float>(row, col);
			int d_idx_i = (int) (disp_f * (1.0f / m_resolution_f));
			if (d_idx_i >= u_dispImgHeight_i) {
				d_idx_i = u_dispImgHeight_i - 1;
			}
			u_dispImg_32F.at<float>(d_idx_i, col) += 1;
			v_dispImg_32F.at<float>(row, d_idx_i) += 1;
		}
	}
	img_vec.push_back(DisparityImage(u_dispImg_32F));
	img_vec.push_back(DisparityImage(v_dispImg_32F));
	return img_vec;
}

void DisparityImage::create_UV_dispImages(DisparityImage &r_VDisp,
		DisparityImage &r_UDisp) {
	//std::vector<DisparityImage> img_vec = std::vector<DisparityImage>();
	int orgImgWidth_i = cols;
	int orgImgHeight_i = rows;

	int u_dispImgWidth_i = orgImgWidth_i;
	int u_dispImgHeight_i =
			(int) ((m_max_f - m_min_f) * (1.0f / m_resolution_f));
	int v_dispImgWidth_i = (int) ((m_max_f - m_min_f) * (1.0f / m_resolution_f));
	int v_dispImgHeight_i = orgImgHeight_i;

	cv::Mat u_dispImg_32F = cv::Mat(u_dispImgHeight_i, u_dispImgWidth_i,
			CV_32F);
	cv::Mat v_dispImg_32F = cv::Mat(v_dispImgHeight_i, v_dispImgWidth_i,
			CV_32F);

	u_dispImg_32F = cv::Mat::zeros(u_dispImg_32F.rows, u_dispImg_32F.cols,
			CV_32FC1);
	v_dispImg_32F = cv::Mat::zeros(v_dispImg_32F.rows, v_dispImg_32F.cols,
			CV_32FC1);

	for (int row = 0; row < orgImgHeight_i; ++row) {
		for (int col = 0; col < orgImgWidth_i; ++col) {
			float disp_f = at<float>(row, col);
			int d_idx_i = (int) (disp_f * (1.0f / m_resolution_f));
			if (d_idx_i >= u_dispImgHeight_i) {
				d_idx_i = u_dispImgHeight_i - 1;
			}
			u_dispImg_32F.at<float>(d_idx_i, col) += 1;
			v_dispImg_32F.at<float>(row, d_idx_i) += 1;
		}
	}
	r_VDisp = DisparityImage(v_dispImg_32F);
	r_UDisp = DisparityImage(u_dispImg_32F);
	//return img_vec;
}

void DisparityImage::create_UV_dispImages(cv::Mat & r_Vdisp, cv::Mat &r_Udisp) {
	int v_Width_i = (int) ((m_max_f - m_min_f) * 1.0f / m_resolution_f);
	int v_Height_i = rows;
	int u_Width_i = cols;
	int u_Height_i = v_Width_i;

	r_Vdisp = cv::Mat::zeros(v_Height_i, v_Width_i, CV_32FC1);
	r_Udisp = cv::Mat::zeros(u_Height_i, u_Width_i, CV_32FC1);

	for (int row = 0; row < rows; ++row) {
		for (int col = 0; col < cols; ++col) {
			float disp_f = at<float>(row, col);
			int d_idx_i = (int) (disp_f * (1.0f / m_resolution_f));
			if (d_idx_i >= v_Width_i) {
				d_idx_i = v_Width_i - 1;
			}
			r_Vdisp.at<float>(row, d_idx_i) += 1;
			r_Udisp.at<float>(d_idx_i, col) += 1;
		}
	}
}

cv::Mat DisparityImage::create_visu_image() {
	cv::Mat resultImg_8U(rows, cols, CV_8UC1);
	for (int i = 0; i < rows; ++i) {
		float* Di = ptr<float>(i);
		char* Ii = resultImg_8U.ptr<char>(i);
		for (int j = 0; j < cols; ++j) {
			Ii[j] = (char) (255.0f * ((Di[j] - m_min_f) / (m_max_f - m_min_f)));
		}
	}
	return resultImg_8U;
}
#endif
