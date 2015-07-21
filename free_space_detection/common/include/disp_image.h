#ifndef DEPTH_IMAGE_H
#define DEPTH_IMAGE_H

class DisparityImage: cv::Mat {
private:
	float m_min_f;
	float m_max_f;
	float m_resolution_f;

public:
	DisparityImage();
	DisparityImage(cv::Mat img_32F);
	~DisparityImage() {
	}

	float getMin() const {
		return m_min_f;
	}
	float getMax() const {
		return m_max_f;
	}
	int getHeight() const {
		return rows;
	}
	int getWidth() const {
		return cols;
	}
	float getResolution() const {
		return m_resolution_f;
	}
	float get(int index) {
		return at<float>(index);
	}

	void create_UV_dispImages(cv::Mat & r_Vimg_32F, cv::Mat &r_Uimg_32F);
	std::vector<DisparityImage> create_UV_dispImages();
	void create_UV_dispImages(DisparityImage & r_Vdisp,
			DisparityImage &r_Udisp);
	cv::Mat create_visu_image();
};
#endif
