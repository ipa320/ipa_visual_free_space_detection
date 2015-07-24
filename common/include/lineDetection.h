#ifndef LINE_DETECTION
#define LINE_DETECTION

#include <line.h>

class LineDetection {
private:
	bool m_debug_b;
	float m_houghThresh_f;
	float m_houghRadRes_f;
	float m_houghThetaRes_f;
	float m_houghMinLineLength_f;
	float m_houghMaxLineGap_f;

	void probHough(cv::Mat &r_img_8U, std::vector<cv::Vec4i> &r_lines_vec);
	void hough(const cv::Mat &r_img_8U, std::vector<Line> &r_lines_vec);

public:
	LineDetection();
	void init_hough(float thresh_f, float rad_f, float theta_f, float minLen_f,
			float maxGap_f);
	std::vector<Line> detect_lines(const cv::Mat &r_img_8U);
	void drawLines(cv::Mat &r_img_8U, const std::vector<Line> &r_lines_vec);
	Line getGroundLine(const std::vector<Line> &r_lines_vec);
};

#endif
