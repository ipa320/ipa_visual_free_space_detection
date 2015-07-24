#ifndef LINE_DETECTION_CPP
#define LINE_DETECTION_CPP
#include <lineDetection.h>

LineDetection::LineDetection() {
	m_debug_b = false;
	m_houghThresh_f = 450;
	m_houghRadRes_f = 2;
	m_houghThetaRes_f = CV_PI / 180;
	m_houghMinLineLength_f = 250;
	m_houghMaxLineGap_f = 0;
}

void LineDetection::probHough(cv::Mat &r_img_8U,
		std::vector<cv::Vec4i> &r_lines_vec) {
	cv::HoughLinesP(r_img_8U, r_lines_vec, m_houghRadRes_f, m_houghThetaRes_f,
			m_houghThresh_f, m_houghMinLineLength_f, m_houghMaxLineGap_f);
}

void LineDetection::hough(const cv::Mat &r_img_8U,
		std::vector<Line> &r_lines_vec) {
	std::vector<cv::Vec4i> tmpLines_vec = std::vector<cv::Vec4i>();

	cv::Mat filteredImg_8U;
	r_img_8U.copyTo(filteredImg_8U);

	//filter with Gauss and set all points with intensity < thresh to zero
	cv::GaussianBlur(filteredImg_8U, filteredImg_8U, cv::Size(0, 0), 0.7f);
	//ToDO -> outsource
	float thresh_f = 10.0f;
	for (int row = 0; row < filteredImg_8U.rows; ++row) {
		for (int col = 0; col < filteredImg_8U.cols; ++col) {
			if (filteredImg_8U.at<unsigned char>(
					filteredImg_8U.cols * row + col) < thresh_f) {
				filteredImg_8U.at<unsigned char>(
						filteredImg_8U.cols * row + col) = 0;
			}
		}
	}
	probHough(filteredImg_8U, tmpLines_vec);

	for (uint i = 0; i < tmpLines_vec.size(); ++i) {
		cv::Point2f p1 = cv::Point2f(tmpLines_vec.at(i)[0],
				tmpLines_vec.at(i)[1]);
		cv::Point2f p2 = cv::Point2f(tmpLines_vec.at(i)[2],
				tmpLines_vec.at(i)[3]);
		Line line = Line(p1, p2);
		r_lines_vec.push_back(line);
	}
}

void LineDetection::drawLines(cv::Mat &r_img_8U,
		const std::vector<Line> &r_lines_vec) {
	r_img_8U = cv::Mat::zeros(r_img_8U.rows, r_img_8U.cols, CV_8U);
	Line groundLine = Line(cv::Point2f(0, 0), cv::Point2f(0, 0));

	for (uint i = 0; i < r_lines_vec.size(); ++i) {
		Line line = r_lines_vec.at(i);
		line.draw(r_img_8U, cv::Scalar(100, 100, 100));

		if (i == 0) {
			groundLine = line;
		}
		if (line.getLength() > groundLine.getLength()) {
			groundLine = line;
		}

	}
	//groundLine.draw(r_img_8U, cv::Scalar(255,255,255));

}

Line LineDetection::getGroundLine(const std::vector<Line> &r_lines_vec) {
	Line groundLine = Line(cv::Point2f(0, 0), cv::Point2f(0, 0));
	float maxLineLength_f = 0;
	if (m_debug_b) {
		printf("getGroundLine\n");
	}
	for (uint i = 0; i < r_lines_vec.size(); ++i) {
		Line line = r_lines_vec.at(i);
		float length_f = line.getLength();

		if (m_debug_b) {
			printf("line %i -> m = %.1f\n", i, line.getGradient());
		}
		//groundLine in V disp Image is the longest non-vertical line (m!=INFINITY) -> m < 1
		if (length_f > maxLineLength_f && fabs(line.getGradient()) < 1.0f) {
			groundLine = line;
			maxLineLength_f = length_f;
		}
	}
	if (m_debug_b) {
		printf("GroundLine with %.0f/%.0f\n", groundLine.m_start.x,
				groundLine.m_start.y);
	}
	return groundLine;
}

void LineDetection::init_hough(float thresh_f, float rad_f, float theta_f,
		float minLen_f, float maxGap_f) {
	m_houghThresh_f = thresh_f;
	m_houghRadRes_f = rad_f;
	m_houghThetaRes_f = theta_f;
	m_houghMinLineLength_f = minLen_f;
	m_houghMaxLineGap_f = maxGap_f;
}

std::vector<Line> LineDetection::detect_lines(const cv::Mat &r_img_8U) {
	std::vector<Line> lines = std::vector<Line>();
	hough(r_img_8U, lines);
	return lines;
}
#endif
