#ifndef SORTING_CPP
#define SORTING_CPP
#include <sorting.h>

struct sortLines_ByX: public std::binary_function<Line, Line, bool> {
	bool operator()(const Line& l1, const Line& l2) const {
		return l1.m_start.x < l2.m_start.x;
	}
};
struct sortLines_ByY: public std::binary_function<Line, Line, bool> {
	bool operator()(const Line& l1, const Line& l2) const {
		return l1.m_start.y < l2.m_start.y;
	}
};
struct sortLines_ByX_and_Y: public std::binary_function<Line, Line, bool> {
	bool operator()(const Line& l1, const Line& l2) const {
		return (l1.m_start.x < l2.m_start.x)
				|| ((l1.m_start.x == l2.m_start.x)
						&& (l1.m_start.y < l2.m_start.y));
	}
};

void Sorting::sort(std::vector<Line> &lines) {
	std::sort(lines.begin(), lines.end(), sortLines_ByX_and_Y());
}

void Sorting::drawIndices(std::vector<Line> &lines, cv::Mat &img) {
	cv::Scalar color = cv::Scalar(255, 0, 0, 255);
	for (uint i = 0; i < lines.size(); ++i) {
		Line line = lines.at(i);
		std::stringstream idx;
		idx << i;
		cv::putText(img, idx.str().c_str(), line.m_start, 1, 1.0f, color, 1, 5,
				false);
	}
}
#endif
