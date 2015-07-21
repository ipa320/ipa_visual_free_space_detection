#ifndef SORTING_H
#define SORTING_H
#include <line.h>

class Sorting {
private:
public:
	static void sort(std::vector<Line> &r_lines_vec);
	static void drawIndices(std::vector<Line> &r_lines_vec, cv::Mat &r_img);
};
#endif
