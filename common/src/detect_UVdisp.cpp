#ifndef UV_DISP_CPP
#define UV_DISP_CPP
#include <detect_UVdisp.h>

class Obstacle {
private:
	pcl::PointXYZ m_center;
	float m_width_f;
	float m_height_f;
public:
	Obstacle() :
			m_center(pcl::PointXYZ(0, 0, 0)), m_width_f(0), m_height_f(0) {
	}
	Obstacle(pcl::PointXYZ center, float width_f, float height_f) :
			m_center(center), m_width_f(width_f), m_height_f(height_f) {
	}
	~Obstacle() {
	}
};

UVDisp::UVDisp(FreespaceCommon* &r_ptr_common) {
	m_ptr_common = r_ptr_common;
	m_ptr_lineDetection = new LineDetection();
	m_ptr_lineDetection->init_hough(80, 2, CV_PI / 180, 50, 20);
	m_ptr_matching = new Matching();
}
UVDisp::~UVDisp() {
	if (m_ptr_lineDetection != NULL) {
		delete m_ptr_lineDetection;
		m_ptr_lineDetection = NULL;
	}
	if (m_ptr_matching != NULL) {
		delete m_ptr_matching;
		m_ptr_matching = NULL;
	}
}

//compute relative amount of invalid values in disparity image
bool UVDisp::isGoodQuality(const cv::Mat &r_image) {
	int width_i = r_image.cols;
	int height_i = r_image.rows;
	int invalid_i = 0;
	for (int row = 0; row < height_i; ++row) {
		for (int col = 0; col < width_i; ++col) {
			if (r_image.at<float>(row, col) == 0) {
				++invalid_i;
			}
		}
	}
	float result = ((float) invalid_i) / ((float) (width_i * height_i));
	//printf ("res = %f\n", result);
	return 1.0f - result > m_ptr_common->getMinValidDispThreshD();
}
//use texture info for handling invalid disparity (unknown, 0.5)
void UVDisp::handleInvalidValues(cv::Mat &r_disparity_freespace_mask_32F,
		const cv::Mat &r_hi_segmentation_mask_32F) {
	int height_i = r_disparity_freespace_mask_32F.rows;
	int width_i = r_disparity_freespace_mask_32F.cols;
	for (int row = 0; row < height_i; ++row) {
		for (int col = 0; col < width_i; ++col) {
			if (r_disparity_freespace_mask_32F.at<float>(row, col) == 0.5f) {
				r_disparity_freespace_mask_32F.at<float>(row, col) =
						r_hi_segmentation_mask_32F.at<float>(row, col);
			}
		}
	}
}

//break if very good line (< m_thresh_f) is found to get it faster --> ToDo: improve!
void UVDisp::findNearestLine(cv::Point &p, const std::vector<Line> &r_lines_vec,
		float m_thresh_f, int &r_bestIndex_i, float &r_minDist_f) {
	for (unsigned int lineIdx_i = 0; lineIdx_i < r_lines_vec.size(); ++lineIdx_i) {
		Line l = r_lines_vec.at(lineIdx_i);
		float dist_f = l.getDistance(p);
		if (dist_f < r_minDist_f) {
			r_minDist_f = dist_f;
			r_bestIndex_i = lineIdx_i;
			if (dist_f < m_thresh_f)
				break;
		}
	}
}
void UVDisp::classify(DisparityImage &r_orgDispImg_32F, Line &r_groundLine,
		cv::Mat &r_freespace_32F) {
	int orgWidth_i = r_orgDispImg_32F.getWidth();
	int orgHeight_i = r_orgDispImg_32F.getHeight();
	int vWidth_i =
			(int) ((r_orgDispImg_32F.getMax() - r_orgDispImg_32F.getMin())
					* 1.0f / r_orgDispImg_32F.getResolution());
	int vHeight_i = orgHeight_i;

	r_freespace_32F = cv::Mat::zeros(orgHeight_i, orgWidth_i, CV_32FC1);

	for (int row = 0; row < orgHeight_i; ++row) {
		for (int col = 0; col < orgWidth_i; ++col) {

			//get original disparity and compute disp position in V disp Image
			float disp_f = r_orgDispImg_32F.get(orgWidth_i * row + col);
			int d_idx_i = (int) (disp_f
					* (1.0f / r_orgDispImg_32F.getResolution()));

			//if invalid disparity
			if (disp_f <= 0) {
				//unknown
				r_freespace_32F.at<float>(row * orgWidth_i + col) = 0.5f;
			}

			if (disp_f > r_orgDispImg_32F.getMin()
					&& disp_f < r_orgDispImg_32F.getMax()) {
				//check if this point is on groundLine!
				cv::Point point_Vdisp = cv::Point(d_idx_i, row);
				float dist_f = r_groundLine.getDistance(point_Vdisp);
				if (dist_f < m_ptr_common->getVdispThreshD()) {
					r_freespace_32F.at<float>(row * orgWidth_i + col) = 1;
				}
			}
		}
	}
}

//ToDo
void UVDisp::assignLines(DisparityImage &r_orgDispImg_32F,
		std::vector<Line> clustered_U_vec, std::vector<Line> clustered_V_vec) {
	long unsigned int timeIt = m_ptr_common->get_time_ms64();
	int orgWidth_i = r_orgDispImg_32F.getWidth();
	int orgHeight_i = r_orgDispImg_32F.getHeight();

	int vWidth_i =
			(int) ((r_orgDispImg_32F.getMax() - r_orgDispImg_32F.getMin())
					* 1.0f / r_orgDispImg_32F.getResolution());
	int vHeight_i = orgHeight_i;

	int uHeight_i = vWidth_i;
	int uWidth_i = orgWidth_i;

	for (int row = 0; row < orgHeight_i; ++row) {
		for (int col = 0; col < orgWidth_i; ++col) {

			//get original disparity and compute disp position in V disp Image
			float disp_f = r_orgDispImg_32F.get(orgWidth_i * row + col);
			int d_idx_i = (int) (disp_f
					* (1.0f / r_orgDispImg_32F.getResolution()));
			if (disp_f > r_orgDispImg_32F.getMin()
					&& disp_f < r_orgDispImg_32F.getMax()) {
				//check if this point is on groundLine!
				cv::Point point_Vdisp = cv::Point(d_idx_i, row);
				cv::Point point_Udisp = cv::Point(col, d_idx_i);
				//ToDo
				//detect obstacles
			}
		}
	}
}

void UVDisp::detect(const cv::Mat &input_32F, cv::Mat &freespace_mask_32F) {
	long unsigned int time = m_ptr_common->get_time_ms64();
	DisparityImage orgDispImg, vDispImg, uDispImg;
	std::vector<Line> lines_U, lines_V, clustered_U_vec, clustered_V_vec;
	std::vector<vector<Line> > cluster_U = std::vector<vector<Line> >();
	std::vector<vector<Line> > cluster_V = std::vector<vector<Line> >();

	//create original DisparityImage
	orgDispImg = DisparityImage(input_32F);

	//create V Disparity image and create U Disparity image
	orgDispImg.create_UV_dispImages(vDispImg, uDispImg);

	//prepare line detection
	m_V_hough_1_resultImg_8U = cv::Mat::zeros(vDispImg.getHeight(),
			vDispImg.getWidth(), CV_8UC1);
	m_V_hough_2_resultImg_8U = cv::Mat::zeros(vDispImg.getHeight(),
			vDispImg.getWidth(), CV_8UC1);
	m_U_hough_1_resultImg_8U = cv::Mat::zeros(uDispImg.getHeight(),
			uDispImg.getWidth(), CV_8UC1);
	m_U_hough_2_resultImg_8U = cv::Mat::zeros(uDispImg.getHeight(),
			uDispImg.getWidth(), CV_8UC1);
	m_VDisp_resultImg_8U = vDispImg.create_visu_image();
	m_UDisp_resultImg_8U = uDispImg.create_visu_image();

	//do line detection -> ToDo -> improve (do this faster! Takes ca. 10 ms)
	//long unsigned int time_2 = m_ptr_common->get_time_ms64();
	lines_V = m_ptr_lineDetection->detect_lines(m_VDisp_resultImg_8U);
	lines_U = m_ptr_lineDetection->detect_lines(m_UDisp_resultImg_8U);
	//std::cout << "disp detect lines " << m_ptr_common->get_time_ms64()-time_2 << " ms, \n";

	//sort lines
	Sorting::sort(lines_V);
	Sorting::sort(lines_U);

	//cluster lines
	m_ptr_matching->cluster(lines_V, cluster_V);
	m_ptr_matching->cluster(lines_U, cluster_U);
	clustered_V_vec = m_ptr_matching->processCluster(cluster_V);
	clustered_U_vec = m_ptr_matching->processCluster(cluster_U);

	//detect ground line in V disparity image
	Line groundLine = m_ptr_lineDetection->getGroundLine(clustered_V_vec);

	//ToDo
	//assignLines(orgDispImg, clustered_V_vec, clustered_U_vec);

	//classify pixel in original disparity image
	classify(orgDispImg, groundLine, freespace_mask_32F);

	//ToDo final eval
	//m_ptr_common->writeToPNG("disparity", 	orgDispImg.create_visu_image());

	//visualization
	if (m_ptr_common->isDebugB() || m_ptr_common->isWriteResultsB()) {
		m_orgDisp_resultImg_8U = orgDispImg.create_visu_image();
		Line groundLine = m_ptr_lineDetection->getGroundLine(clustered_V_vec);

		//draw lines
		groundLine.draw(m_V_hough_2_resultImg_8U, cv::Scalar(255, 255, 255));
		m_ptr_lineDetection->drawLines(m_V_hough_2_resultImg_8U,
				clustered_V_vec);
		m_ptr_lineDetection->drawLines(m_V_hough_1_resultImg_8U, lines_V);
		m_ptr_lineDetection->drawLines(m_U_hough_1_resultImg_8U, lines_U);
		m_ptr_lineDetection->drawLines(m_U_hough_2_resultImg_8U,
				clustered_U_vec);

		//draw line indices
		Sorting::drawIndices(lines_V, m_V_hough_1_resultImg_8U);
		Sorting::drawIndices(lines_U, m_U_hough_1_resultImg_8U);
		Sorting::drawIndices(clustered_V_vec, m_V_hough_2_resultImg_8U);
		Sorting::drawIndices(clustered_U_vec, m_U_hough_2_resultImg_8U);

		if (m_ptr_common->isDebugB()) {
			show();
		}
		if (m_ptr_common->isWriteResultsB()) {
			writeResults();
		}
	}
}
void UVDisp::show() {
	m_ptr_common->show("org_disp", m_orgDisp_resultImg_8U);
	m_ptr_common->show("V_disp", m_VDisp_resultImg_8U);
	m_ptr_common->show("U_disp", m_UDisp_resultImg_8U);
	m_ptr_common->show("U_Hough_org", m_U_hough_1_resultImg_8U);
	m_ptr_common->show("V_Hough_org", m_V_hough_1_resultImg_8U);
	m_ptr_common->show("V_Hough", m_V_hough_2_resultImg_8U);
	m_ptr_common->show("U_Hough", m_U_hough_2_resultImg_8U);
}
void UVDisp::writeResults() {
	m_ptr_common->writeToPNG("org_disp", m_orgDisp_resultImg_8U);
	m_ptr_common->writeToPNG("V_disp", m_VDisp_resultImg_8U);
	m_ptr_common->writeToPNG("U_disp", m_UDisp_resultImg_8U);
	m_ptr_common->writeToPNG("V_Hough_org", m_V_hough_1_resultImg_8U);
	m_ptr_common->writeToPNG("U_Hough_org", m_U_hough_1_resultImg_8U);
	m_ptr_common->writeToPNG("V_Hough", m_V_hough_2_resultImg_8U);
	m_ptr_common->writeToPNG("U_Hough", m_U_hough_2_resultImg_8U);
}
#endif
