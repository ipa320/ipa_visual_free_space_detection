#ifndef MATCHING_H
#define MATCHING_H

class Matching {
private:
	static constexpr float s_alpha_f = 1.0f;
	static constexpr float s_beta_f = 2.0f;
	static constexpr float s_distThresh_f = 180.0f;
	static constexpr float s_eps_f = 5.5f;
	static constexpr float s_epsM_f = 0.02f;
	static constexpr int s_minPoints_i = 1;

public:
	Matching() {
	}
	~Matching() {
	}

	static void match(std::vector<Line> &lines_01, std::vector<Line> &lines_02,
			std::vector<double> &distances);
	static void match(std::vector<Line> &lines);
	static void drawMatches(std::vector<Line> &lines_, cv::Mat &img,
			std::vector<double> &distances);

	//Segmentation with DBScan
	std::vector<std::vector<Line> > DBSCAN(std::vector<Line> &r_lines_vec,
			float eps_f, float epsM_f, int minLength_f);
	std::vector<int> DB_regionQuery(std::vector<Line> &r_lines_vec,
			Line &r_line, float epsPt_f, float epsM_f);
	void cluster(std::vector<Line> &r_lines_vec,
			std::vector<std::vector<Line> > &r_cluster_vec);
	std::vector<Line> processCluster(
			std::vector<std::vector<Line> > &r_cluster_vec);
};

#endif
