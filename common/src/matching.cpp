#ifndef MATCHING_CPP
#define MATCHING_CPP
#include <matching.h>

float getDistance(cv::Point2f p1, cv::Point2f p2) {
	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

std::vector<int> Matching::DB_regionQuery(std::vector<Line> &r_lines_vec,
		Line &r_line, float epsPt_f, float epsM_f) {
	std::vector<int> retKeys_vec;
	for (size_t i = 0; i < r_lines_vec.size(); ++i) {
		bool condition_b = (r_line.m_start.x < (r_lines_vec.at(i)).m_start.x);
		Line l1 = condition_b ? r_line : r_lines_vec.at(i);
		Line l2 = condition_b ? r_lines_vec.at(i) : r_line;
		float m1_f = l1.getGradient();
		float m2_f = l2.getGradient();

		bool sameGradSign_b = (m1_f <= 0 && m2_f <= 0) || (m1_f >= 0 && m2_f >= 0);
		double gradDist_d = fabs(m2_f - m1_f) + 0.001f;
		if (!sameGradSign_b) {
			gradDist_d *= 10.0f;
		}
		double ptDist_d = getDistance(l1.m_start, l2.m_start)
				+ getDistance(l1.m_end, l2.m_end);

		gradDist_d = s_alpha_f * gradDist_d + s_beta_f * ptDist_d;

		if (gradDist_d < s_distThresh_f) {
			retKeys_vec.push_back(i);
		}
	}
	return retKeys_vec;
}

void Matching::cluster(std::vector<Line> &r_lines_vec,
		std::vector<std::vector<Line> > &cluster) {
	if (r_lines_vec.empty()) {
		std::cout << "empty vector -> no lines to cluster!" << std::endl;
	} else {
		cluster = DBSCAN(r_lines_vec, s_eps_f, s_epsM_f, s_minPoints_i);
	}
}

//DBScan algorithm (adapted to lines)
std::vector<std::vector<Line> > Matching::DBSCAN(std::vector<Line> &r_lines_vec,
		float eps_f, float epsM_f, unsigned int minLines_i) {
	std::vector<std::vector<Line> > clusters;
	std::vector<bool> clustered, visited;
	std::vector<int> noise;
	std::vector<int> neighborLines, neighborLines_;
	int c;

	size_t noKeys = r_lines_vec.size();

	//init clustered and visited
	for (size_t k = 0; k < noKeys; ++k) {
		clustered.push_back(false);
		visited.push_back(false);
	}

	c = 0;
	clusters.push_back(std::vector<Line>());

	//for each unvisted point P in dataset points
	for (size_t i = 0; i < noKeys; ++i) {
		if (!visited[i]) {
			//Mark P as visited
			visited[i] = true;
			neighborLines = DB_regionQuery(r_lines_vec, r_lines_vec.at(i),
					eps_f, epsM_f);
			if (neighborLines.size() < minLines_i) {
				//Mark P as Noise
				noise.push_back(i);
			} else {
				clusters.push_back(std::vector<Line>());
				++c;
				//expand cluster -> add P to cluster c
				clusters[c].push_back(r_lines_vec.at(i));

				//for each point P' in neighborPts
				for (size_t j = 0; j < neighborLines.size(); ++j) {
					//if P' is not visited
					if (!visited[neighborLines[j]]) {
						//Mark P' as visited
						visited[neighborLines[j]] = true;

						neighborLines_ = DB_regionQuery(r_lines_vec,
								r_lines_vec.at(neighborLines[j]), eps_f,
								epsM_f);
						neighborLines.insert(neighborLines.end(),
								neighborLines_.begin(), neighborLines_.end());
					}
					// if P' is not yet a member of any cluster
					// add P' to cluster c
					if (!clustered[neighborLines[j]]) {
						clusters[c].push_back(r_lines_vec.at(neighborLines[j]));
					}
				}
			}
		}
	}
	return clusters;
}

std::vector<Line> Matching::processCluster(
		std::vector<std::vector<Line> > &r_cluster_vec) {
	std::vector<Line> result_vec = std::vector<Line>();
	for (size_t cl = 0; cl < r_cluster_vec.size(); ++cl) {
		std::vector<Line> cluster = r_cluster_vec.at(cl);
		if (!cluster.empty()) {
			//calculate "mean" line
			float avg_sX_f = 0;
			float avg_sY_f = 0;
			float avg_eX_f = 0;
			float avg_eY_f = 0;
			float min_sX_f = 1000.0f;
			float min_sY_f = 1000.0f;
			float min_eX_f = 1000.0f;
			float min_eY_f = 1000.0f;
			float max_sX_f = 0;
			float max_sY_f = 0;
			float max_eX_f = 0;
			float max_eY_f = 0;
			int size_i = cluster.size();

			for (size_t i = 0; i < cluster.size(); ++i) {
				Line line = cluster.at(i);
				avg_sX_f += line.m_start.x;
				avg_sY_f += line.m_start.y;
				avg_eX_f += line.m_end.x;
				avg_eY_f += line.m_end.y;

				//find minimum and maximum
				if (line.m_start.x < min_sX_f) {
					min_sX_f = line.m_start.x;
				}
				if (line.m_start.y < min_sY_f) {
					min_sY_f = line.m_start.y;
				}
				if (line.m_end.x < min_eX_f) {
					min_eX_f = line.m_end.x;
				}
				if (line.m_end.y < min_eY_f) {
					min_eY_f = line.m_end.y;
				}
				if (line.m_start.x > max_sX_f) {
					max_sX_f = line.m_start.x;
				}
				if (line.m_start.y > max_sY_f) {
					max_sY_f = line.m_start.y;
				}
				if (line.m_end.x > max_eX_f) {
					max_eX_f = line.m_end.x;
				}
				if (line.m_end.y > max_eY_f) {
					max_eY_f = line.m_end.y;
				}
			}
			avg_sX_f /= size_i;
			avg_sY_f /= size_i;
			avg_eX_f /= size_i;
			avg_eY_f /= size_i;
			float lineWidth_f = (sqrt(
					pow((max_sX_f - min_sX_f), 2)
							+ pow((min_sY_f - max_sY_f), 2))
					+ sqrt(
							pow((max_eX_f - min_eX_f), 2)
									+ pow((min_eY_f - max_eY_f), 2))) / 2.0f;

			//ToDo -> improve
			lineWidth_f = lineWidth_f < 10 ? 10 : lineWidth_f;
			Line meanLine = Line(cv::Point2f(avg_sX_f, avg_sY_f),
					cv::Point2f(avg_eX_f, avg_eY_f), lineWidth_f);
			result_vec.push_back(meanLine);
		}
	}
	return result_vec;
}
#endif
