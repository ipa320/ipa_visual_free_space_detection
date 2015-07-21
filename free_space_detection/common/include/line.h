#ifndef LINE_H
#define LINE_H

class Line {
private:
	float m_width_f;
public:
	cv::Point2f m_start;
	cv::Point2f m_end;

	Line(cv::Point2f s, cv::Point2f e, float width_f = 10) {
		//check that start < end
		m_start = s.x < e.x ? s : e;
		m_end = s.x < e.x ? e : s;
		m_width_f = width_f;
	}

	void draw(cv::Mat &img, cv::Scalar color, float thickness = 1) {
		cv::line(img, m_start, m_end, color, thickness, CV_AA);
	}

	float getLength() {
		return sqrt(pow(m_end.x - m_start.x, 2) + pow(m_end.y - m_start.y, 2));
	}

	float getWidth() const {
		return m_width_f;
	}

	float getDistance(cv::Point2f p1, cv::Point2f p2) {
		return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
	}

	bool onSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r) {
		if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x)
				&& q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
			return true;

		return false;
	}

	float getGradient() {
		float dx = (m_end.x - m_start.x);
		float dy = (m_end.y - m_start.y);
		if (dx != 0) {
			return dy / dx;
		} else {
			return 1000.0f;
		}
	}

	float getYIntersection() {
		return (m_start.y - getGradient() * m_start.x);
	}

	bool doIntersect(Line &l2) {
		float m1 = getGradient();
		float m2 = l2.getGradient();
		bool sameMVZ = m1 <= 0 && m2 <= 0 || m1 >= 0 && m2 >= 0;
		bool similarGradient = false;

		if (sameMVZ && fabs(m2 - m1) <= 0.1f)
			similarGradient = true;

		cv::Point2f p1 = m_start;
		cv::Point2f q1 = m_end;
		cv::Point2f p2 = l2.m_start;
		cv::Point2f q2 = l2.m_end;

		// p1, q1 and p2 are colinear and p2 lies on segment p1q1
		if (similarGradient && onSegment(p1, p2, q1))
			return true;
		// p1, q1 and p2 are colinear and q2 lies on segment p1q1
		if (similarGradient && onSegment(p1, q2, q1))
			return true;
		// p2, q2 and p1 are colinear and p1 lies on segment p2q2
		if (similarGradient && onSegment(p2, p1, q2))
			return true;
		// p2, q2 and q1 are colinear and q1 lies on segment p2q2
		if (similarGradient && onSegment(p2, q1, q2))
			return true;

		return false; // Doesn't fall in any of the above cases
	}

	double getMinDistance(Line &l2) {
		//check if lines intersect --> return 0, else check real distance
		double dist = 0;
		bool intersect = doIntersect(l2);
		if (!intersect) {
			double min_01 = std::min(getDistance(m_start, l2.m_start),
					getDistance(m_start, l2.m_end));
			double min_02 = std::min(getDistance(m_end, l2.m_start),
					getDistance(m_end, l2.m_start));
			dist = std::min(min_01, min_02);
		}
		return dist;
	}

	float getDistance(cv::Point &point) {
		float x_1 = m_start.x;
		float y_1 = m_start.y;
		float x_2 = m_end.x;
		float y_2 = m_end.y;
		float Dx = x_2 - x_1;
		float Dy = y_2 - y_1;

		float dist = abs(
				Dy * (float) point.x - Dx * (float) point.y - x_1 * y_2
						+ x_2 * y_1);
		dist /= sqrt(pow(Dx, 2) + pow(Dy, 2));
		return dist;
	}
};

#endif
