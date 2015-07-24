/*
 Copyright (C) 2006 Pedro Felzenszwalb

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef DISJOINT_SET
#define DISJOINT_SET

// disjoint-set forests using union-by-rank and path compression (sort of).

typedef struct {
	int rank;
	int p;
	int size;
	//added member
	double label_prob;
	double local_mean;
	double local_variance;
} uni_elt;

class universe {
public:
	universe(int elements);
	~universe();
	int find(int x);
	void join(int x, int y);
	int size(int x) const {
		return elts[x].size;
	}
	int num_sets() const {
		return num;
	}

	//added functionality
	double getProbability(int idx);
	void setProbability(int idx, double value_f);
	//int 		getSize			(int idx);

	double getMean(int idx);
	void setMean(int idx, double value_f);
	double getVariance(int idx);
	void setVariance(int idx, double value_f);

private:
	uni_elt *elts;
	int num;
};

universe::universe(int elements) {
	elts = new uni_elt[elements];
	num = elements;
	for (int i = 0; i < elements; i++) {
		elts[i].rank = 0;
		elts[i].size = 1;
		elts[i].p = i;
		//added functionality
		elts[i].label_prob = 0.5f;
		elts[i].local_mean = 0;
		elts[i].local_variance = 1.0f;
	}
}

universe::~universe() {
	delete[] elts;
}

int universe::find(int x) {
	int y = x;
	while (y != elts[y].p)
		y = elts[y].p;
	elts[x].p = y;
	return y;
}

void universe::join(int x, int y) {
	if (elts[x].rank > elts[y].rank) {
		int size_x = elts[x].size;
		elts[y].p = x;
		elts[x].size += elts[y].size;
		//added functionality
		//use old x size and divide by new size
		elts[x].label_prob = (size_x * elts[x].label_prob
				+ elts[y].size * elts[y].label_prob) / elts[x].size;
		elts[x].local_mean = (size_x * elts[x].local_mean
				+ elts[y].size * elts[y].local_mean) / elts[x].size;
		elts[x].local_mean += elts[y].local_mean;
	} else {
		elts[x].p = y;
		int size_y = elts[y].size;
		elts[y].size += elts[x].size;
		//added functionality
		//use old y size and divide by new size
		elts[y].label_prob = (elts[x].size * elts[x].label_prob
				+ size_y * elts[y].label_prob) / elts[y].size;
		elts[y].local_mean = (elts[x].size * elts[x].local_mean
				+ size_y * elts[y].local_mean) / elts[y].size;
		if (elts[x].rank == elts[y].rank) {
			elts[y].rank++;
		}
	}
	num--;
}

/**************************
 * added functionality 	*
 **************************/
double universe::getProbability(int idx) {
	return elts[idx].label_prob;
}
void universe::setProbability(int idx, double value_f) {
	elts[idx].label_prob = value_f;
}
double universe::getMean(int idx) {
	return elts[idx].local_mean;
}
void universe::setMean(int idx, double value_f) {
	elts[idx].local_mean = value_f;
}
double universe::getVariance(int idx) {
	return elts[idx].local_variance;
}
void universe::setVariance(int idx, double value_f) {
	elts[idx].local_variance = value_f;
}
#endif
