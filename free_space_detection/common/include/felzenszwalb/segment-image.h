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

#ifndef SEGMENT_IMAGE
#define SEGMENT_IMAGE

#include <cstdlib>
#include <felzenszwalb/image.h>
#include <felzenszwalb/misc.h>
#include <felzenszwalb/filter.h>
#include "felzenszwalb/segment-graph.h"
#include <unordered_map>

typedef struct {
	int size;
	cv::Mat_<double> intensities;
	std::vector<uint32_t> pixelIndices;

	double prob;
	double mean;
	double variance;
	rgb color;
} Region;

// random color
rgb random_rgb() {
	rgb c;
	double r;

	c.r = (uchar) random();
	c.g = (uchar) random();
	c.b = (uchar) random();

	return c;
}

// dissimilarity measure between pixels
static inline float diff(image<float> *r, image<float> *g, image<float> *b,
		int x1, int y1, int x2, int y2) {
	return sqrt(
			square(imRef(r, x1, y1) - imRef(r, x2, y2))
					+ square(imRef(g, x1, y1) - imRef(g, x2, y2))
					+ square(imRef(b, x1, y1) - imRef(b, x2, y2)));
}

/*
 * Segment an image
 *
 * Returns graph.
 *
 * im: image to segment.
 * out: color image representing the segmentation (will be filled)
 * sigma: to smooth the image.
 * c: constant for treshold function.
 * min_size: minimum component size (enforced by post-processing stage).
 * num_ccs: number of connected components in the segmentation.

 * NOTES -> changed return value to graph and output image is reference to pointer image<rgb>, added binary mask
 */
std::unordered_map<uint32_t, Region>* segment_image(image<rgb> *im,
		image<rgb> *&out, cv::Mat &mask_32F, cv::Mat &r_intensity_32F,
		float sigma_f, float c_f, int min_size_i, float thresh_f, int *num_ccs,
		bool algo1_b, bool writeOutImg_b = false) {

	int width = im->width();
	int height = im->height();

	image<float> *r = new image<float>(width, height);
	image<float> *g = new image<float>(width, height);
	image<float> *b = new image<float>(width, height);

	// smooth each color channel
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			imRef(r, x, y) = imRef(im, x, y).r;
			imRef(g, x, y) = imRef(im, x, y).g;
			imRef(b, x, y) = imRef(im, x, y).b;
		}
	}
	image<float> *smooth_r = smooth(r, sigma_f);
	image<float> *smooth_g = smooth(g, sigma_f);
	image<float> *smooth_b = smooth(b, sigma_f);

	delete r;
	delete g;
	delete b;

	// build graph
	edge *edges = new edge[width * height * 4];
	int num = 0;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			if (x < width - 1) {
				edges[num].a = y * width + x;
				edges[num].b = y * width + (x + 1);
				edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x + 1,
						y);
				num++;
			}

			if (y < height - 1) {
				edges[num].a = y * width + x;
				edges[num].b = (y + 1) * width + x;
				edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x,
						y + 1);
				num++;
			}

			if ((x < width - 1) && (y < height - 1)) {
				edges[num].a = y * width + x;
				edges[num].b = (y + 1) * width + (x + 1);
				edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x + 1,
						y + 1);
				num++;
			}

			if ((x < width - 1) && (y > 0)) {
				edges[num].a = y * width + x;
				edges[num].b = (y - 1) * width + (x + 1);
				edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x + 1,
						y - 1);
				num++;
			}
		}
	}
	delete smooth_r;
	delete smooth_g;
	delete smooth_b;

	// segment
	universe *u = segment_graph(width * height, num, edges, c_f);

	// post process small components
	for (int i = 0; i < num; i++) {
		int a = u->find(edges[i].a);
		int b = u->find(edges[i].b);
		if ((a != b)
				&& ((u->size(a) < min_size_i) || (u->size(b) < min_size_i))) {
			u->join(a, b);
		}
	}
	delete[] edges;

	/**************************
	 * added functionality 	*
	 **************************/
	int numberRegions_i = u->num_sets();
	if (out == NULL) {
		out = new image<rgb>(width, height);
	}
	std::unordered_map<uint32_t, Region> *region_map = new std::unordered_map<
			uint32_t, Region>();

	//count free space probability
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			uint32_t pixelIdx_i = y * width + x;
			int regionIdx_i = u->find(pixelIdx_i);
			double prob_d = static_cast<double>(mask_32F.at<float>(pixelIdx_i)
					> 0.5f);
			double intensity_d = static_cast<double>(r_intensity_32F.at<float>(
					pixelIdx_i));

			if (region_map->count(regionIdx_i) == 0) {
				//create new Region, color of region can never be black
				Region region_f;
				int numPixel_i = u->size(regionIdx_i);
				region_f.size = numPixel_i;
				do {
					region_f.color = random_rgb();
				} while (region_f.color.r != 0 && region_f.color.g != 0
						&& region_f.color.b != 0);

				region_f.intensities.push_back(intensity_d);
				region_f.pixelIndices.push_back(pixelIdx_i);
				std::pair<std::uint32_t, Region> item(regionIdx_i, region_f);
				region_map->insert(item);
			} else {
				//if already existing, get prob value
				Region *region_f = &(region_map->at(regionIdx_i));
				region_f->prob += (prob_d / (double) region_f->size);
				region_f->intensities.push_back(intensity_d);
				region_f->pixelIndices.push_back(pixelIdx_i);
			}
		}
	}
	for (auto it = region_map->begin(); it != region_map->end(); ++it) {
		Region *region_f = &(it->second);
		cv::Scalar mean, stddev;
		meanStdDev(region_f->intensities, mean, stddev, cv::Mat());
		region_f->mean = mean.val[0];
		region_f->variance = stddev.val[0] * stddev.val[0];
	}

	//classify each segment
	if (writeOutImg_b) {
		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				double intensity_d = static_cast<double>(r_intensity_32F.at<
						float>(y, x));
				int comp = u->find(y * width + x);
				Region *region_f = &region_map->at(comp);

				//draw into output image
				rgb black;
				black.r = 0;
				black.g = 0;
				black.b = 0;
				if (region_f->prob > thresh_f) {
					imRef(out, x, y) = region_map->at(comp).color;
				} else {
					imRef(out, x, y) = black;
				}
			}
		}
	}

	if (u != NULL) {
		delete u;
	}
	return region_map;
}

#endif
