/*-------------------------------------------------------------
Copyright 2019 Wenxin Liu, Kartik Mohta, Giuseppe Loianno

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
--------------------------------------------------------------*/

#include "sdd_vio/grid.h"
#include <ros/console.h>


namespace sdd_vio {

Grid::Grid(int im_rows, int im_cols, int row_size, int col_size) {
	resize(im_rows, im_cols, row_size, col_size);
	ROS_DEBUG_STREAM("number of grid rows:"<<grid_rows_);
	ROS_DEBUG_STREAM("number of grid cols:"<<grid_cols_);
}

void Grid::resize(int im_rows, int im_cols, int row_size, int col_size) {
	grid_rows_ = ceil(im_rows / row_size);  // take the ceiling
	grid_cols_ = ceil(im_cols / col_size);
	grid_num_ = grid_rows_ * grid_cols_;

	row_size_ = row_size;
	col_size_ = col_size;

	row_size_end_ = row_size;
	col_size_end_ = col_size;
	if (im_rows % row_size != 0)
		row_size_end_ = im_rows % row_size;
	if (im_cols % col_size != 0)
		col_size_end_ = im_cols % col_size;

	// Don't forget to resize
	grid_.resize(grid_num_);
	if (grid_num_ < 1) {
		ROS_WARN_STREAM("Grid object not properly initialized! Total number of cells smaller than 1.");
	}
	reset();
}

void Grid::reset() {
	std::fill(grid_.begin(), grid_.end(), 0);
	num_filled_ = 0;
}

void Grid::draw(cv::Mat& image) {
	// Draw rows
	for (int r = 1; r < grid_rows_; ++r) {
		int y = r*row_size_ - 1;
		cv::Point p1(0, y);
		cv::Point p2(image.cols, y);
		cv::line(image, p1, p2, cv::Scalar(0, 200, 0));
	}

	// Draw cols
	for (int c = 1; c < grid_cols_; ++c) {
		int x = c*col_size_ - 1;
		cv::Point p1(x, 0);
		cv::Point p2(x, image.rows);
		cv::line(image, p1, p2, cv::Scalar(0, 200, 0));
	}
}

void Grid::prune(cv::Mat& G, cv::Mat& G_binary) {

	for (int i = 0; i < grid_num_; ++i) {
		/* obtain rectangle info */
		int x = (i % grid_cols_)*col_size_;
		int y = (i / grid_cols_)*row_size_;
		int w = col_size_;
		int h = row_size_;
		if (i % grid_cols_ == grid_cols_-1)
			w = col_size_end_;
		if (i / grid_cols_ == grid_rows_-1)
			h = row_size_end_;

		/* set region of interest */
		cv::Mat binaryROI = G_binary(cv::Rect(x,y,w,h));
		cv::Mat gradROI = G(cv::Rect(x,y,w,h));

		/* if no binary point exist, skip */
		double min, max;
    	cv::minMaxLoc(binaryROI, &min, &max);
    	if (max == 0)
    		continue;

    	/* otherwise find maximum point location of gradROI and set binaryROI of max to 255 */
    	double minVal, maxVal;
		cv::Point minLoc, maxLoc;
		cv::minMaxLoc(gradROI, &minVal, &maxVal, &minLoc, &maxLoc);
		binaryROI.setTo(cv::Scalar(0));
		binaryROI.at<uchar>(maxLoc) = 255;

	}

}

bool Grid::inside(int r, int c) const {
  	return r >= 0 && r < grid_rows_ && c >= 0 && c < grid_cols_;
}


}  // namespace sdd_vio
