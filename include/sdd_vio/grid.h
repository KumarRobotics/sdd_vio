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


#ifndef GRID_H_
#define GRID_H_

#include <vector>
#include <Eigen/Dense>
#include "opencv2/core/version.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace sdd_vio {

class Grid
{
public:
	Grid(int im_rows, int im_cols, int row_size, int col_size);

	void resize(int im_rows, int im_cols, int row_size, int col_size);  // resize the grid
	void reset();

	/* draw lines on the image to show grid - details documented in handbook */
	void draw(cv::Mat& image);
	/* filter points, take input vector and their gradients */
	void prune(cv::Mat& G, cv::Mat& G_binary);
	/* check if r and c are valid grid row and col numbers */
	bool inside(int r, int c) const;

	int& cell(int r, int c) { return grid_[r * grid_cols_ + c]; };
	int num_rows() const {return grid_rows_;};
	int num_cols() const {return grid_cols_;};

private:
	int grid_rows_, grid_cols_, grid_num_;  // number of grids
	int row_size_, col_size_;  // length of one cell
	int row_size_end_, col_size_end_;  // the length of the cell at the right/down edge
	std::vector<int> grid_;
	int num_filled_;
};


}  // namespace sdd_vio


#endif
