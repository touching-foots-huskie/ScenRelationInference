#include "parse_tool.h"

std::string GetCurrentWorkingDir( void ) {
    // current working directory
    char buff[FILENAME_MAX];
    GetCurrentDir( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir;
};

// Parse Matrix in Transpose version
Eigen::MatrixXd MatrixParseT(configuru::Config& config_array) {
	int dim1 = config_array.array_size();
	int dim2 = config_array[0].array_size();

	Eigen::MatrixXd parsed_matrix;
	parsed_matrix.resize(dim2, dim1);
	int count_1 = 0;
	for (auto v : config_array.as_array()) {
		int count_2 = 0;
		for (auto p : v.as_array()) {
			parsed_matrix(count_2, count_1) = (double)p;
			count_2++;
		}
		count_1++;
	}
	return parsed_matrix;
};

Eigen::MatrixXd VetorParse(configuru::Config& config_array) {
	int dim1 = config_array.array_size();
	Eigen::MatrixXd parsed_vector;
	parsed_vector.resize(dim1, 1);
	int count = 0;
	for (auto p : config_array.as_array()) {
		parsed_vector(count) = (double)p;
		count++;
	}
	return parsed_vector;
};

//Write Eigen Matrix into Config array


// Here original_matrix is not Ref, for it has to be resized
void ConcatMatrix(Eigen::MatrixXd& original_matrix,
				  const Eigen::Ref<Eigen::MatrixXd>& new_matrix) {
	Eigen::MatrixXd temp_matrix = original_matrix;
	original_matrix.resize(original_matrix.rows(), original_matrix.cols() + new_matrix.cols());
	original_matrix << temp_matrix, new_matrix;
};

void RemoveMatrix(Eigen::MatrixXd& original_matrix, int start_index, int cols_num) {
	Eigen::MatrixXd upper_matrix = original_matrix.block(0, 0, original_matrix.rows(), start_index);
	Eigen::MatrixXd lower_matrix = original_matrix.block(0, start_index + cols_num,
								   original_matrix.rows(), cols_num);
	original_matrix.resize(upper_matrix.rows(), upper_matrix.cols() + lower_matrix.cols());
	original_matrix << upper_matrix, lower_matrix;
};