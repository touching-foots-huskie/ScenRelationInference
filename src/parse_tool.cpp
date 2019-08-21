#include "parse_tool.h"

std::string GetCurrentWorkingDir( void ) {
    // current working directory
    char buff[FILENAME_MAX];
    GetCurrentDir( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir;
};

Eigen::MatrixXd MatrixParse(const configuru::Config& config_array) {
	int dim1 = config_array.array_size();
	int dim2 = config_array[0].array_size();

	Eigen::MatrixXd parsed_matrix;
	parsed_matrix.resize(dim1, dim2);
	int count_1 = 0;
	for (auto v : config_array.as_array()) {
		int count_2 = 0;
		for (auto p : v.as_array()) {
			parsed_matrix(count_1, count_2) = (double)p;
			count_2++;
		}
		count_1++;
	}
	return parsed_matrix;
};

// Parse Matrix in Transpose version
Eigen::MatrixXd MatrixParseT(const configuru::Config& config_array) {
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

/*
Output of Vector parse is a column vector
 */
Eigen::MatrixXd VetorParse(const configuru::Config& config_array) {
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
void WriteMatrix4d(configuru::Config& config,
                   const Eigen::Ref<Eigen::MatrixXd>& matrix, 
                   std::string name){
    config[name] = configuru::Config::array(
        {
            configuru::Config::array({matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3)}),
            configuru::Config::array({matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3)}),
            configuru::Config::array({matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3)}),
            configuru::Config::array({matrix(3, 0), matrix(3, 1), matrix(3, 2), matrix(3, 3)}),
        }
    );
};

// Here original_matrix is not Ref, for it has to be resized
// If no matrix before, directly assign, else concat together
void ConcatMatrix(Eigen::MatrixXd& original_matrix,
				  const Eigen::Ref<Eigen::MatrixXd>& new_matrix) {
	if(original_matrix.rows() == 0){
        original_matrix = new_matrix;
    }
    else{
        Eigen::MatrixXd temp_matrix = original_matrix;
        original_matrix.resize(original_matrix.rows(), original_matrix.cols() + new_matrix.cols());
        original_matrix << temp_matrix, new_matrix;
    }
};

void RemoveMatrix(Eigen::MatrixXd& original_matrix, int start_index, int cols_num) {
	int total_num = original_matrix.cols();

    Eigen::MatrixXd upper_matrix = original_matrix.block(0, 0, original_matrix.rows(), start_index);
	Eigen::MatrixXd lower_matrix = original_matrix.block(0, start_index + cols_num,
								   original_matrix.rows(), total_num - (start_index + cols_num));
                                   
	original_matrix.resize(upper_matrix.rows(), upper_matrix.cols() + lower_matrix.cols());
	original_matrix << upper_matrix, lower_matrix;
};

void DisplayMatrix(const Eigen::Ref<Eigen::MatrixXd>& matrix, std::string name){
    std::cout << "-------" << name << "---------" << std::endl;
    std::cout << matrix;
    std::cout << std::endl;
};