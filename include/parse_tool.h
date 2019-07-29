#ifndef PARSE_TOOL
#define PARSE_TOOL

#include "configuru.hpp"

#include <Eigen/Eigen>

#include <stdio.h>  /* defines FILENAME_MAX */
#include <unistd.h>
#define GetCurrentDir getcwd

std::string GetCurrentWorkingDir( void );

// Parse Matrix in Transpose version
Eigen::MatrixXd MatrixParseT(configuru::Config& config_array);
Eigen::MatrixXd VetorParse(configuru::Config& config_array);

// Here original_matrix is not Ref, for it has to be resized
void ConcatMatrix(Eigen::MatrixXd& original_matrix,
				  const Eigen::Ref<Eigen::MatrixXd>& new_matrix);

void RemoveMatrix(Eigen::MatrixXd& original_matrix, int start_index, int cols_num);
#endif // !PARSE_TOOL

