#ifndef PARSE_TOOL
#define PARSE_TOOL

#include "configuru.hpp"

#include <Eigen/Eigen>

#include <stdio.h>  /* defines FILENAME_MAX */
#include <unistd.h>
#include <iostream>

#define GetCurrentDir getcwd

std::string GetCurrentWorkingDir( void );

// Parse Matrix in Transpose version
Eigen::MatrixXd MatrixParse(configuru::Config& config_array);
Eigen::MatrixXd MatrixParseT(configuru::Config& config_array);
Eigen::MatrixXd VetorParse(configuru::Config& config_array);

// Write 4dMatrix into Config
void WriteMatrix4d(configuru::Config& config,
                   const Eigen::Ref<Eigen::MatrixXd>& matrix, 
                   std::string name); 

// Here original_matrix is not Ref, for it has to be resized
void ConcatMatrix(Eigen::MatrixXd& original_matrix,
				  const Eigen::Ref<Eigen::MatrixXd>& new_matrix);

void RemoveMatrix(Eigen::MatrixXd& original_matrix, int start_index, int cols_num);

// Result Rendering
void DisplayMatrix(const Eigen::Ref<Eigen::MatrixXd>& matrix, std::string name);
#endif // !PARSE_TOOL

