#include "csvstream.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <unistd.h>
#include <Eigen/Eigen>

using namespace std;

#ifndef DATAPARSE
#define DATAPARSE

bool MeasureParse(csvstream& csvin, int& frame, int& label, int& id, 
    Eigen::Matrix4d& transform, double& detection_score, double& render_score, std::string& img_path)
{
	float value;
	map<string, string> row;
	if (csvin >> row)
	{
		frame = stoi(row["frame"]);
		label = stoi(row["label"]);
		id = stoi(row["id"]);
        img_path = row["image_path"];

		stringstream ssin(row["transform"]);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				ssin >> value;
				transform(i, j) = value;
			}
		}

		// score & bbox:
		if (label != -1) {
			stringstream sd1in(row["detection_score"]);
			stringstream sd2in(row["render_score"]);

			sd1in >> detection_score;
			sd2in >> render_score;
        }
		else {
			detection_score = 0.0;
			render_score = 0.0;
		}
	}
	else
	{
		return false;
	}
	return true;
};

void WriteResult(ofstream& output_file, unsigned int frame, int label, int id, Eigen::Matrix4d transform,
    std::string img_path)
{
    output_file << frame << "," << label << "," << id << ",";
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            output_file << transform(i, j) << " ";
        }
    }
    output_file << "," << img_path;
    output_file << std::endl;
};

#endif
