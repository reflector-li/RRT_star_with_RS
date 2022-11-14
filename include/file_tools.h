//
// Created by reflector on 2021/7/26.
//

#ifndef _FILE_TOOLS_H_
#define _FILE_TOOLS_H_

#include <sys/types.h>
#include <dirent.h>
#include <string.h>  
#include <string>
#include <vector>
#include<iostream>
#include<fstream>
#include<sstream>


void ReadCsv(const std::string &file_name,std::vector<double> &data);
void GetFileNames(std::string path,std::vector<std::string>& filenames);
void stringToken(const std::string sToBeToken, const std::string sSeperator, std::vector<std::string>& vToken);


#endif 