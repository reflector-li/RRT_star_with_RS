#include "file_tools.h"

void ReadCsv(const std::string &file_name,std::vector<double> &data)
{
    data.clear();
    std::ifstream in_file(file_name,std::ios::in);
    if(!in_file.is_open())
    {
        std::cout<<"can not open fine"<<std::endl;
        return;
    }
    std::string line;
    std::string word;
    std::stringstream ss;
    getline(in_file,line);
    ss<<line;
    while(getline(ss,word,','))
    {
       double temp;
       data.push_back(stod(word));
    }
}


//path是文件夹路径，filenames为文件夹下所有文件的文件的路径向量
void GetFileNames(std::string path,std::vector<std::string>& filenames)
{

    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str())))
        return;
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            // filenames.push_back(path + "/" + ptr->d_name);
            filenames.push_back((ptr->d_name));
    }
    closedir(pDir);

}


//将字符串sToBeToken按照sSeperator分割符进行切分，然后各部分存入vToken中
void stringToken(const std::string sToBeToken, 
                  const std::string sSeperator, std::vector<std::string>& vToken)
{
    std::string sCopy = sToBeToken;
    int iPosEnd = 0;
    while (true)
    {
        iPosEnd = sCopy.find(sSeperator);
        if (iPosEnd == -1 )
        {
            vToken.push_back(sCopy);
            break;
        }
        vToken.push_back(sCopy.substr(0, iPosEnd));
        sCopy = sCopy.substr(iPosEnd + 1);
    }
}