#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <iomanip>
// LIXIAOTENG 2020-12-14 
using namespace std;

#define WINDATALEN   2048*1024

class Sany_file
{
public:
	void file(/* args */);
	void creat_file(char * filename, char * data, int len);
	void add_file(char * filename, char * data, int len);
    int delete_context(char * filename, int iLen);
	int delete_file(char * filename);
	void Empty_file(char * filename);

    void log(std::string content, int type = 1);
	//void read_file(char * filename, string readline);
};



