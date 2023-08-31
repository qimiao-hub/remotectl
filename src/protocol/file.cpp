#include "file.h"
#include <string>
#include <fstream>
#include <iostream>
#include <stdio.h>


using namespace std;
static char *buf_01 = NULL;
static char *buf_02 = NULL;

void Sany_file::file(/* args */)
{
    std::cout << "Sany_file_Init !" << std::endl;
}


//创建第一组txt并写入数据（可能能写完全/可能不能写完全）
void Sany_file::creat_file(char * filename, char * data, int len)
{
    ofstream fout(filename);//example: "1.txt"
    if (fout.is_open())
    {
        for(int i = 0; i < len; i++)
        {
            fout << data[i] ;
        }
    }
    else
    {
        cout << "Error Creat File !" << endl;
    }
    fout.close();
}


//在原来文件的基础上追加所要添加的数据
void Sany_file::add_file(char * filename, char * data, int len)
{
    ofstream fout(filename,ios::app);//example: "1.txt"
    if(fout)
    {
        for(int i = 0; i < len; i++)
        {
            fout << data[i] ;
        }
    }
    else
    {
        cout << "Don't Open the File !" << endl;
    }
    fout.close();
}

int Sany_file::delete_context(char * filename, int iLen)
{
    char tmp_file[20] = { "./tmp.txt" };
    long int cur_add = 0;
    FILE *fSource;
    FILE *tmp;
    tmp = fopen(tmp_file, "w");
    int writeCount = 0; // 每次调用fwrite()写入的字节数
    fSource = fopen(filename, "r+");

    fseek(fSource, 0, SEEK_END);
    cur_add = ftell(fSource);
    int index = cur_add - iLen;
    if(0 == index)
    {
        if(buf_01 == NULL)
        {
            buf_01 = (char *)malloc(WINDATALEN);
        }
        fseek(fSource, 0, SEEK_SET);      //重置fSource 为首地址
        fread(buf_01, 1, cur_add, fSource);  //读取 删除前的数据
        writeCount = fwrite(buf_01, 1, WINDATALEN, tmp); //存入tmp
        printf("saved :%d bytes data\n", writeCount);
        //free(buf_01);
    }
    else
    {
        if(buf_02 == NULL)
        {
            buf_02 = (char *)malloc(WINDATALEN);
        }
        fseek(fSource, 0, SEEK_SET);      //重置fSource 为首地址
        fread(buf_02, 1, cur_add, fSource);  //读取 删除前的数据
        writeCount = fwrite(buf_02, 1, index, tmp); //存入tmp
        printf("saved :%d bytes data\n", writeCount);
        //free(buf_02);
    }
    fclose(fSource);
    fclose(tmp);
    remove(filename);
    rename("./tmp.txt", filename);

    return writeCount;
}

//删除对应txt文件
int Sany_file::delete_file(char * filename)
{
    remove(filename);//example: "1.txt"
    return 0;
}


//清空txt文件内的文本内容
void Sany_file::Empty_file(char * filename)
{
    ofstream fout(filename, ios::out);//example: "1.txt"
    fout.close();
}

#if 0
void Sany_file::read_file(char * filename, string readline)
{
    ifstream in(filename);//example: "1.txt"
    if(!in.is_open())
    {
        cout << "can not open this file !" << endl;
        return 0;
    }
    else
    {
        while(getline(in,readline))
        {
            cout << readline <<endl;
        }
    }
    in.close();
}
#endif

void Sany_file::log(std::string content, int type)
{
    static int m_auto_delete_log = 1;// = 1;
    const char* logfile = "./00downline_file.txt";
    FILE *fp;
    int exist=0;
    exist=access(logfile,F_OK);//判断文件是否存在，不存在返回－1
    //if(exist != -1 && m_auto_delete_log == 1)
    //{
    //  if(0 == remove(logfile))
    //  {
    //     m_auto_delete_log = 0;
    // }
    //   return ;
    //}

    fp=fopen(logfile,"a+");
    if(fp!=NULL)
    {
        if(exist<0)
        {   //文件不存在
            fprintf(fp,"*******%20s%20s%5s%s","错误时间","文件名","行号","错误内容\r\n");
        }
        if(type == 0)
        {
            fprintf(fp,"%s\n",content.c_str());
        }
        else
        {
            time_t occur_time;
            time(&occur_time);
            struct tm ts;
            localtime_r(&(occur_time),&ts);

            char time_c[20];
            sprintf(time_c,"%02d%02d%02d,%02d:%02d:%02d",ts.tm_year+1900-2000,ts.tm_mon+1,ts.tm_mday,ts.tm_hour,ts.tm_min,ts.tm_sec);
            fprintf(fp,"%s: %s\r\n",time_c,content.c_str());
        }
    }
    else
    {
        // printf("error:%s\n",strerror(errno));
    }
    //signal(SIGXFSZ, SIG_DFL);
    fclose(fp);
}




