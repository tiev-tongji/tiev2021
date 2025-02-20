//
// Created by xlz on 17-10-20.
//

#ifndef TEST_CARTOGRAPHER_PARAMETERREADER_H
#define TEST_CARTOGRAPHER_PARAMETERREADER_H
#include <fstream>
#include <vector>
#include <map>

using namespace std;
class ParameterReader
{
public:
    ParameterReader( string filename )
    {
        if (filename == "")
        {
            filename = "/parameters.txt";
        }
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            std::cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);

        if (iter == data.end())
        {
            cerr<<data[key]<<endl;
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};


#endif //TEST_CARTOGRAPHER_PARAMETERREADER_H
