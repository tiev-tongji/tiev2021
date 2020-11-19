#include <iostream>
#include <vector>
#include <string>
#include <string.h>
#include <cstring>
#include <algorithm>
#include <fstream>
using namespace std;

void SplitString(const string &s, vector<string> &v, const string &c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}

// int main()
// {
//     fstream fp;
//     fp.open("noisy", ios::in);
//     vector<int> a;
//     vector<int> b;
//     char s[100];
//     while (1)
//     {
//         if (fp.eof())
//             break;
//         fp.getline(s, 100);
//         //cout << s << endl;
//         for (int j = 0; j < sizeof(s); j++)
//         {
//             if (s[j] == 'X')
//             {
//                 cout << s[j + 2] << s[j + 3] << s[j + 4] << ' ';
//                 cout << s[j + 8] << s[j + 9] << s[j + 10] << ' ';
//             }
//         }
//     }
// }

int main()
{
    fstream fp;
    fp.open("noisy.txt", ios::in);
    char s[2000];
    vector<int> X;
    vector<int> Y;

    fp.getline(s, 2000);
    string str = s;
    vector<string> ar;
    SplitString(str, ar, " ");

    int len = ar.size();
    for (int i = 0; i < len; i += 2)
    {
        int o = 1;
        int x = strtoul(ar[i].c_str(), NULL, 10);
        int y = strtoul(ar[i + 1].c_str(), NULL, 10);
        if (i == 0)
        {
            X.push_back(x);
            Y.push_back(y);
            continue;
        }
        for (int j = 0; j < X.size(); j++)
        {
            if (X[j] == x && Y[j] == y)
            {
                o = 0;
            }
        }
        if (o)
        {
            X.push_back(x);
            Y.push_back(y);
        }
    }
    cout << X.size() << endl;
    // for (int i = 0; i < X.size(); i++)
    // {
    //     cout << X[i] << ' ' << Y[i] << endl;
    // }
}