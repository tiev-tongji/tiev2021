#include <fstream>
#include <iostream>
using namespace std;

namespace TiEV{

class BinaryFileHelper{
public:
    template<typename T> static void Read(fstream& file, T& value){
        file.read((char*)&value, sizeof(T));
    }

    template<typename T> static void Write(fstream& file, const T& value){
        file.write((char*)&value, sizeof(T));
    }
};

}
