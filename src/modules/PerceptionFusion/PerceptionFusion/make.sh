cd ../Cartographer
mkdir -p build
cd build
cmake ..
make -j8
cd ../../PerceptionFusion
mkdir -p build
cd build
cmake ..
make -j4

