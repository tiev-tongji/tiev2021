.PHONY: include/zcmmsg
CC = clang
CXXFLAGS = `pkg-config opencv --cflags` -Iinclude -std=c++11 -Wfatal-errors -fPIC

pybind11_shared_object: src/pybind11_module.cc rotate.o cluster.o polynomial.o include/zcmmsg
	${CC} -shared ${CXXFLAGS}\
		`python3 -m pybind11 --includes` \
		`pkg-config opencv --libs` -lzcm \
		$< *.o -o postproc_lane`python3-config --extension-suffix`

rotate.o: src/rotate.cc
	${CC} -c $< -o $@ ${CXXFLAGS}

cluster.o: src/cluster.cc
	${CC} -c $< -o $@ ${CXXFLAGS}

polynomial.o: src/polynomial.cc
	${CC} -c $< -o $@ ${CXXFLAGS}

include/zcmmsg: include/zcmmsg/structLANES.zcm
	zcm-gen -x --lazy include/zcmmsg/structLANES.zcm --cpp-hpath ./include/zcmmsg

clean:
	rm *.so *.o
