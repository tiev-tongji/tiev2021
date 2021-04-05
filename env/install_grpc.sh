#!/bin/bash
# Fail on first error
set -e

apt-get install -y build-essential cmake autoconf libtool pkg-config libssl-dev 

mkdir -p /tmp/grpc && cd /tmp/grpc

git clone -b v1.35.0 https://gitee.com/mirrors/grpc.git
cd grpc
sed -i s#github.com#gitclone.com/github.com#g .gitmodules
git submodule sync
git submodule update --init
# 来自https://gitee.com/mirrors/grpc/blob/master/test/distrib/cpp/run_distrib_test_cmake.sh
# absl
cd third_party
export GRPC_THIRD_PARTY=$(pwd)
mkdir -p ${GRPC_THIRD_PARTY}/abseil-cpp/cmake/build
cd ${GRPC_THIRD_PARTY}/abseil-cpp/cmake/build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE ../..
make -j4 install
# c-ares
mkdir -p ${GRPC_THIRD_PARTY}/cares/cares/cmake/build
cd ${GRPC_THIRD_PARTY}/cares/cares/cmake/build
cmake -DCMAKE_BUILD_TYPE=Release ../..
make -j4 install
# protobuf
mkdir -p ${GRPC_THIRD_PARTY}/protobuf/cmake/build
cd ${GRPC_THIRD_PARTY}/protobuf/cmake/build
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release ..
make -j4 install
# re2
mkdir -p ${GRPC_THIRD_PARTY}/re2/cmake/build
cd ${GRPC_THIRD_PARTY}/re2/cmake/build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE ../..
make -j4 install
# zlib
mkdir -p ${GRPC_THIRD_PARTY}/zlib/cmake/build
cd ${GRPC_THIRD_PARTY}/zlib/cmake/build
cmake -DCMAKE_BUILD_TYPE=Release ../..
make -j4 install
# gRPC
mkdir -p ${GRPC_THIRD_PARTY}/../cmake/build
cd ${GRPC_THIRD_PARTY}/../cmake/build
cmake ../.. \
-DCMAKE_BUILD_TYPE=Release  \
-DgRPC_INSTALL=ON   \
-DgRPC_BUILD_TESTS=OFF  \
-DgRPC_CARES_PROVIDER=package   \
-DgRPC_ABSL_PROVIDER=package    \
-DgRPC_PROTOBUF_PROVIDER=package    \
-DgRPC_RE2_PROVIDER=package \
-DgRPC_SSL_PROVIDER=package \
-DgRPC_ZLIB_PROVIDER=package
make -j4 install

#clean
cd /tmp && rm -rf /tmp/grpc