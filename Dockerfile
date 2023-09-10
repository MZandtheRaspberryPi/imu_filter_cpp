FROM mzandtheraspberrypi/imu_websocket_broadcaster:build-2023-09-10 as build

RUN apt-get update
ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN  apt-get install nano git cmake build-essential g++ libeigen3-dev libboost-all-dev -y

COPY . /repo

WORKDIR /repo
RUN mkdir build
WORKDIR /repo/build
RUN cmake -Dabsl_DIR=/abseil/CMakeProject/install/lib/cmake/absl ..
RUN make

FROM ubuntu:22.04

COPY --from=build /repo/build/imu_filter_cpp_main /entrypoint/imu_filter_cpp_main
COPY --from=build /repo/build/imu_websockets/libimu_websockets_lib.so /usr/local/lib/
COPY --from=build /usr/local/lib/libprotobuf.so.24.3.0 /usr/local/lib/

ENV LD_LIBRARY_PATH=/usr/local/lib

STOPSIGNAL SIGINT

ENTRYPOINT ["/entrypoint/imu_filter_cpp_main"]
