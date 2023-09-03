FROM ubuntu:20.04 as build

RUN apt-get update
ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN  apt-get install nano git cmake build-essential g++ libeigen3-dev protobuf-compiler libboost-all-dev -y

COPY . /repo

WORKDIR /repo
RUN mkdir build
WORKDIR /repo/build
RUN cmake ..
RUN make

FROM ubuntu:22.04

COPY --from=build /repo/build/imu_filter_cpp_main /entrypoint/imu_filter_cpp_main

STOPSIGNAL SIGINT

ENTRYPOINT ["/entrypoint/imu_filter_cpp_main"]