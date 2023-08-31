FROM ubuntu:20.04

RUN apt-get update
ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN  apt-get install nano git cmake build-essential g++ libeigen3-dev -y

COPY . /repo

WORKDIR /repo
RUN mkdir build
WORKDIR /repo/build
RUN cmake ..
RUN make