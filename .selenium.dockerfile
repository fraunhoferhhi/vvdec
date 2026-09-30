# vim: filetype=dockerfile

FROM debian:trixie
# using debian as a base instead of ubuntu, because in ubuntu chromium is a snap-package, which
# does not simply work in docker containers

LABEL maintainer="Gabriel Hege" \
      description="Emscripen and Chromium+Selenium test environment"

ARG DEBIAN_FRONTEND=noninteractive

RUN echo 'APT::Install-Recommends "false";' >> /etc/apt/apt.conf

RUN apt-get update &&     \
    apt-get install -y    \
        bzip2             \
        ca-certificates   \
        ccache            \
        cmake             \
        curl              \
        git               \
        git               \
        ninja-build       \
        python3           \
        xz-utils
ENV CMAKE_GENERATOR=Ninja

RUN apt-get update && apt-get install -y --no-install-recommends chromium-driver

# install selenium from debian package
RUN apt-get update && apt-get install -y --no-install-recommends python3-selenium

#RUN apt-get update && apt-get install -y python3-pip
#RUN pip install --user selenium

ARG EMSDK_VER=latest

WORKDIR /opt
RUN git clone https://github.com/emscripten-core/emsdk.git
ENV PATH=$PATH:/opt/emsdk
RUN emsdk install $EMSDK_VER && \
    emsdk activate $EMSDK_VER


RUN apt-get autoclean &&         \
    apt-get autoremove &&        \
    apt-get clean &&             \
    rm -rf /var/lib/apt/lists/*
