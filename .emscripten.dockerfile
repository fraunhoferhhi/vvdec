# vim: filetype=dockerfile

FROM vigitlab.fe.hhi.de:5050/pub/dockerimages/ubuntu_2004_dev:latest

LABEL maintainer="Gabriel Hege" \
      description="Emscripten build environment based on HHI Ubuntu image"

WORKDIR /opt
RUN git clone https://github.com/emscripten-core/emsdk.git
ENV PATH=$PATH:/opt/emsdk
RUN emsdk install latest && \
    emsdk activate latest
