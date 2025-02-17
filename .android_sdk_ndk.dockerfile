FROM vigitlab.fe.hhi.de:5050/pub/dockerimages/ubuntu_2404_dev

LABEL maintainer="Gabriel Hege"                          \
      description="Android SDK & NDK build environment"

ARG v_sdk=11076708_latest
ARG v_sdk_platform=35
ARG v_sdk_build_tools=35.0.0
ARG v_ndk=r27c
ARG v_ndk_n=27.2.12479018

ENV ANDROID_HOME=/opt/android-sdk-linux
ENV ANDROID_NDK_ROOT=/opt/android-ndk-${v_ndk}
ENV PATH="${PATH}:${ANDROID_HOME}/cmdline-tools/bin"

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y                                                                                                                                                                                                         &&                                                                               \
    apt-get dist-upgrade -y &&                 \
    apt-get install -y --no-install-recommends \
                    default-jdk-headless       \
                    less

RUN mkdir -p "${ANDROID_HOME}"

#ADD commandlinetools-linux-${v_sdk}.zip /tmp/commandlinetools-linux-${v_sdk}.zip
#ADD android-ndk-${v_ndk}-linux.zip      /tmp/android-ndk-${v_ndk}-linux.zip
RUN wget https://dl.google.com/android/repository/commandlinetools-linux-${v_sdk}.zip -O /tmp/commandlinetools-linux-${v_sdk}.zip && \
    wget http://dl.google.com/android/repository/android-ndk-${v_ndk}-linux.zip       -O /tmp/android-ndk-${v_ndk}-linux.zip &&      \
    unzip -q "/tmp/commandlinetools-linux-${v_sdk}.zip" -d "${ANDROID_HOME}" && rm "/tmp/commandlinetools-linux-${v_sdk}.zip" &&     \
    unzip -q "/tmp/android-ndk-${v_ndk}-linux.zip"      -d /opt              && rm "/tmp/android-ndk-${v_ndk}-linux.zip"

RUN yes | "${ANDROID_HOME}/cmdline-tools/bin/sdkmanager" --sdk_root="${ANDROID_HOME}" --licenses &&                                  \
    yes | "${ANDROID_HOME}/cmdline-tools/bin/sdkmanager" --sdk_root="${ANDROID_HOME}"                                                \
                                                         "build-tools;${v_sdk_build_tools}"                                          \
                                                         "extras;android;m2repository"                                               \
                                                         "platforms;android-${v_sdk_platform}"

RUN apt-get autoclean &&         \
    apt-get autoremove &&        \
    apt-get clean &&             \
    rm -rf /var/lib/apt/lists/*
