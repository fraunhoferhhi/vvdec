# Fraunhofer Versatile Video Decoder (VVdeC)

Versatile Video Coding (VVC) is the most recent international video coding standard, developed by the Joint Video Experts Team (JVET) of the ITU-T Video Coding Experts Group (VCEG) and the ISO/IEC Moving Picture Experts Group (MPEG). VVC is the successor of the High Efficiency Video Coding (HEVC) standard and will be released by ITU-T as H.266 and by ISO/IEC as MPEG-I Part 3 (ISO/IEC 23090-3). The new standard targets a 50% bit-rate reduction over HEVC at the same visual quality. In addition, VVC proves to be truly versatile by including tools for efficient coding of video content in emerging applications, e.g. high dynamic range (HDR), adaptive streaming, computer generated content as well as immersive applications like 360 degree video and augmented reality (AR).

The Fraunhofer Versatile Video Decoder (VVdeC) is a fast VVC x86 software decoder implementation. The decoder supports most standard features available in the Main10 profile, with support for some high-level features still pending.

#  How to build VVdeC?

The software uses CMake to create platform-specific build files. 
A working CMake installation is required for building the software.
Download CMake from http://www.cmake.org/ and install it. The following targets are supported: Windows (Visual Studio), Linux (gcc) and MacOS (clang).

## Building using CMake

Open a command prompt on your system and change into the root directory of this project (location of this README.md file).

Create a build directory in the root directory:

    mkdir build

After that use one of the following cmake commands. Feel free to change the commands to satisfy your needs.

Windows sample for Visual Studio 2017 64 Bit:

    cd build
    cmake .. -G "Visual Studio 15 2017 Win64"

Linux Release Makefile sample:

    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release

Linux Debug Makefile sample:

    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Debug

MacOS-X Xcode sample:

    cd build
    cmake .. -G "Xcode"

Available CMake switches:
* VVDEC_ENABLE_BITSTREAM_DOWNLOAD: enables downloading of conformance bitstreams for testing
* VVDEC_ENABLE_INSTALL: enables creation of the install-target
    
## Building using plain make

The project includes an easy to use make interface which bundles the most important use-cases. 
    
Remarks:
* You still need to install CMake to use the make tool
* For Windows, you can install the make command as a part gnuwin32

Open a command prompt on your system and change into the root directory of this project (location of this README.md file).

To use the default system compiler simply call:

    make all

The project includes a simple test suite based on [JVET conformance bitstreams](https://www.itu.int/wftp3/av-arch/jvet-site/bitstream_exchange/VVC/). To enable it, call the make command with the following argument:

    make enable-bitstream-download=1 ...
    
To generate a solution for the default builder on your system simply call:

    make configure
    
To run the simple conformance test suite (if the bitstreams are downloaded and available) call:

    make test
    
The above call only tests the sequences that are know to work. To run a test over all conformance sequences with supported profile call:

    make test-all

# Contributing

Feel free to contribute. To do so:

* Fork the current-most state of the master branch
* Apply the desired changes
* Create a pull-request to the upstream repository

# License

Please see [LICENSE.txt](./LICENSE.txt) file for the terms of use of the contents of this repository.

For more information, please contact: vvc@hhi.fraunhofer.de

**Copyright (c) 2018-2021 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.**

**All rights reserved.**
