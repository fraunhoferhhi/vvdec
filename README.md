# VVdeC

![VVdeC Logo](https://github.com/fraunhoferhhi/vvdec/wiki/img/VVdeC_RGB_small.png)

VVdeC, the Fraunhofer Versatile Video Decoder, is a fast software H.266/VVC decoder implementation supporting **all features** of the **VVC Main10** profile.

## Supported architectures


<table>
  <tr>
    <th colspan="2" align="center" valign="center"><strong>Windows</strong></th>
    <th colspan="2" align="center" valign="center"><strong>Linux</strong></th>
    <th colspan="2" align="center" valign="center"><strong>MacOS X</strong></th>
    <th colspan="2" align="center" valign="center"><strong>Android</strong></th>
    <th colspan="2" align="center" valign="center"><strong>iOS</strong></th>
    <th colspan="2" align="center" valign="center"><strong>Browser (WASM)</strong></th>
  </tr>
  <tr>
    <td>Win32</td>
    <td>:white_check_mark:</td>
    <td>x86</td>
    <td>:white_check_mark:</td>
    <td></td>
    <td></td>
    <td></td>
    <td></td>
    <td></td>
    <td></td>
    <td>Edge</td>
    <td>:white_check_mark:</td>
  </tr>
  <tr>
    <td>x64</td>
    <td>:white_check_mark:</td>
    <td>x86_64</td>
    <td>:white_check_mark:</td>
    <td>x64</td>
    <td>:white_check_mark:</td>
    <td></td>
    <td></td>
    <td></td>
    <td></td>
    <td>Firefox</td>
    <td>:white_check_mark:</td>
  </tr>
  <tr>
    <td>armv7</td>
    <td>:x:</td>
    <td>armv7</td>
    <td>:white_check_mark:</td>
    <td></td>
    <td></td>
    <td>armv7</td>
    <td>:white_check_mark:</td>
    <td>armv7</td>
    <td>:black_square_button:</td>
    <td>Chrome</td>
    <td>:white_check_mark:</td>
  </tr>
  <tr>
    <td>aarch64</td>
    <td>:white_check_mark:</td>
    <td>aarch64</td>
    <td>:white_check_mark:</td>
    <td>arm64</td>
    <td>:white_check_mark:</td>
    <td>aarch64</td>
    <td>:white_check_mark:</td>
    <td>arm64</td>
    <td>:black_square_button:</td>
    <td>Safari</td>
    <td>:x:</td>
  </tr>
</table>

:white_check_mark: tested and works :black_square_button: needs testing (might already work) :x: does not work

Other architectures and platforms might work, see the [Wiki](https://github.com/fraunhoferhhi/vvdec/wiki#supported-architectures).

## Information

See the [Wiki-Page](https://github.com/fraunhoferhhi/vvdec/wiki) for more information:

* [Build information](https://github.com/fraunhoferhhi/vvdec/wiki/Build)
* [Usage documentation](https://github.com/fraunhoferhhi/vvdec/wiki/How-to-use-VVdeC)
* [License](https://github.com/fraunhoferhhi/vvdec/wiki/License)
* [Publications](https://github.com/fraunhoferhhi/vvdec/wiki/Publications)
* [Version history](https://github.com/fraunhoferhhi/vvdec/wiki/Changelog)

## Build

VVdeC uses CMake to describe and manage the build process. A working [CMake](https://cmake.org/) installation is required to build the software. In the following, the basic build steps are described. Please refer to the [Wiki](https://github.com/fraunhoferhhi/vvdec/wiki/Build) for the description of all build options.

### How to build using CMake?

To build using CMake, create a `build` directory and generate the project:

```sh
mkdir build
cd build
cmake .. <build options>
```

To actually build the project, run the following after completing project generation:

```sh
cmake --build .
```

For multi-configuration projects (e.g. Visual Studio or Xcode) specify `--config Release` to build the release configuration.

### How to build using GNU Make?

On top of the CMake build system, convenience Makefile is provided to simplify the build process. To build using GNU Make please run the following:

```sh
make install-release <options>
```

Use the option `enable-bitstream-download=1` in the make command to download the VVC conformance bitstreams for testing.

Other supported build targets include `configure`, `release`, `debug`, `relwithdebinfo`, `test`,  and `clean`. Refer to the Wiki for a full list of supported features.

## Citing

Please use the following citation when referencing VVdeC in literature:

```bibtex
@InProceedings{VVdeC,
  author    = {Wieckowski, Adam and Hege, Gabriel and Bartnik, Christian and Lehmann, Christian and Stoffers, Christian and Bross, Benjamin and Marpe, Detlev},
  booktitle = {Proc. IEEE International Conference on Image Processing (ICIP)},
  date      = {2020},
  title     = {Towards A Live Software Decoder Implementation For The Upcoming Versatile Video Coding (VVC) Codec},
  doi       = {10.1109/ICIP40778.2020.9191199},
  pages     = {3124-3128},
}

```

## Contributing

Feel free to contribute. To do so:

* Fork the current-most state of the master branch
* Apply the desired changes
* For non-trivial contributions, add your name to [AUTHORS.md](./AUTHORS.md)
* Create a pull-request to the upstream repository

## License

Please see [LICENSE.txt](./LICENSE.txt) file for the terms of use of the contents of this repository.

For more information, please contact: vvc@hhi.fraunhofer.de

**Copyright (c) 2018-2024 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.**

**All rights reserved.**

**VVdeC® is a registered trademark of the Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.**
