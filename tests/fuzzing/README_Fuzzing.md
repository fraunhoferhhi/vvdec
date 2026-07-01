# VVdeC fuzzing environment

This is a fuzzing environment for the VVdeC video decoder. It is based on clang's libFuzzer (https://llvm.org/docs/LibFuzzer.html).

## Building the fuzzing target

Build the fuzzing target using clang.

```bash
CC=clang CXX=clang++ cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DVVDEC_TOPLEVEL_OUTPUT_DIRS=0 -DVVDEC_FUZZING_BUILD=1 -DVVDEC_USE_ADDRESS_SANITIZER=1
cmake --build build
```

If you get linker errors about missing libclang_rt.*-*.a, you may need to install the clang runtime libraries. On Debian/Ubuntu install the `libclang-rt-dev` package.

## Prepare the seed corpus

You need a seed corpus of (preferably short) but differing bitstreams to use as input for the fuzzer. You can create an initial corpus by using some of the bitstreams from the VVdeC test suite.

For example, you can create a corpus directory and copy the 20 smallest bitstreams from the test suite into it:

```bash
mkdir fuzz_corpus
ls -S ext/bitstreams/*/*.bit | tail -n 20 | xargs -I {} cp {} fuzz_corpus/
```

## Run the fuzzer

Run the fuzzer with the corpus and an output directory for crashes:

This will run the fuzzer with a timeout of 20 seconds per input until it finds a crash. The crashes will be saved in the `fuzz_output` directory.

```bash
mkdir -p fuzz_output
UBSAN_OPTIONS=halt_on_error=1:abort_on_error=1 ./build/bin/vvdec_fuzzer -timeout=20 -artifact_prefix=fuzz_output/ fuzz_corpus/
```

You can pass some additional VVdeC options to the fuzzer using --option=value similar to the ones used by vvdecapp. Currently supported are `--t=, --p=, --simd=, --v=, --eh=`.