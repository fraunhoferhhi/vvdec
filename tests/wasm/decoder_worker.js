'use strict';

let VVdeC;
let test_case_output = [];

function sleep(milliseconds) {
  return new Promise(resolve => setTimeout(resolve, milliseconds));
}
function printOut(text) {
  if (arguments.length > 1) text = Array.prototype.slice.call(arguments).join(' ');
  test_case_output.push(text);
  // console.log(text);
  //  postMessage({ "cmd": "out", "text": text });
}
function printErr(text) {
  if (arguments.length > 1) text = Array.prototype.slice.call(arguments).join(' ');
  test_case_output.push(text);
  console.warn(text);
  //  postMessage({ "cmd": "out", "text": text });
}

function createFileAndPathFS(filename) {
  const dirMatch = filename.match(/.*\//);
  if (dirMatch && !VVdeC.FS.analyzePath(dirMatch[0]).exists) {
    VVdeC.FS.createPath("/", dirMatch[0]);
  }
  if (!VVdeC.FS.analyzePath(filename).exists) {
    // VVdeC.FS.createPreloadedFile('/', bitstream, bitstream, true, false); // this is broken due to the Browser javascript library somehow not being linked
    VVdeC.FS.createLazyFile('/', filename, filename, true, false);
    VVdeC.FS.readFile(filename); // workaround to ensure the file is actually loaded
  }
}

onmessage = async function (e) {
  if (e.data.cmd === "init") {
    let scriptUrl = e.data.appPath + "/vvdecapp.js";
    importScripts(scriptUrl);

    let module_config = {
      print: printOut,
      printErr: printErr,
      locateFile: function (f, p) {
        // console.log("searching: " + f);
        return e.data.appPath + f;
      },
      mainScriptUrlOrBlob: scriptUrl,
    };

    try {
      VVdeC = await CreateVVdeC(module_config);

      console.log("INIT");

      postMessage({ cmd: "initDone" });
    }

    catch (e) {
      printErr(e);
      if (e.toString() === "out of memory") {
        printErr("Is this a 32 bit browser? The VVdeC WASM player needs a browser built for 64 bit.");
      }
    }
    return;
  }

  if (!VVdeC) {
    console.error("VVdeC not initialized");
    return;
  }

  switch (e.data.cmd) {
    case "callMain":
      test_case_output = [];
      const bitstream = e.data.bitstream;
      createFileAndPathFS(bitstream);
      const md5File = bitstream.replace(/.bit$/, ".yuv.md5");
      createFileAndPathFS(md5File);
      let md5 = new TextDecoder("utf-8").decode(VVdeC.FS.readFile(md5File).subarray(0, 32));
      //md5 = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"

      const threads = e.data.threads || 10;
      let args = [
        '-b', bitstream,
        '-t', threads,
        '-v', e.data.logLevel || 4,
        '-md5', md5
      ];
      if (e.data.repeat >= 2) {
        args = args.concat(['-L', e.data.repeat + '']);
      }

      args = args.map(e => e + ''); // ensure all args are strings

      let countLoading = 0;
      const unusedWorkers = VVdeC.PThread.unusedWorkers;
      while (unusedWorkers.length < threads) {
        ++countLoading;
        VVdeC.PThread.allocateUnusedWorker();
        VVdeC.PThread.loadWasmModuleToWorker(unusedWorkers[unusedWorkers.length - 1], () => { --countLoading; });
      }
      while (countLoading !== 0) {
        await sleep(10);
      }

      try {
        const startTime = performance.now();
        const ret = VVdeC.callMain(args);
        const endTime = performance.now();

        // printOut("done.");
        postMessage({
          cmd: "decoderDone",
          duration: (endTime - startTime) / 1000,
          output: test_case_output,
          exitStatus: ret
        });
      }
      catch (e) {
        postMessage({
          cmd: "exception",
          output: test_case_output,
        });
      }

      test_case_output = [];

      break;
  }
};
