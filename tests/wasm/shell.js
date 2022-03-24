'use strict';

let decoderWorker;
const bitstreamList = document.getElementById("selectBitstream");
let doDecoderRunAll = false;
let runningIndex;

let testSuiteStartTime, testSuiteEndTime;
let countTestCases;
const failedTestCases = [];

const buttonRun = document.getElementById("buttonRun");
buttonRun.onclick = () => runDecoder(false);

window.onload = async function () {
  populateBitstreamList();
  decoderWorker = new Worker("decoder_worker.js");
  decoderWorker.onmessage = handleWorkerMessage;
  decoderWorker.postMessage({
    'cmd': 'init',
    'appPath': await findAppPath()
  });
};


function handleWorkerMessage(e) {
  switch (e.data.cmd) {
    case "out":
    case "err":
      print(e.data.text);
      break;

    case "initDone":
      buttonRun.disabled = false;
      // updateUIButtons();
      if (location.hash === "#autorun") {
        runDecoder(false);
      }

      break;

    case "decoderDone":
      if (e.data.exitStatus === 0) {
        print(`${bitstreamList[runningIndex].value} duration: ${e.data.duration.toPrecision(4)}`);
      }
      else {
        for (let line of e.data.output) {
          print(line);
        }
        printErr(`${bitstreamList[runningIndex].value} ==> ERROR: Return Code: ${e.data.exitStatus}`);
        failedTestCases.push(bitstreamList[runningIndex].value);
      }

      if (document.getElementById("checkRunAll").checked) {
        runDecoder(true);
      }
      else {
        testsDone();
      }
      break;

    case "exception":
      for (let line of e.data.output) {
        print(line);
      }
      printErr(`${bitstreamList[runningIndex].value} ==> ERROR: Exception`);
      break;
  }
}

const output = document.getElementById('output');
output.value = ''; // clear browser cache
function print(text) {
  console.log(text);

  if (!output) {
    return;
  }

  output.value += text + "\n";
  output.scrollTop = output.scrollHeight; // focus on bottom
}
function clearOutput() {
  output.value = '';
}

function printErr(text) {
  console.error(text);

  if (!output) {
    return;
  }

  output.value += text + "\n";
  output.scrollTop = output.scrollHeight; // focus on bottom
}


async function populateBitstreamList() {
  const response = await fetch("bitstreams.json");
  const bitstreams = await response.json();
  for (let b of bitstreams) {
    const opt = document.createElement("option");
    if (b instanceof Array) {
      opt.value = b[0].toString();
      opt.text = b[1].toString();;
    }
    else {
      opt.text = b.toString();

      // // select default:
      // if (opt.text === set_default || opt.value === set_default) { opt.selected = true; }
      bitstreamList.add(opt);
    }
  }
}

async function findAppPath() {
  const tryPaths = ['/bin/release-static/', document.location.href.match(/.*\//)[0] + 'bin/', '/install/bin/'];

  for (let path of tryPaths) {
    let resp = await fetch(path + 'vvdecapp.js', { method: 'HEAD' });
    if (resp.ok)
      return path;
  }
  return Promise.reject();
}

function runDecoder(runNext) {
  buttonRun.disabled = true;
  if (!testSuiteStartTime) {
    testSuiteStartTime = performance.now();
    countTestCases = 0;
  }

  ++countTestCases;

  if (runNext) {
    bitstreamList.selectedIndex = runningIndex + 1;
    if (bitstreamList.selectedIndex === -1) {
      bitstreamList.selectedIndex = 0;
      testsDone();
      return;
    }
  }
  // skip list separators
  while (bitstreamList.value.startsWith("---") || bitstreamList.value.startsWith("===")) {
    ++bitstreamList.selectedIndex;
  }
  runningIndex = bitstreamList.selectedIndex;

  decoderWorker.postMessage({
    cmd: 'callMain',
    bitstream: '/ext/bitstreams/' + bitstreamList.value,
    threads: document.getElementById("numThreads").value,
    logLevel: document.getElementById("logLevel").value
    // repeat: document.getElementById("repeat").value,
  });
}

function testsDone() {
  buttonRun.disabled = false;

  testSuiteEndTime = performance.now();

  print("Done.\n");
  print(`${countTestCases} test cases.\nTotal duration: ${(testSuiteEndTime - testSuiteStartTime) / 1000}s`);

  if (failedTestCases.length) {
    let summary = "FAILED TEST CASES:\n\n";
    for (let t of failedTestCases) {
      summary += "    " + t + "\n";
    }
    printErr(summary);
  }
};
