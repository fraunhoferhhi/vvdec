#!/usr/bin/env python3

from selenium import webdriver
from selenium.webdriver.chrome.options import Options as ChromeOptions
from selenium.webdriver.chrome.service import Service as ChromeService

import time
import logging
import atexit

OUTPUT_TIMEOUT = 120

opts = ChromeOptions()
opts.capabilities['goog:loggingPrefs'] = { 'browser':'ALL' }
opts.add_argument('--disable-gpu')
opts.add_argument('--no-sandbox')
opts.add_argument('--headless')

opts.binary_location = "/usr/bin/chromium"
chromeservice = ChromeService(executable_path="/usr/bin/chromedriver")

driver = webdriver.Chrome(options=opts, service=chromeservice)
atexit.register(driver.quit)

driver.get('http://localhost:8000/tests/wasm/shell.html#autorun')

count_no_output = 0
return_code = 0
done = False
while( not done ):
    log = driver.get_log('browser')
    for e in log:
        msg = e['message']
        # print(e)
        print(msg)
        if msg.find("Done.") >= 0:
            done = True
        if msg.find("ERROR: Exception") >= 0:   # emscripten exception
            return_code = 2
            done = True
        if msg.find("ERROR:") >= 0:   # failed test case
            return_code = 1

    if len(log):
        count_no_output = 0
    else:
        count_no_output += 1
        if( count_no_output > OUTPUT_TIMEOUT ):
            print("Output timeout")
            return_code = 3
            done = True

    time.sleep(1)


exit(return_code)
