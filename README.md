# Andor driver used by OT

This rep is designed to used with [sipyco](https://github.com/m-labs/sipyco)

# Basic usage

## NDSP

In order to use NDSP, run 'andor_ndsp_server.py' on the server.

There exist multiple ways to access the instance from remote client:

1. use sipyco_rpctool [ip] [port] interactive
2. add following code in device.db
   ```python
      device_db.update({
         "emccd": {
                 "type": "controller",
                 "host": "[ip]",
                 "port": 3249,
                 "command": "python /abs/path/to/aqctl_hello.py -p {port}"
         },
   })
   ```
   and call emccd as a device in experiment, see [example](./examples/artiq-master/repository/example_device_in_EnvExperiment.py)
3. call emccd as a client, see [example](./examples/example_client.py)

> NOTE: do not use ::1 as ip address when use remote control, also check your defender if there is something wrong with remote control.
## Logs

In order to use embedded logging, first call logging.basicConfig as in [andor_ndsp_server](./andor_ndsp_server.py)

## Brief introduction to OT encapsulation

### member functions

1. by_serial_number(cls, using_threading: bool = True) -> Camera:
   > generate a camera instance by searching serial number configured in json
   > NOTE1. The serial number only contain the number after X-(EMCCD for instance)
   > NOTE2. This is called in andor_ndsp_server

2. set_temperature_until_reach(self, temperature: int, timeout: float = 0):
   > NOTE: This will not change json, so calling this function may cause frequently warning from temp_watcher

3. reconfig_json(self):
   > reload json without restart
   > NOTE:
   > "dllPath",
   > "iniPath",
   > "productModel",
   > "serialNumber"
   > can not reconfig, and this list may be out of date, to see the most updated one, search 'unchanged_list' in andor.py

4. stop_camera(self):
   > stop threading and shut down camera
   > NOTE: some camera may need temperature control during this process, which is not realized currently

5. start_acquisition_with_watcher(self):
   > start acquistion with a thread watch data in circular buffer

6. get_data_from_pipeline(self): -> str | None:
   > get data stored in a pipeline by data watcher, return None if empty

7. stop_data_watcher(self):
   > stop data watcher and abort acquisition
   > if data_watcher is not used, please call 'abort_acquisition()' to abort

8. data_watcher_is_working(self) -> bool:
   >check the state of data_watcher, the camera should be in acquisition if data_watcher is working

### member variant
1. _data_pipeline:
   > this is inherit from queue.Queue, so you can call corresponding functions if needed.
   > However this is not recommended since it will also be accessed by other thread, access it without acquiring lock may cause problem in some case.


# Detail

see comment in andor.py

# NOTES(Updating):

1. set_acquisition_mode is not directly imported from sdk, so its usage is quite different from the manual, see its definition before use it. Some parameters of acquisition is not used currently as a result
2. start acquisition will clear circular buffer
3. as image accumulate in circular buffer without reading:
   get_number_new_images returns:
   0, 0(no image) along with andor error code 20024(NO NEW DATA)
   1, 1(one image)
   1, 2(two images)
   1, 3(three images)
   etc.
   - get oldest image result in
   2, 3(two left)
   3, 3(one left)
   3, 3(no image) along with andor error code 20024(NO NEW DATA)
