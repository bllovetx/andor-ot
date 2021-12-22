"""This's python api used by OT, USTC base on andorpy(https://github.com/quartiq/andorpy)

All parameters should be configured in AndorConfig.json in the same dir with this file.

Logging should be configured in __main__.py
"""

from __future__ import print_function, unicode_literals, division, annotations

from ctypes import (Structure, c_ulong, c_long, c_int, c_uint, c_float,
                    byref, oledll, create_string_buffer)
import json
import logging
import os
from queue import Queue
# import sys
import threading
import time
from typing import TYPE_CHECKING, Dict, Iterator, Protocol, Tuple, Type, Union
import warnings


_error_codes: Dict[int, str] = {
    20001: "DRV_ERROR_CODES",
    20002: "DRV_SUCCESS",
    20003: "DRV_VXNOTINSTALLED",
    20006: "DRV_ERROR_FILELOAD",
    20007: "DRV_ERROR_VXD_INIT",
    20010: "DRV_ERROR_PAGELOCK",
    20011: "DRV_ERROR_PAGE_UNLOCK",
    20013: "DRV_ERROR_ACK",
    20024: "DRV_NO_NEW_DATA",
    20026: "DRV_SPOOLERROR",
    20034: "DRV_TEMP_OFF",
    20035: "DRV_TEMP_NOT_STABILIZED",
    20036: "DRV_TEMP_STABILIZED",
    20037: "DRV_TEMP_NOT_REACHED",
    20038: "DRV_TEMP_OUT_RANGE",
    20039: "DRV_TEMP_NOT_SUPPORTED",
    20040: "DRV_TEMP_DRIFT",
    20050: "DRV_COF_NOTLOADED",
    20053: "DRV_FLEXERROR",
    20066: "DRV_P1INVALID",
    20067: "DRV_P2INVALID",
    20068: "DRV_P3INVALID",
    20069: "DRV_P4INVALID",
    20070: "DRV_INIERROR",
    20071: "DRV_COERROR",
    20072: "DRV_ACQUIRING",
    20073: "DRV_IDLE",
    20074: "DRV_TEMPCYCLE",
    20075: "DRV_NOT_INITIALIZED",
    20076: "DRV_P5INVALID",
    20077: "DRV_P6INVALID",
    20078: "DRV_INVALID_MODE",
    20083: "P7_INVALID",
    20089: "DRV_USBERROR",
    20091: "DRV_NOT_SUPPORTED",
    20099: "DRV_BINNING_ERROR",
    20990: "DRV_NOCAMERA",
    20991: "DRV_NOT_SUPPORTED",
    20992: "DRV_NOT_AVAILABLE"
}

# From typing issue 182
if TYPE_CHECKING:
    class JSONArray(list[JSON], Protocol):  # type: ignore
        __class__: Type[list[JSON]]  # type: ignore

    class JSONObject(dict[str, JSON], Protocol):  # type: ignore
        __class__: Type[dict[str, JSON]]  # type: ignore

    JSON = Union[None, float, str, JSONArray, JSONObject]


class AndorError(Exception):
    def __str__(self):
        return "%i: %s" % (self.args[0], _error_codes[self.args[0]])

    @classmethod
    def check(cls, status):
        if status != 20002:
            raise cls(status)


class AndorCapabilities(Structure):
    _fields_ = [
        ("size", c_ulong),
        ("acq_modes", c_ulong),
        ("read_modes", c_ulong),
        ("trigger_modes", c_ulong),
        ("camera_type", c_ulong),
        ("pixel_mode", c_ulong),
        ("set_functions", c_ulong),
        ("get_functions", c_ulong),
        ("features", c_ulong),
        ("pci_card", c_ulong),
        ("em_gain_capability", c_ulong),
        ("ft_read_modes", c_ulong),
    ]

_andor_logger = logging.getLogger(__name__)
_dll = None
_this_dir = os.path.dirname(os.path.realpath(__file__))



def load_dll(path: str):
    global _dll
    assert not _dll
    _dll = oledll.LoadLibrary(path)


def get_available_cameras() -> int:
    """get_available_cameras This function returns the total number of Andor cameras currently installed. It is possible 
    to call this function before any of the cameras are initialized.

    :return: the number of cameras currently installed
    :rtype: int
    """    
    n = c_long()
    assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
    AndorError.check(_dll.GetAvailableCameras(byref(n)))
    return n.value


def set_current_camera(c):
    assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
    AndorError.check(_dll.SetCurrentCamera(c))


_current_handle = None


def make_current(c):
    global _current_handle
    if c != _current_handle:
        set_current_camera(c)
        _current_handle = c


class Camera:
    @classmethod
    def by_index(cls, i: int) -> Camera:
        """by_index This function returns a Camera Class instance init by the handle for the camera specified by cameraIndex. When multiple 
        Andor cameras are installed the handle of each camera must be retrieved in order to 
        select a camera using the SetCurrentCamera function. 
        The number of cameras can be obtained using the GetAvailableCameras function.

        :param i: index of any of the installed cameras.
            Valid values    0 to NumberCameras-1 where NumberCameras is the value 
                            returned by the GetAvailableCameras function.
        :type i: int
        :return: Camera instance init by handle of the camera.
        :rtype: Camera
        """        
        h = c_long()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetCameraHandle(i, byref(h)))
        return cls(h.value)

    @classmethod
    def first(cls) -> Camera:
        """first return the Camera instance of first camera, see by_index()

        :return: Camera instance of the first camera
        :rtype: Camera
        """        
        return cls.by_index(0)

    def _make_current(self):
        make_current(self.handle)

    def __init__(
        self, handle: int, 
        with_json: bool = False, 
        using_threading: bool = False, 
        json_config: JSONObject = None
    ):
        self.handle: int = handle
        # config
        self.with_json: bool = with_json
        self.using_threading: bool = using_threading
        # logger
        global _andor_logger
        self.logger: logging.Logger = _andor_logger
        if self.with_json:
            # Load json file if not loaded
            self.json_config: JSONObject = self._load_config_from_json() if (json_config is None) else json_config
            # configurations that used frequently, not use until _config_according_to_json
            self.read_out_mode: str = ""
            self.acquisition_mode: str = ""

        if using_threading:
            ## NOTE: using a boolean flag and lambda can also work with GIL,
            ## but this is not guarranteed campared with threading.Event/Lock

            ## data watcher
            # threading configurations
            self.data_watcher_wait_time: float = self.json_config["dataWatcherWaitTime"] if self.with_json else 0.2 # s
            # internal var
            self._data_watcher: threading.Thread | None = None
            self._data_pipeline: Queue = Queue() # data pushed by data_watcher from camera, get by user
            self._data_watcher_stop: threading.Event = threading.Event() # flag event to stop data_watcher
            self._data_watcher_working: threading.Event = threading.Event() # data_watcher's working state(True: working)
            self._pipeline_lock: threading.Lock = threading.Lock()  # pipeline lock in case user and data_watcher 
                                                                    # access pipeline at same time

            ## temp watcher
            self.temp_watcher_wait_time: float = self.json_config["tempWatcherWaitTime"] if self.with_json else 30
            self._temp_watcher: threading.Thread | None = None
            self._temp_watcher_stop: threading.Event = threading.Event() # flag event to stop temp_wathcer
            self._temp_watcher_working: threading.Event = threading.Event() # temp_wathcer's working status

        if self.with_json:
            # config according to product model
            self._config_according_to_json()
            

    @staticmethod
    def _best_index(values, search):
        dev = None
        for index, value in enumerate(values):
            if dev is None or abs(value - search) < dev:
                best = index
                dev = abs(value - search)
        return best

    # # OT Encapsulation Functions (NOTE: not tested without json)
    @classmethod
    def by_serial_number(cls, using_threading: bool = True) -> Camera:
        """by_serial_number This method is designed to automatically find target camera
        with serial number from json. 'with_json' is a must and by default 'using_threading'
        is enabled.

        :param using_threading: whether to use threading, see __init__ for detail, defaults to True (False not tested currently)
        :type using_threading: bool, optional
        :return: camera specified by serial number if found
        :rtype: Camera
        """  
        global _andor_logger

        # load json to get target serial number and dll/ini file path
        temp_json: JSONObject = cls._load_config_from_json()
        target_serial_number: int = temp_json["serialNumber"] # TODO:(xzqZeng@gmail) check int or str
        dll_path: str = temp_json["dllPath"]  
        ini_path: str = temp_json["iniPath"]

        # load dll
        load_dll(dll_path)
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended

        # get available camera
        available_cameras_number: int = get_available_cameras()
        _andor_logger.info(f"Andor: {available_cameras_number} cameras available now.")

        # traverse to check
        serial_number_list = []
        for camera_index in range(available_cameras_number):
            c_temp_handle = c_long()
            AndorError.check(_dll.GetCameraHandle(camera_index, byref(c_temp_handle)))
            temp_handle = c_temp_handle.value
            temp_camera = Camera(temp_handle)
            temp_camera.initialize(ini_path)
            temp_serial_number = temp_camera.get_camera_serial_number() 
            # if found: return camera, log/check model
            if temp_serial_number == target_serial_number:
                _andor_logger.info("Andor: target camera found, start to configure")
                return Camera(temp_handle, with_json=True, using_threading=using_threading, json_config=temp_json)
            serial_number_list.append(temp_serial_number)
        # not found: assert camera not found and log serial number list
        _andor_logger.info(str(("Andor: current serial number list: ", serial_number_list)))
        assert False, "serial number not found among currently available cameras, see log for serial number list"

    def set_temperature_until_reach(self, temperature: int, timeout: float = 0):
        """set_temperature_until_reach set temperature and wait until reach target or time out

        :param temperature: target temperature, neareat possible temperature will be choosen alternatively
            if out of range 
        :type temperature: int
        :param timeout: time out in seconds, set to 0 will use json setting, if json not used, using default,
            defaults to 0
        :type timeout: float, optional
        """        
        # parse time out
        temp_time_out: float = timeout
        if temp_time_out == 0: # not set in para
            if self.with_json: # get time out from json
                temp_time_out = self.json_config["setTempTimeOut"]
            else: # set to default value and warning
                temp_time_out = 1800
                warnings.warn("set temperature time out is not found, using " + str(temp_time_out))

        # check cooler and target temperature
        assert self.is_cooler_on, "must open cooler before call set_temperature_until_reach"
        min_temp, max_temp = self.get_temperature_range()
        target_temp: int = temperature
        if temperature < min_temp:
            target_temp = min_temp
            warnings.warn("target temperature lower than min one, set to " + str(target_temp) + " instead.")
        elif temperature > max_temp:
            target_temp = max_temp
            warnings.warn("target temperature higher than max one, set to " + str(target_temp) + " instead.")

        # set and wait with time out
        self.set_temperature(target_temp)
        start_time: float = time.time()
        cur_temp, cooling_status = self.get_temperature_f()
        while (
            abs(cur_temp - target_temp) > self.json_config["tempTolerance"] 
            and (time.time() - start_time < temp_time_out)
        ):
            time.sleep(5)
            cur_temp, cooling_status = self.get_temperature_f()
            self.logger.info("Andor: cooling: current temperature: %.2f, current status: %s" % (cur_temp, cooling_status))
        if cooling_status != "DRV_TEMP_STABILIZED": 
            # not stabilized
            warnings.warn(
                "temperature not stabilized! current temperature: %.2f, current status: %s" 
                % (cur_temp, cooling_status)
            )
        if abs(cur_temp - target_temp) > self.json_config["tempTolerance"]: 
            # time out, out of tolerance
            warnings.warn(
                "cooling time out, temperature out of tolerance! "
                "current temperature: %.2f, current status: %s" 
                % (cur_temp, cooling_status)
            )

    def reconfig_json(self):
        """reconfig_json reconfigure after json changed during camera running,
        some config should not be changed see 'unchanged_list'
        """        
        # load new json check para can't change
        temp_json = self._load_config_from_json()
        unchanged_list = [
            "dllPath",
            "iniPath",
            "productModel",
            "serialNumber"
        ]
        para_unchanged: bool = True
        changed_para_name: str = ""
        for para_i in unchanged_list:
            (changed_para_name, para_unchanged) = \
                (changed_para_name, para_unchanged) \
                if (temp_json[para_i] == self.json_config[para_i]) \
                else (para_i, False)
        if not para_unchanged:
            warnings.warn(changed_para_name + " is not supported to reconfig")
            return
        # close threads if used and working
        if self.using_threading:
            if self._data_watcher_working.is_set():
                self.stop_data_watcher()
            if self._temp_watcher_working.is_set():
                self._stop_temp_watcher()
        # config before _config_according_to_json
        self.data_watcher_wait_time: float = self.json_config["dataWatcherWaitTime"] if self.with_json else 0.2 # s
        self.temp_watcher_wait_time: float = self.json_config["tempWatcherWaitTime"] if self.with_json else 30 # s
        # replace json and config according to json
        self.json_config = temp_json
        self._config_according_to_json()

    def stop_camera(self):
        # close threads if used
        if self.using_threading:
            if self._data_watcher_working.is_set():
                self.stop_data_watcher()
            if self._temp_watcher_working.is_set():
                self._stop_temp_watcher()
            
        # TODO:(xzqZeng@gmail.com) set temp (and wait)

        # close hardware
        self.shutdown()



    # ## 'using_threading' methods - data watcher
    def start_acquisition_with_watcher(self):
        """start_acquisition_with_watcher Start acquisition and open a data
        watcher to handle data from camera. Should be called after finish last acquisition
        and stop data watcher either by waitting until acquisition finish or interrupt the
        data watcher with 'Camera.stop_data_watcher'

        To get data from pipeline managed by data watcher, call 'Camera.get_data_from_pipeline'
        # NOTE: circular buffer will be cleared right after calling this
        """        
        # check setings
        if not self.using_threading:
            warnings.warn(
                "This method uses threading, set 'using_threading' to True in order "
                "to enable this method."
            )
            return

        # check states
        ## program states
        if self._data_watcher_working.is_set(): # data watcher is still working
            warnings.warn(
                "data watcher is still working, stopped in order to process "
                "new acquisition.\n"
                "In order to interrupt it, please call Camera.stop_data_watcher()"
            )
            self.stop_data_watcher()
            assert not self._data_watcher_working.is_set(), "failed to stop data watcher"
        if not self._data_pipeline.empty(): # data existing in pipeline
            warnings.warn("pipeline is not empty, should check")
        ## TODO: camera states

        # init (pipeline?), datawatcher
        self._data_watcher_stop.clear()
        # start acquisition
        self.start_acquisition()
        # start data watcher
        self._data_watcher = threading.Thread(target=self._watch_data, daemon=True)
        self._data_watcher.start()

    def get_data_from_pipeline(self) -> str | None:
        """get_data_from_pipeline get data from pipeline managed by data watcher.

        :return: newest data not get by user, None if no data available in pipeline
        :rtype: str | None
        """        
        # check setings
        if not self.using_threading:
            warnings.warn(
                "This method uses threading, set 'using_threading' to True in order "
                "to enable this method."
            )
            return None

        # wait for pipeline to be available, raise assertion fail if time out
        if not self._pipeline_lock.acquire(timeout=3):
            assert False, "user failed to acquire pipeline's lock"
        temp_data: str | None = None if self._data_pipeline.empty() else self._data_pipeline.get_nowait()
        self._pipeline_lock.release()
        return temp_data

    def stop_data_watcher(self):
        # check setings
        if not self.using_threading:
            warnings.warn(
                "This method uses threading, set 'using_threading' to True in order "
                "to enable this method."
            )
            return
        if not self._data_watcher_stop.is_set():
            # set event(stop)
            self._data_watcher_stop.set()
            # join
            self._data_watcher.join()
            # set event(working)
            self._data_watcher_working.clear()
            self.logger.info("Andor: data watcher stopped by enforcement")
        else:
            warnings.warn("data watcher already stoped")

    def data_watcher_is_working(self) -> bool:
        # check setings
        if not self.using_threading:
            warnings.warn(
                "This method uses threading, set 'using_threading' to True in order "
                "to enable this method."
            )
            return False
            
        return self._data_watcher_working.is_set()

    # ## 'using_threading' method - temp_watcher


    # # OT Assist Functions
    @staticmethod
    def _load_config_from_json() -> JSONObject:
        """_load_config_from_json Load json configuration from this_dir/AndorConfig.json

        :return: JSON configuration
        :rtype: JSONObject
        """
        global _this_dir, _andor_logger
        json_file_path = _this_dir + "\\AndorConfig.json"
        assert os.path.exists(json_file_path), ("json config file does not exist, "
            "please configure Andor camera using 'AndorConfig.json' in the same "
            "dir with this file")
        try:
            json_file = open(json_file_path)
        except:
            print("failed to open json config file")
        temp_config = json.load(json_file)
        json_file.close()
        _andor_logger.info("Andor: json file loaded successful.")
        return temp_config

    def _config_according_to_json(self):
        # in common
        ## store configurations that used frequently
        self.read_out_mode = self.json_config["readoutMode"]
        self.acquisition_mode = self.json_config["acquisitionMode"]
        # sdk should be initialized before all dll class member function is called
        self.initialize(self.json_config["iniPath"])
        # config depend on model type
        # TODO:(xzqZeng@gmail.com): add other model
        assert self.json_config["productModel"] == "iXon Ultra 888", \
            "Currently only iXon Ultra 888 is supported to config with json"
        self._iXon_ultra_888_config()


    def _iXon_ultra_888_config(self):
        # check camera
        print("please check: - model type is " + str(self.get_head_model()) + \
            ", serial number is " + str(self.get_camera_serial_number()))
        
        # open cooler and fan
        cooler_previous_state = "on" if self.is_cooler_on() else "off"
        self.logger.info("Andor: cooler is " + cooler_previous_state + " before configuration")
        if not self.json_config["coolerOn"]:
            warnings.warn("cooler not set to on! check your configuration")
        self.cooler(on=self.json_config["coolerOn"])
        self.logger.info("Andor: cooler " + ("on" if self.json_config["coolerOn"] else "off"))
        self.set_fan_mode(self.json_config["fanMode"])
        fan_modes = ["full", "low", "off"]
        self.logger.info("Andor: fan set to " + fan_modes[self.json_config["fanMode"]])
        self.set_temperature_until_reach(
            self.json_config["targetTemperature"]
        )
        ## watch temperature if "watchTemperature"
        if self.json_config["watchTemperature"]:
            # start temp watcher
            self._temp_watcher_stop.clear()
            self._temp_watcher = threading.Thread(target=self._watch_temp, daemon=True)
            self._temp_watcher.start()
        else: 
            # warning
            warnings.warn("temp watcher not used!")

        # set em gain
        em_gain = self.json_config["emGain"]
        em_gain_mode = self.json_config["emGainMode"]
        em_adv = self.json_config["emAdvanced"]
        if em_adv:
            warnings.warn(("Using gains greater than 300 "
                "is not generally recommended for most applications " 
                "as this will reduce the lifespan of the EMCCD")
            )
        else:
            assert em_gain <= 300, "enable em advanced feature to use large em gain"
        self.set_em_advanced(1 if em_adv else 0)
        self.set_em_gain_mode(em_gain_mode)
            

        # set acquisition mode and relative parameters
        if self.acquisition_mode == "Single Scan":
            self.set_acquisition_mode(1)
            self.set_exposure_time(
                self.json_config["acquisitionParameters"]["singleScan"]["exposureTime"]
            )
        ## TODO:(xzqZeng@gmail.com) add support to more acquisition modes

        # set readout mode and relative parameters
        if self.read_out_mode == "Image":
            self.set_read_mode(4)
            set_image_paras = self.json_config["readoutParameters"]["image"]["setImageParameters"]
            self.set_image(
                (set_image_paras["hbin"], set_image_paras["vbin"]),
                (
                    set_image_paras["hstart"], set_image_paras["hend"],
                    set_image_paras["vstart"], set_image_paras["vend"]
                )
            )
        ## TODO:(xzqZeng@gmail.com) add support to more readout modes

        # set shutter mode
        shutter_paras = self.json_config["setShutterParameters"]
        self.set_shutter(
            shutter_paras["typ"], shutter_paras["mode"],
            shutter_paras["closingTime"], shutter_paras["openingTime"]
        )

        # set trigger mode
        if self.json_config["triggerMode"] == "External":
            self.set_trigger_mode(1)
        ## TODO:(xzqZeng@gmail.com) add support to more trigger modes
        
        # set hs & vs speed
        hs_config = self.json_config["setHSSpeedParameters"]
        self.set_hs_speed(hs_config["typ"], hs_config["index"])
        self.set_preamp_gain(hs_config["preAmpGain"])
        hs_speed = self.get_hs_speeds()
        self.logger.info(hs_speed)

        vs_config = self.json_config["setVSSpeedParameters"]
        self.set_vs_amplitude(vs_config["vsAmp"])
        self.set_vs_speed(vs_config["index"])
        vs_speed = self.get_vs_speeds()
        self.logger.info(vs_speed)

        ## TODO:(xzqZeng@gmail.com) add more configuration, log info, check configuration 
        
    def _watch_data(self):
        # init data watcher
        self._data_watcher_working.set()
        # check
        while not self._data_watcher_stop.wait(timeout=self.data_watcher_wait_time):
            # send data from andor to pipeline if available
            image_index_first, image_index_last = self.get_number_new_images()
            image_in_circular_buffer = image_index_last - image_index_first
            if self.acquisition_mode == "Single Scan": # can use dict of function instead
                if image_in_circular_buffer:
                    if not self._pipeline_lock.acquire(timeout=3):
                        assert False, "Data watcher failed to acquire pipeline's lock"
                    self._data_pipeline.put_nowait(self.get_oldest_image16)
                    self._pipeline_lock.release()
                    self._data_watcher_stop.set() # only get one image
                    self.logger.info("Andor: mission of data watch finish, start to stop automaticly")
        # stop watch (stop andor acquisition if needed and log/warn)
        if _error_codes[self.get_status()] == "DRV_ACQUIRING": # still in acquiring
            self.abort_acquisition()
            self.logger.info("Andor: Acquisition aborted")
        self._data_watcher_working.clear()

    def _watch_temp(self):
        # init temp watcher
        self._temp_watcher_working.set()
        # check
        while not self._temp_watcher_stop.wait(timeout=self.temp_watcher_wait_time):
            # check status and temp
            cur_temp, cooling_status = self.get_temperature_f()
            if cooling_status != "DRV_TEMP_STABILIZED": 
                # system not stabilized
                warnings.warn(
                    "system temperature not stabilized! current temperature: %.2f, current status: %s" 
                    % (cur_temp, cooling_status)
                )
            if abs(cur_temp - self.json_config["targetTemperature"]) > self.json_config["tempTolerance"]: 
                # out of tolerance
                warnings.warn(
                    "temperature out of tolerance! "
                    "current temperature: %.2f, current status: %s" 
                    % (cur_temp, cooling_status)
                )
            # log
            self.logger.info(
                "Andor temp watcher: current temperature: %.2f, current status: %s" % (cur_temp, cooling_status)
            )
        # stop watch
        warnings.warn(
            "temp watcher stopping "
            "check if you're neither reconfiguring nor closing camera"
        )
        self._temp_watcher_working.clear()

    def _stop_temp_watcher(self):
        """_stop_temp_watcher stop temperature watcher, this should be dangerous to be 
        accessed by user
        """        
        # set event(stop)
        self._data_watcher_stop.set()
        # join
        self._data_watcher.join()
        # set event(working)
        self._data_watcher_working.clear()


    # # Function loaded from dll

    def initialize(self, ini: str) -> None:     
        """initialize This function will initialize the Andor SDK System. As part of the initialization procedure on 
        some cameras (i.e. Classic, iStar and earlier iXion) the DLL will need access to a
        DETECTOR.INI which contains information relating to the detector head, number pixels, 
        readout speeds etc. If your system has multiple cameras then see the section Controlling 
        multiple cameras

        :param ini: Path to the directory containing the files DETECTOR.INI
        :type ini: str
        """
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.Initialize(ini))

    def shutdown(self) -> None:
        """shutdown This function will close the AndorMCD system down.
        """        
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.ShutDown())

    def get_capabilities(self) -> AndorCapabilities:
        """get_capabilities This function will fill in an AndorCapabilities structure with the capabilities associated with 
        the connected camera. Before passing the address of an AndorCapabilites structure to the 
        function the ulSize member of the structure should be set to the size of the structure. In 
        C++ this can be done with the line:
        caps->ulSize = sizeof(AndorCapabilities);
        Individual capabilities are determined by examining certain bits and combinations of bits in 
        the member variables of the AndorCapabilites structure. The next few pages contain a 
        summary of the capabilities currently returned.

        :return: an AndorCapabilities c-type structure filled with the capabilities associated with the connected camera
        :rtype: AndorCapabilities
        """        
        self._make_current()
        caps = AndorCapabilities()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetCapabilities(byref(caps)))
        return caps

    def get_detector(self) -> Tuple[int, int]:
        """get_detector This function returns the size of the detector in pixels. The horizontal axis is taken to be 
        the axis parallel to the readout register.

        :return: 
            - x_pixels - number of horizontal pixels
            - y_pixels - number of vertical pixels.
        :rtype: Tuple[int, int]
        """        
        self._make_current()
        x, y = c_long(), c_long()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetDetector(byref(x), byref(y)))
        return x.value, y.value

    def get_head_model(self):
        """get_head_model This function will retrieve the type of CCD attached to your system.

        :return: [description]
        :rtype: [type]
        """      
        self._make_current()
        m = create_string_buffer(32)
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetHeadModel(m))
        return m.value

    def get_hardware_version(self):
        """get_hardware_version This function returns the Hardware version information.

        :return: [description]
        :rtype: [type]
        """        
        self._make_current()
        card = c_uint()
        flex10k = c_uint()
        dummy1 = c_uint()
        dummy2 = c_uint()
        firmware = c_uint()
        build = c_uint()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetHardwareVersion(
            byref(card), byref(flex10k), byref(dummy1), byref(dummy2),
            byref(firmware), byref(build)))
        return dict(card=card.value, flex10k=flex10k.value,
                    firmware=firmware.value, build=build.value)

    def get_camera_serial_number(self) -> int:
        """get_camera_serial_number This function will retrieve camera’s serial number

        :return: serial number
        :rtype: int
        """        
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        current_serial_number = c_int()
        AndorError.check(_dll.GetCameraSerialNumber(byref(current_serial_number)))
        return current_serial_number.value
                    

    def set_vs_amplitude(self, amplitude: int):
        """set_vs_amplitude If you choose a high readout speed (a low readout time), then you should also consider 
        increasing the amplitude of the Vertical Clock Voltage. 
        There are five levels of amplitude available for you to choose from:
            - Normal
            - +1
            - +2
            - +3 
            - +4 
        Exercise caution when increasing the amplitude of the vertical clock voltage, since higher 
        clocking voltages may result in increased clock-induced charge (noise) in your signal. In 
        general, only the very highest vertical clocking speeds are likely to benefit from an 
        increased vertical clock voltage amplitude.

        :param amplitude: desired Vertical Clock Voltage Amplitude
            Valid values:
                0 - Normal
                1->4 - Increasing Clock voltage Amplitude
        :type amplitude: int
        """        
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetVSAmplitude(amplitude))

    def get_vs_speeds(self) -> Iterator[float]:
        """get_vs_speeds As your Andor SDK system may be capable of operating at more than one vertical shift 
        speed this function will return the actual speeds available. The value returned is in 
        microseconds.

        :yield: speed in microseconds per pixel shift.
        :rtype: Iterator[float]
        """        
        self._make_current()
        n = c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetNumberVSSpeeds(byref(n)))
        speed = c_float()
        for i in range(n.value):
            AndorError.check(_dll.GetVSSpeed(i, byref(speed)))
            yield speed.value

    def set_vs_speed(self, speed):
        if isinstance(speed, float):
            speeds = self.get_vs_speeds()
            speed = self._best_index(speeds, speed)
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetVSSpeed(speed))

    def get_number_of_adc_channels(self):
        self._make_current()
        n = c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetNumberADChannels(byref(n)))
        return n.value

    def get_adc_channel(self):
        self._make_current()
        c = c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetADCChannel(byref(c)))
        return c.value

    def set_adc_channel(self, channel):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetADChannel(channel))
        AndorError.check(_dll.SetOutputAmplifier(channel))

    def get_bit_depth(self, channel=None):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        if channel is None:
            return [self.get_bit_depth(c)
                    for c in range(self.get_number_of_adc_channels())]
        else:
            b = c_int()
            AndorError.check(_dll.GetBitDepth(channel, byref(b)))
            return b.value

    def set_fan_mode(self, mode):
        """0 = full, 1 = low, 2 = off"""
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetFanMode(mode))

    def cooler(self, on=True):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        if on:
            AndorError.check(_dll.CoolerON())
        else:
            AndorError.check(_dll.CoolerOFF())

    def is_cooler_on(self) -> bool:
        """is_cooler_on This function checks the status of the cooler.

        :return: iCoolerStatus
            - 0: cooler is off
            - 1: cooler is on
        :rtype: bool
        """        
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        cooler_status = c_int()
        AndorError.check(_dll.IsCoolerOn(byref(cooler_status)))
        return bool(cooler_status.value)

    def get_temperature_range(self):
        self._make_current()
        min, max = c_int(), c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetTemperatureRange(byref(min), byref(max)))
        return min.value, max.value

    def get_temperature(self) -> Tuple[int, str]:
        """get_temperature This function returns the temperature of the detector to the nearest degree. It also gives 
        the status of cooling process.

        :return: 
            - temperature(int):  temperature of the detector
            - status(str): status of cooling process
                DRV_NOT_INITIALIZED     System not initialized.
                DRV_ACQUIRING           Acquisition in progress.
                DRV_ERROR_ACK           Unable to communicate with card.
                DRV_TEMP_OFF            Temperature is OFF.
                DRV_TEMP_STABILIZED     Temperature has stabilized at set point.
                DRV_TEMP_NOT_REACHED    Temperature has not reached set point.
                DRV_TEMP_DRIFT          Temperature had stabilized but has since drifted
                DRV_TEMP_NOT_STABILIZED Temperature reached but not stabilized

        :rtype: Tuple[int, str]
        """        
        self._make_current()
        t = c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        status = _dll.GetTemperature(byref(t))
        return t.value, _error_codes[status]

    def get_temperature_f(self) -> Tuple[float, str]:
        """get_temperature_f This function returns the temperature in degrees of the detector. It also gives the status of 
        cooling process.

        :return: 
            - temperature(float):  temperature of the detector
            - status(str): status of cooling process
                DRV_NOT_INITIALIZED     System not initialized.
                DRV_ACQUIRING           Acquisition in progress.
                DRV_ERROR_ACK           Unable to communicate with card.
                DRV_TEMP_OFF            Temperature is OFF.
                DRV_TEMP_STABILIZED     Temperature has stabilized at set point.
                DRV_TEMP_NOT_REACHED    Temperature has not reached set point.
                DRV_TEMP_DRIFT          Temperature had stabilized but has since drifted
                DRV_TEMP_NOT_STABILIZED Temperature reached but not stabilized

        :rtype: Tuple[float, str]
        """        
        self._make_current()
        t = c_float()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        status = _dll.GetTemperatureF(byref(t))
        return t.value, _error_codes[status]

    def set_temperature(self, t):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetTemperature(int(t)))

    def get_preamp_gains(self):
        self._make_current()
        n = c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetNumberPreAmpGains(byref(n)))
        gain = c_float()
        for i in range(n.value):
            AndorError.check(_dll.GetPreAmpGain(i, byref(gain)))
            yield gain.value

    def set_preamp_gain(self, gain):
        if isinstance(gain, float):
            gains = self.get_preamp_gains()
            gain = self._best_index(gains, gain)
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetPreAmpGain(gain))

    def get_em_gain_range(self):
        self._make_current()
        low, high = c_int(), c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetEMGainRange(byref(low), byref(high)))
        return low.value, high.value

    def set_em_advanced(self, state: int):
        """set_em_advanced This function turns on and off access to higher EM gain levels within the SDK. Typically, 
        optimal signal to noise ratio and dynamic range is achieved between x1 to x300 EM Gain. 
        Higher gains of > x300 are recommended for single photon counting only. Before using 
        higher levels, you should ensure that light levels do not exceed the regime of tens of 
        photons per pixel, otherwise accelerated ageing of the sensor can occur.

        :param state: int state: Enables/Disables access to higher EM gain levels
            - 1 – Enable access
            - 0 – Disable access
        :type state: int
        """       
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        self._make_current()
        AndorError.check(_dll.SetEMAdvanced(state))

    def set_em_gain_mode(self, mode):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetEMGainMode(mode))

    def set_emccd_gain(self, gain):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetEMCCDGain(gain))

    def get_status(self):
        self._make_current()
        state = c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetStatus(byref(state)))
        return state.value

    def start_acquisition(self):
        """
        NOTE: circular buffer will be cleared by hardware right after this
        """
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.StartAcquisition())

    def abort_acquisition(self):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.AbortAcquisition())

    def set_shutter(self, a, b, c, d):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetShutter(a, b, c, d))

    def shutter(self, open=True):
        self.set_shutter(0, 1 if open else 2, 0, 0)

    def get_acquisition_timings(self) -> Tuple[float, float, float]:
        """get_acquisition_timings This function will return the current “valid” acquisition timing information. This function 
        should be used after all the acquisitions settings have been set, e.g. SetExposureTime, 
        SetKineticCycleTime and SetReadMode etc. The values returned are the actual times 
        used in subsequent acquisitions. 

        This function is required as it is possible to set the exposure time to 20ms, accumulate 
        cycle time to 30ms and then set the readout mode to full image. As it can take 250ms to 
        read out an image it is not possible to have a cycle time of 30ms.

        :return: 
            - exposure: valid exposure time in seconds
            - accumulate: valid accumulate cycle time in seconds
            - kinetic: valid kinetic cycle time in seconds
        :rtype: Tuple[float, float, float]
        """        
        self._make_current()
        exposure = c_float()
        accumulate = c_float()
        kinetic = c_float()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetAcquisitionTimings(
            byref(exposure), byref(accumulate), byref(kinetic)))
        return exposure.value, accumulate.value, kinetic.value

    def set_acquisition_mode(self, frames=0):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        if frames == 1:
            AndorError.check(_dll.SetAcquisitionMode(1))
        elif frames > 1:
            AndorError.check(_dll.SetAcquisitionMode(3))
            AndorError.check(_dll.SetNumberAccumulations(1))
            AndorError.check(_dll.SetAccumulationCycleTime(0))
            AndorError.check(_dll.SetNumberKinetics(frames))
        elif frames == 0:
            AndorError.check(_dll.SetAcquisitionMode(5))

    def set_baseline_clamp(self, active=True):
        """set_baseline_clamp This function turns on and off the baseline clamp functionality. With this feature enabled 
        the baseline level of each scan in a kinetic series will be more consistent across the 
        sequence.

        :param active: Enables/Disables Baseline clamp functionality, defaults to True
        :type active: bool, optional
        """        
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetBaselineClamp(int(active)))

    def set_exposure_time(self, time):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetExposureTime(c_float(time)))

    def set_frame_transfer_mode(self, mode):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetFrameTransferMode(mode))

    def get_hs_speeds(self, channel=None):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        if channel is None:
            for channel in range(self.get_number_of_adc_channels()):
                for _ in self.get_hs_speeds(channel):
                    yield channel, _
        else:
            n = c_int()
            AndorError.check(_dll.GetNumberHSSpeeds(channel, 0, byref(n)))
            speed = c_float()
            for i in range(n.value):
                AndorError.check(_dll.GetHSSpeed(channel, 0, i, byref(speed)))
                yield speed.value

    def set_hs_speed(self, speed):
        if isinstance(speed, float):
            speeds = self.get_hs_speeds(self.get_adc_channel())
            speed = self._best_index(speeds, speed)
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetHSSpeed(0, speed))

    def set_kinetic_cycle_time(self, time):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetKineticCycleTime(c_float(time)))

    def set_read_mode(self, mode):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetReadMode(mode))

    def set_image(self, bin, roi):
        """
        bin: (bin_x, bin_y)
        roi: (x1, x2, y1, y2)
        """
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetImage(bin[0], bin[1],
                                       roi[0], roi[1], roi[2], roi[3]))
        nx = (roi[1] - roi[0] + 1)//bin[0]
        ny = (roi[3] - roi[2] + 1)//bin[1]
        self.shape = nx, ny
        self.size = nx * ny

    def set_trigger_mode(self, mode):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetTriggerMode(mode))

    def set_fast_ext_trigger(self, mode):
        self._make_current()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.SetFastExtTrigger(mode))

    def get_number_new_images(self) -> Tuple[int, int]:
        """get_number_new_images This function will return information on the number of new images (i.e. images which have 
        not yet been retrieved) in the circular buffer. This information can be used with 
        GetImages to retrieve a series of the latest images. If any images are overwritten in the 
        circular buffer they can no longer be retrieved and the information returned will treat 
        overwritten images as having been retrieved.

        :return: 
            - first: returns the index of the first available image in the circular buffer.
            - last: returns the index of the last available image in the circular buffer.
            NOTE: - as image accumulate in circular buffer without reading:
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
        :rtype: Tuple[int, int]
        """
        self._make_current()
        first, last = c_long(), c_long()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetNumberNewImages(byref(first), byref(last)))
        return first.value, last.value

    def get_oldest_image16(self) -> str:
        """get_oldest_image16 16-bit version of the GetOldestImage function.

        :return: 16-bit image data in str format
        :rtype: str
        """        
        self._make_current()
        buffer: str = create_string_buffer(2*self.size)
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetOldestImage16(buffer, self.size))
        return buffer

    def get_images16(self, first_last=None):
        self._make_current()
        if first_last is None:
            try:
                first, last = self.get_number_new_images()
            except AndorError:
                return
        else:
            first, last = first_last
        n = last - first + 1
        if n == 0:
            return
        buf = create_string_buffer(2*n*self.size)
        valid_first, valid_last = c_long(), c_long()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetImages16(
            first, last, buf, n*self.size,
            byref(valid_first), byref(valid_last)))
        for i in range(n):
            yield buf[2*self.size*i:2*self.size*(i + 1)]
