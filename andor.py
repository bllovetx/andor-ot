"""This python api used by OT, USTC base on andorpy(https://github.com/quartiq/andorpy)

All parameters should be configured in AndorConfig.json in the same dir with this file.

Logging should be configured in __main__.py
"""

from __future__ import print_function, unicode_literals, division, annotations

from ctypes import (Structure, c_ulong, c_long, c_int, c_uint, c_float,
                    byref, oledll, create_string_buffer)
import json
import logging
import os
# import sys
from typing import TYPE_CHECKING, Dict, Iterator, Protocol, Tuple, Type, Union



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
        self.handle = handle
        self.with_json = with_json
        self.using_threading = using_threading
        global _andor_logger
        self.logger = _andor_logger
        if self.with_json:
            # Load json file if not loaded
            self.json_config = self._load_config_from_json() if (json_config is None) else json_config
            # config according to product model
            assert self.json_config["productModel"] == "iXon Ultra 888", \
                "Currently only iXon Ultra 888 is supported to config with json"
            self._iXon_ultra_888_config()
            

    @staticmethod
    def _best_index(values, search):
        dev = None
        for index, value in enumerate(values):
            if dev is None or abs(value - search) < dev:
                best = index
                dev = abs(value - search)
        return best

    # # OT Encapsulation Functions
    @classmethod
    def by_serial_number(cls, using_threading: bool = True) -> Camera:
        """by_serial_number This method is designed to automatically find target camera
        with serial number from json. 'with_json' is a must and by default 'using_threading'
        is enabled.

        :param using_threading: whether to use threading, see __init__ for detail, defaults to True
        :type using_threading: bool, optional
        :return: camera specified by serial number if found
        :rtype: Camera
        """  
        global _andor_logger  
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        # get available camera
        available_cameras_number = get_available_cameras()
        _andor_logger.info(f"{available_cameras_number} cameras available now.")

        # load json to get target serial number
        temp_json = cls._load_config_from_json()
        target_serial_number = temp_json["serialNumber"]

        # traverse to check
        serial_number_list = []
        for camera_index in range(available_cameras_number):
            c_temp_handle = c_long()
            AndorError.check(_dll.GetCameraHandle(camera_index, byref(c_temp_handle)))
            temp_handle = c_temp_handle.value
            temp_camera = Camera(temp_handle)
            temp_serial_number = temp_camera.get_camera_serial_number() 
            # if found: return camera, log/check model
            if temp_serial_number == target_serial_number:
                _andor_logger.info("target camera found, start to configure")
                return Camera(temp_handle, with_json=True, using_threading=using_threading, json_config=temp_json)
            serial_number_list.append(temp_serial_number)
        # not found: assert camera not found and log serial number list
        _andor_logger.info(str(("current serial number list: ", serial_number_list)))
        assert False, "serial number not found among currently available cameras, see log for serial number list"

    # # OT Assist Functions
    @staticmethod
    def _load_config_from_json() -> JSONObject:
        """_load_config_from_json Load json configuration from this_dir/AndorConfig.json

        :return: JSON configuration
        :rtype: JSON
        """
        global _this_dir 
        json_file_path = _this_dir + "\\AndorConfig.json"
        assert os.path.exists(json_file_path), ("json config file does not exist, "
            "please configure Andor camera using 'AndorConfig.json' in the same "
            "dir with this file")
        try:
            json_file = open(json_file_path)
        except:
            print("json config file failed to load")
        temp_config = json.load(json_file)
        json_file.close()
        return temp_config

    def _iXon_ultra_888_config(self):
        # check camera
        pass

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

    def get_temperature_range(self):
        self._make_current()
        min, max = c_int(), c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetTemperatureRange(byref(min), byref(max)))
        return min.value, max.value

    def get_temperature(self):
        self._make_current()
        t = c_int()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        status = _dll.GetTemperature(byref(t))
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

    def get_acquisition_timings(self):
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

    def get_number_new_images(self):
        self._make_current()
        first, last = c_long(), c_long()
        assert _dll is not None, "_dll not initialized!" # In case of _dll = None, can also use "# type: ignore" but not recommended
        AndorError.check(_dll.GetNumberNewImages(byref(first), byref(last)))
        return first.value, last.value

    def get_oldest_image16(self):
        self._make_current()
        buffer = create_string_buffer(2*self.size)
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
