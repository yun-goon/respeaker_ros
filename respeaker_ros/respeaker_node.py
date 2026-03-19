#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
# Ported to ROS2

import math
import os
import struct
import sys
import time
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
import wave

import angles
import numpy as np
import pyaudio
import rclpy
import usb.core
import usb.util
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA, Int32, UInt8MultiArray

try:
    from pixel_ring import usb_pixel_ring_v2
except IOError as e:
    print(e)
    raise RuntimeError("Check the device is connected and recognized")


RESPEAKER_RW_PARAMS = [
    'STATNOISEONOFF', 'GAMMA_ETAIL', 'AGCTIME', 'GAMMA_NS_SR', 'AGCDESIREDLEVEL',
    'MIN_NN_SR', 'STATNOISEONOFF_SR', 'MIN_NS_SR', 'GAMMA_ENL', 'NLAEC_MODE',
    'TRANSIENTONOFF', 'RT60ONOFF', 'AGCGAIN', 'NONSTATNOISEONOFF', 'AECFREEZEONOFF',
    'AGCMAXGAIN', 'CNIONOFF', 'NONSTATNOISEONOFF_SR', 'NLATTENONOFF', 'ECHOONOFF',
    'AECNORM', 'AECSILENCELEVEL', 'GAMMA_E', 'FREEZEONOFF', 'AGCONOFF',
    'MIN_NN', 'MIN_NS', 'GAMMA_NS', 'HPFONOFF', 'GAMMAVAD_SR', 'GAMMA_NN', 'GAMMA_NN_SR',
]

PARAMETERS = {
    'AECFREEZEONOFF': (18, 7, 'int', 1, 0, 'rw', 'Adaptive Echo Canceler updates inhibit.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'AECNORM': (18, 19, 'float', 16, 0.25, 'rw', 'Limit on norm of AEC filter coefficients'),
    'AECPATHCHANGE': (18, 25, 'int', 1, 0, 'ro', 'AEC Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
    'RT60': (18, 26, 'float', 0.9, 0.25, 'ro', 'Current RT60 estimate in seconds'),
    'HPFONOFF': (18, 27, 'int', 3, 0, 'rw', 'High-pass Filter on microphone signals.', '0 = OFF', '1 = ON - 70 Hz cut-off', '2 = ON - 125 Hz cut-off', '3 = ON - 180 Hz cut-off'),
    'RT60ONOFF': (18, 28, 'int', 1, 0, 'rw', 'RT60 Estimation for AES. 0 = OFF 1 = ON'),
    'AECSILENCELEVEL': (18, 30, 'float', 1, 1e-09, 'rw', 'Threshold for signal detection in AEC [-inf .. 0] dBov (Default: -80dBov = 10log10(1x10-8))'),
    'AECSILENCEMODE': (18, 31, 'int', 1, 0, 'ro', 'AEC far-end silence detection status. ', '0 = false (signal detected) ', '1 = true (silence detected)'),
    'AGCONOFF': (19, 0, 'int', 1, 0, 'rw', 'Automatic Gain Control. ', '0 = OFF ', '1 = ON'),
    'AGCMAXGAIN': (19, 1, 'float', 1000, 1, 'rw', 'Maximum AGC gain factor. ', '[0 .. 60] dB (default 30dB = 20log10(31.6))'),
    'AGCDESIREDLEVEL': (19, 2, 'float', 0.99, 1e-08, 'rw', 'Target power level of the output signal. ', '[-inf .. 0] dBov (default: -23dBov = 10log10(0.005))'),
    'AGCGAIN': (19, 3, 'float', 1000, 1, 'rw', 'Current AGC gain factor. ', '[0 .. 60] dB (default: 0.0dB = 20log10(1.0))'),
    'AGCTIME': (19, 4, 'float', 1, 0.1, 'rw', 'Ramps-up / down time-constant in seconds.'),
    'CNIONOFF': (19, 5, 'int', 1, 0, 'rw', 'Comfort Noise Insertion.', '0 = OFF', '1 = ON'),
    'FREEZEONOFF': (19, 6, 'int', 1, 0, 'rw', 'Adaptive beamformer updates.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'STATNOISEONOFF': (19, 8, 'int', 1, 0, 'rw', 'Stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NS': (19, 9, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise. min .. max attenuation'),
    'MIN_NS': (19, 10, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
    'NONSTATNOISEONOFF': (19, 11, 'int', 1, 0, 'rw', 'Non-stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NN': (19, 12, 'float', 3, 0, 'rw', 'Over-subtraction factor of non- stationary noise. min .. max attenuation'),
    'MIN_NN': (19, 13, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
    'ECHOONOFF': (19, 14, 'int', 1, 0, 'rw', 'Echo suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_E': (19, 15, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (direct and early components). min .. max attenuation'),
    'GAMMA_ETAIL': (19, 16, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (tail components). min .. max attenuation'),
    'GAMMA_ENL': (19, 17, 'float', 5, 0, 'rw', 'Over-subtraction factor of non-linear echo. min .. max attenuation'),
    'NLATTENONOFF': (19, 18, 'int', 1, 0, 'rw', 'Non-Linear echo attenuation.', '0 = OFF', '1 = ON'),
    'NLAEC_MODE': (19, 20, 'int', 2, 0, 'rw', 'Non-Linear AEC training mode.', '0 = OFF', '1 = ON - phase 1', '2 = ON - phase 2'),
    'SPEECHDETECTED': (19, 22, 'int', 1, 0, 'ro', 'Speech detection status.', '0 = false (no speech detected)', '1 = true (speech detected)'),
    'FSBUPDATED': (19, 23, 'int', 1, 0, 'ro', 'FSB Update Decision.', '0 = false (FSB was not updated)', '1 = true (FSB was updated)'),
    'FSBPATHCHANGE': (19, 24, 'int', 1, 0, 'ro', 'FSB Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
    'TRANSIENTONOFF': (19, 29, 'int', 1, 0, 'rw', 'Transient echo suppression.', '0 = OFF', '1 = ON'),
    'VOICEACTIVITY': (19, 32, 'int', 0, 0, 'ro', 'VAD voice activity status.', '0 = false (no voice activity)', '1 = true (voice activity)'),
    'STATNOISEONOFF_SR': (19, 33, 'int', 1, 0, 'rw', 'Stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
    'NONSTATNOISEONOFF_SR': (19, 34, 'int', 1, 0, 'rw', 'Non-stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
    'GAMMA_NS_SR': (19, 35, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.0)'),
    'GAMMA_NN_SR': (19, 36, 'float', 3, 0, 'rw', 'Over-subtraction factor of non-stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.1)'),
    'MIN_NS_SR': (19, 37, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
    'MIN_NN_SR': (19, 38, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
    'GAMMAVAD_SR': (19, 39, 'float', 1000, 0, 'rw', 'Set the threshold for voice activity detection.', '[-inf .. 60] dB (default: 3.5dB 20log10(1.5))'),
    'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle. Current value. Orientation depends on build configuration.')
}


def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles to quaternion (same as tf.transformations)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield


class RespeakerInterface(object):
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018
    TIMEOUT = 100000

    def __init__(self, logger=None):
        self._logger = logger or rclpy.logging.get_logger('respeaker_interface')
        self.dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)
        if not self.dev:
            raise RuntimeError("Failed to find Respeaker device")
        self._logger.info("Initializing Respeaker device")
        self.dev.reset()
        self.pixel_ring = usb_pixel_ring_v2.PixelRing(self.dev)
        self.set_led_think()
        time.sleep(10)
        self.set_led_trace()
        self._logger.info("Respeaker device initialized (Version: %s)" % self.version)

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
        finally:
            self.dev = None

    def write(self, name, value):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return
        if data[5] == 'ro':
            raise ValueError('{} is read-only'.format(name))
        id_ = data[0]
        if data[2] == 'int':
            payload = struct.pack(b'iii', data[1], int(value), 1)
        else:
            payload = struct.pack(b'ifi', data[1], float(value), 0)
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, id_, payload, self.TIMEOUT)

    def read(self, name):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return None
        id_ = data[0]
        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40
        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id_, 8, self.TIMEOUT)
        response = struct.unpack(b'ii', response.tobytes())
        if data[2] == 'int':
            return response[0]
        return response[0] * (2.0 ** response[1])

    def set_led_think(self):
        self.pixel_ring.set_brightness(10)
        self.pixel_ring.think()

    def set_led_trace(self):
        self.pixel_ring.set_brightness(20)
        self.pixel_ring.trace()

    def set_led_color(self, r, g, b, a):
        self.pixel_ring.set_brightness(int(20 * a))
        self.pixel_ring.set_color(r=int(r * 255), g=int(g * 255), b=int(b * 255))

    def set_vad_threshold(self, db):
        self.write('GAMMAVAD_SR', db)

    def is_voice(self):
        return self.read('VOICEACTIVITY')

    @property
    def direction(self):
        return self.read('DOAANGLE')

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT)[0]

    def close(self):
        usb.util.dispose_resources(self.dev)


class RespeakerAudio(object):
    def __init__(self, on_audio, channels=None, suppress_error=True, logger=None):
        self.on_audio = on_audio
        self._logger = logger or rclpy.logging.get_logger('respeaker_audio')
        with ignore_stderr(enable=suppress_error):
            self.pyaudio = pyaudio.PyAudio()
        self.available_channels = None
        self.channels = channels
        self.device_index = None
        self.rate = 16000
        self.bitwidth = 2
        self.bitdepth = 16

        count = self.pyaudio.get_device_count()
        self._logger.debug("%d audio devices found" % count)
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            name = info["name"]
            if isinstance(name, bytes):
                name = name.decode("utf-8")
            chan = info["maxInputChannels"]
            self._logger.debug(" - %d: %s" % (i, name))
            if "respeaker" in name.lower():
                self.available_channels = chan
                self.device_index = i
                self._logger.info("Found %d: %s (channels: %d)" % (i, name, chan))
                break
        if self.device_index is None:
            self._logger.warn("Failed to find respeaker device by name. Using default input")
            info = self.pyaudio.get_default_input_device_info()
            self.available_channels = info["maxInputChannels"]
            self.device_index = info["index"]

        if self.available_channels != 6:
            self._logger.warn("%d channel is found for respeaker" % self.available_channels)
            self._logger.warn("You may have to update firmware.")
        if self.channels is None:
            self.channels = list(range(self.available_channels))
        else:
            self.channels = list(filter(lambda c: 0 <= c < self.available_channels, self.channels))
        if not self.channels:
            raise RuntimeError('Invalid channels %s. (Available channels are %s)' % (
                self.channels, self.available_channels))
        self._logger.info('Using channels %s' % self.channels)

        self.stream = self.pyaudio.open(
            input=True,
            start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=1024,
            stream_callback=self.stream_callback,
            input_device_index=self.device_index,
        )

    def __del__(self):
        self.stop()
        try:
            if self.stream:
                self.stream.close()
        except Exception:
            pass
        finally:
            self.stream = None
        try:
            self.pyaudio.terminate()
        except Exception:
            pass

    def stream_callback(self, in_data, frame_count, time_info, status):
        data = np.frombuffer(in_data, dtype=np.int16)
        chunk_per_channel = len(data) // self.available_channels
        data = np.reshape(data, (chunk_per_channel, self.available_channels))
        for chan in self.channels:
            chan_data = data[:, chan]
            self.on_audio(chan_data.tobytes(), chan)
        return None, pyaudio.paContinue

    def start(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop(self):
        if self.stream.is_active():
            self.stream.stop_stream()


def _param_type_from_name(name):
    """Return (ros_type, default, min, max) for Respeaker rw params."""
    if name not in PARAMETERS:
        return None
    data = PARAMETERS[name]
    type_, max_, min_, rw_ = data[2], data[3], data[4], data[5]
    if rw_ != 'rw':
        return None
    if type_ == 'int' and max_ == 1 and min_ == 0:
        return ('bool', True, None, None)
    if type_ == 'int':
        return ('integer', 0, min_, max_)
    return ('double', 1.0, min_, max_)


class RespeakerNode(Node):
    def __init__(self):
        super().__init__('respeaker_node')
        self.update_rate = self.declare_parameter('update_rate', 10.0).value
        self.sensor_frame_id = self.declare_parameter('sensor_frame_id', 'respeaker_base').value
        self.doa_xy_offset = self.declare_parameter('doa_xy_offset', 0.0).value
        self.doa_yaw_offset = self.declare_parameter('doa_yaw_offset', 90.0).value
        self.speech_prefetch = self.declare_parameter('speech_prefetch', 0.5).value
        self.speech_continuation = self.declare_parameter('speech_continuation', 0.5).value
        self.speech_max_duration = self.declare_parameter('speech_max_duration', 7.0).value
        self.speech_min_duration = self.declare_parameter('speech_min_duration', 0.1).value
        self.main_channel = self.declare_parameter('main_channel', 0).value
        suppress_pyaudio_error = self.declare_parameter('suppress_pyaudio_error', True).value
        self.save_audio = self.declare_parameter('save_audio', True).value
        self.audio_output_dir = Path(
            self.declare_parameter('audio_output_dir', '/tmp/respeaker_audio').value)
        self._shutting_down = False

        if self.save_audio:
            self.audio_output_dir.mkdir(parents=True, exist_ok=True)

        self.respeaker = RespeakerInterface(self.get_logger())
        self.respeaker_audio = RespeakerAudio(self.on_audio, suppress_error=suppress_pyaudio_error, logger=self.get_logger())

        self.speech_audio_buffer = b''
        self.is_speeching = False
        self.speech_stopped = self.get_clock().now()
        self.prev_is_voice = None
        self.prev_doa = None

        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub_vad = self.create_publisher(Bool, 'is_speeching', qos_latched)
        self.pub_doa_raw = self.create_publisher(Int32, 'sound_direction', qos_latched)
        self.pub_doa = self.create_publisher(PoseStamped, 'sound_localization', qos_latched)
        self.pub_audio = self.create_publisher(UInt8MultiArray, 'audio', 10)
        self.pub_speech_audio = self.create_publisher(UInt8MultiArray, 'speech_audio', 10)
        self.pub_audios = {
            c: self.create_publisher(UInt8MultiArray, 'audio/channel%d' % c, 10)
            for c in self.respeaker_audio.channels
        }

        for name in RESPEAKER_RW_PARAMS:
            info = _param_type_from_name(name)
            if not info:
                continue
            ros_type, default, _min_val, _max_val = info
            self.declare_parameter(name, default)
            try:
                val = self.respeaker.read(name)
                if val is not None:
                    if ros_type == 'bool':
                        self.set_parameters([Parameter(name, Parameter.Type.BOOL, bool(val))])
                    elif ros_type == 'integer':
                        self.set_parameters([Parameter(name, Parameter.Type.INTEGER, int(val))])
                    else:
                        self.set_parameters([Parameter(name, Parameter.Type.DOUBLE, float(val))])
            except Exception:
                pass
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self.speech_prefetch_bytes = int(
            self.speech_prefetch * self.respeaker_audio.rate * self.respeaker_audio.bitdepth / 8.0)
        self.speech_prefetch_buffer = b''
        self.respeaker_audio.start()
        self.info_timer = self.create_timer(1.0 / self.update_rate, self._on_timer)
        self.timer_led = None
        self.sub_led = self.create_subscription(ColorRGBA, 'status_led', self._on_status_led, 10)

    def _on_parameters_changed(self, params):
        for p in params:
            if p.name in RESPEAKER_RW_PARAMS:
                try:
                    if p.type_ == Parameter.Type.BOOL:
                        self.respeaker.write(p.name, 1 if p.value else 0)
                    else:
                        self.respeaker.write(p.name, p.value)
                except Exception as e:
                    self.get_logger().error('Failed to set %s: %s' % (p.name, e))
        return rclpy.node.SetParametersResult(successful=True)

    def _on_status_led(self, msg):
        self.respeaker.set_led_color(r=msg.r, g=msg.g, b=msg.b, a=msg.a)
        if self.timer_led is not None:
            try:
                self.timer_led.cancel()
            except Exception:
                pass
            self.timer_led = None
        self.timer_led = self.create_timer(3.0, self._reset_led_trace)

    def _reset_led_trace(self):
        if self.timer_led is not None:
            try:
                self.timer_led.cancel()
            except Exception:
                pass
            self.timer_led = None
        if self.respeaker is not None:
            self.respeaker.set_led_trace()

    def on_audio(self, data, channel):
        if self._shutting_down:
            return
        try:
            self.pub_audios[channel].publish(UInt8MultiArray(data=list(data)))
            if channel == self.main_channel:
                self.pub_audio.publish(UInt8MultiArray(data=list(data)))
                if self.is_speeching:
                    if len(self.speech_audio_buffer) == 0:
                        self.speech_audio_buffer = self.speech_prefetch_buffer
                    self.speech_audio_buffer += data
                else:
                    self.speech_prefetch_buffer += data
                    self.speech_prefetch_buffer = self.speech_prefetch_buffer[-self.speech_prefetch_bytes:]
        except Exception:
            self._shutting_down = True

    def _on_timer(self):
        stamp = self.get_clock().now()
        is_voice = self.respeaker.is_voice()
        doa_rad = math.radians(self.respeaker.direction - 180.0)
        doa_rad = angles.shortest_angular_distance(doa_rad, math.radians(self.doa_yaw_offset))
        doa = math.degrees(doa_rad)

        if is_voice != self.prev_is_voice:
            self.pub_vad.publish(Bool(data=bool(is_voice)))
            self.prev_is_voice = is_voice

        if doa != self.prev_doa:
            self.pub_doa_raw.publish(Int32(data=int(doa)))
            self.prev_doa = doa
            msg = PoseStamped()
            msg.header.frame_id = self.sensor_frame_id
            msg.header.stamp = stamp.to_msg()
            ori = quaternion_from_euler(math.radians(doa), 0, 0)
            msg.pose.position.x = self.doa_xy_offset * np.cos(doa_rad)
            msg.pose.position.y = self.doa_xy_offset * np.sin(doa_rad)
            msg.pose.orientation.w = float(ori[0])
            msg.pose.orientation.x = float(ori[1])
            msg.pose.orientation.y = float(ori[2])
            msg.pose.orientation.z = float(ori[3])
            self.pub_doa.publish(msg)

        if is_voice:
            self.speech_stopped = stamp
        period = rclpy.duration.Duration(seconds=self.speech_continuation)
        if (stamp - self.speech_stopped).nanoseconds < period.nanoseconds:
            self.is_speeching = True
        elif self.is_speeching:
            buf = self.speech_audio_buffer
            self.speech_audio_buffer = b''
            self.is_speeching = False
            duration = 8.0 * len(buf) * self.respeaker_audio.bitwidth
            duration = duration / self.respeaker_audio.rate / self.respeaker_audio.bitdepth
            self.get_logger().info("Speech detected for %.3f seconds" % duration)
            if self.speech_min_duration <= duration < self.speech_max_duration:
                self.pub_speech_audio.publish(UInt8MultiArray(data=list(buf)))

    def _save_audio(self, raw_data, prefix='speech'):
        if not self.save_audio or not raw_data:
            return
        timestamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        wav_path = self.audio_output_dir / ('%s_%s.wav' % (prefix, timestamp))
        with wave.open(str(wav_path), 'wb') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(self.respeaker_audio.bitwidth)
            wav_file.setframerate(self.respeaker_audio.rate)
            wav_file.writeframes(raw_data)
        self.get_logger().info('Saved speech audio to %s' % wav_path)

    def destroy_node(self, *args, **kwargs):
        self._shutting_down = True
        try:
            if self.respeaker_audio:
                self.respeaker_audio.stop()
        except Exception:
            pass
        finally:
            pass
        try:
            if self.is_speeching and self.speech_audio_buffer:
                self._save_audio(self.speech_audio_buffer, prefix='speech_shutdown')
        except Exception:
            pass
        finally:
            self.speech_audio_buffer = b''
            self.is_speeching = False
        try:
            if self.respeaker:
                self.respeaker.close()
        except Exception:
            pass
        finally:
            self.respeaker = None
        try:
            if self.timer_led is not None:
                self.timer_led.cancel()
        except Exception:
            pass
        finally:
            self.respeaker_audio = None
        super().destroy_node(*args, **kwargs)


def main(args=None):
    rclpy.init(args=args)
    node = RespeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
