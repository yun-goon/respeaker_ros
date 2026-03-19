#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
# Ported to ROS2

from datetime import datetime
import json
from pathlib import Path
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray
import speech_recognition as SR

try:
    from vosk import KaldiRecognizer, Model as VoskModel  # pyright: ignore[reportMissingImports]
except ImportError:
    KaldiRecognizer = None
    VoskModel = None


class SpeechToText(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.sample_rate = self.declare_parameter('sample_rate', 16000).value
        self.sample_width = self.declare_parameter('sample_width', 2).value
        self.stt_backend = self.declare_parameter('stt_backend', 'google').value
        self.language = self.declare_parameter('language', 'ko-KR').value
        self.fallback_languages = list(
            self.declare_parameter('fallback_languages', ['en-US']).value)
        self.offline_model_path = self.declare_parameter('offline_model_path', '').value
        self.offline_fallback_model_paths = list(
            self.declare_parameter('offline_fallback_model_paths', []).value)
        self.self_cancellation = self.declare_parameter('self_cancellation', True).value
        self.tts_tolerance = self.declare_parameter('tts_tolerance', 1.0).value
        self.save_audio = self.declare_parameter('save_audio', True).value
        self.audio_output_dir = Path(
            self.declare_parameter('audio_output_dir', '/tmp/respeaker_audio').value)
        self.languages = self._build_language_candidates()
        self.offline_model_paths = self._build_offline_model_paths()

        self.recognizer = SR.Recognizer()
        self.is_canceling = False
        self.last_tts = None
        self.tts_tolerance_timer = None
        self._vosk_models = {}

        if self.save_audio:
            self.audio_output_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info('Saving speech audio to %s' % self.audio_output_dir)
        self.get_logger().info('Speech recognition backend: %s' % self.stt_backend)
        self.get_logger().info('Speech recognition languages: %s' % ', '.join(self.languages))

        if self.self_cancellation:
            self.sub_tts = self.create_subscription(
                Bool, 'tts_speaking', self._tts_cb, 10)
        else:
            self.sub_tts = None

        self.pub_speech = self.create_publisher(
            String, 'speech_to_text', 10)
        self.sub_audio = self.create_subscription(
            UInt8MultiArray, 'audio', self._audio_cb, 10)

    def _tts_cb(self, msg):
        if msg.data:
            self.is_canceling = True
            self.last_tts = None
            try:
                if self.tts_tolerance_timer is not None:
                    self.tts_tolerance_timer.cancel()
            except Exception:
                pass
        else:
            self.last_tts = self.get_clock().now()
            try:
                if self.tts_tolerance_timer is not None:
                    self.tts_tolerance_timer.cancel()
            except Exception:
                pass
            self.tts_tolerance_timer = self.create_timer(
                self.tts_tolerance, self._end_cancellation)

    def _end_cancellation(self):
        if self.tts_tolerance_timer:
            self.tts_tolerance_timer.cancel()
            self.tts_tolerance_timer = None
        self.is_canceling = False

    def _audio_cb(self, msg):
        if self.is_canceling:
            self.get_logger().info('Speech is cancelled')
            return
        raw_data = bytes(msg.data)
        if self.save_audio:
            self._save_audio(raw_data)
        data = SR.AudioData(raw_data, self.sample_rate, self.sample_width)
        self.get_logger().info('Waiting for result %d' % len(data.get_raw_data()))
        backends = self._get_backend_order()
        for backend in backends:
            result = self._recognize_with_backend(data, raw_data, backend)
            if result is not None:
                language, text = result
                self.get_logger().info('Recognized (%s/%s): %s' % (backend, language, text))
                self.pub_speech.publish(String(data=text))
                return
        self.get_logger().warning(
            'Failed to recognize speech for backends %s and languages %s' % (
                ', '.join(backends), ', '.join(self.languages)))

    def _save_audio(self, raw_data):
        timestamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        wav_path = self.audio_output_dir / ('speech_%s.wav' % timestamp)
        with wave.open(str(wav_path), 'wb') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(self.sample_width)
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(raw_data)
        self.get_logger().info('Saved speech audio to %s' % wav_path)

    def _build_language_candidates(self):
        languages = [self.language]
        for language in self.fallback_languages:
            if language and language not in languages:
                languages.append(language)
        return languages

    def _build_offline_model_paths(self):
        model_paths = [self.offline_model_path]
        for path in self.offline_fallback_model_paths:
            model_paths.append(path)
        while len(model_paths) < len(self.languages):
            model_paths.append('')
        return model_paths[:len(self.languages)]

    def _get_backend_order(self):
        if self.stt_backend == 'auto':
            return ['vosk', 'google']
        return [self.stt_backend]

    def _recognize_with_backend(self, data, raw_data, backend):
        if backend == 'google':
            return self._recognize_with_google(data)
        if backend == 'vosk':
            return self._recognize_with_vosk(raw_data)
        self.get_logger().error('Unsupported STT backend: %s' % backend)
        return None

    def _recognize_with_google(self, data):
        for language in self.languages:
            try:
                result = self.recognizer.recognize_google(data, language=language)
                return (language, result)
            except SR.UnknownValueError:
                continue
            except SR.RequestError as e:
                self.get_logger().error('Recognition request failed for %s: %s' % (language, str(e)))
                return None
            except OSError as e:
                self.get_logger().error('Google STT dependency error: %s' % str(e))
                return None
        return None

    def _recognize_with_vosk(self, raw_data):
        if VoskModel is None or KaldiRecognizer is None:
            self.get_logger().error('Vosk is not installed. Run `pip install vosk` to use offline STT.')
            return None
        for language, model_path in zip(self.languages, self.offline_model_paths):
            if not model_path:
                continue
            recognizer = self._get_vosk_recognizer(language, model_path)
            if recognizer is None:
                continue
            if recognizer.AcceptWaveform(raw_data):
                result = json.loads(recognizer.Result())
            else:
                result = json.loads(recognizer.FinalResult())
            text = result.get('text', '').strip()
            if text:
                return (language, text)
        return None

    def _get_vosk_recognizer(self, language, model_path):
        model_key = (language, model_path)
        if model_key not in self._vosk_models:
            path = Path(model_path)
            if not path.exists():
                self.get_logger().error('Offline model path not found for %s: %s' % (language, path))
                return None
            self._vosk_models[model_key] = VoskModel(str(path))
        return KaldiRecognizer(self._vosk_models[model_key], float(self.sample_rate))


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToText()
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
