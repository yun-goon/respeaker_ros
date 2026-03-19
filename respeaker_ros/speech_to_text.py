#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
# Ported to ROS2

from datetime import datetime
from pathlib import Path
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray
import speech_recognition as SR


class SpeechToText(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.sample_rate = self.declare_parameter('sample_rate', 16000).value
        self.sample_width = self.declare_parameter('sample_width', 2).value
        self.language = self.declare_parameter('language', 'ko-KR').value
        self.fallback_languages = list(
            self.declare_parameter('fallback_languages', ['en-US']).value)
        self.self_cancellation = self.declare_parameter('self_cancellation', True).value
        self.tts_tolerance = self.declare_parameter('tts_tolerance', 1.0).value
        self.save_audio = self.declare_parameter('save_audio', True).value
        self.audio_output_dir = Path(
            self.declare_parameter('audio_output_dir', '/tmp/respeaker_audio').value)
        self.languages = self._build_language_candidates()

        self.recognizer = SR.Recognizer()
        self.is_canceling = False
        self.last_tts = None
        self.tts_tolerance_timer = None

        if self.save_audio:
            self.audio_output_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info('Saving speech audio to %s' % self.audio_output_dir)
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
        for language in self.languages:
            try:
                result = self.recognizer.recognize_google(data, language=language)
                self.get_logger().info('Recognized (%s): %s' % (language, result))
                self.pub_speech.publish(String(data=result))
                return
            except SR.UnknownValueError:
                continue
            except SR.RequestError as e:
                self.get_logger().error('Recognition request failed for %s: %s' % (language, str(e)))
                return
        self.get_logger().warning('Failed to recognize speech for languages: %s' % ', '.join(self.languages))

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
