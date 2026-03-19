# respeaker_ros

ROS 2 `ament_python` package for ReSpeaker Mic Array.

## Overview

This package provides:

- ReSpeaker USB device initialization
- multi-channel audio capture
- voice activity detection and direction of arrival publishing
- speech segment publishing
- optional speech-to-text using `SpeechRecognition`
- optional `.wav` saving for captured speech segments

## Supported Device

- [ReSpeaker Mic Array v2.0](http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)

## Package Structure

- `respeaker_node`: hardware access, VAD, DoA, audio publishing
- `speech_to_text`: STT node for `/speech_audio`
- `respeaker_gencfg`: dump current device parameters to YAML
- `launch/respeaker_launch.py`: default launch entrypoint

## Requirements

System packages:

```bash
sudo apt install -y portaudio19-dev flac
```

Python packages:

```bash
pip install -r requirements.txt
```

Optional offline STT:

```bash
pip install vosk
```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select respeaker_ros
source install/setup.bash
```

## USB Permission Setup

Install the udev rule:

```bash
sudo cp config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reconnect the device after applying the rule.

If you still get `usb.core.USBError: [Errno 13] Access denied`, the problem is usually USB permission setup.

## Run

Launch the full stack:

```bash
ros2 launch respeaker_ros respeaker_launch.py
```

The default launch does not reset the USB device. This helps preserve system input volume settings that may otherwise be reset when the device reconnects.

Run nodes individually:

```bash
ros2 run respeaker_ros respeaker_node
ros2 run respeaker_ros speech_to_text
```

## Main Topics

```bash
ros2 topic echo /is_speeching
ros2 topic echo /sound_direction
ros2 topic echo /sound_localization
ros2 topic echo /audio
ros2 topic echo /speech_audio
ros2 topic echo /speech_to_text
```

Topic summary:

- `/audio`: main channel audio as `std_msgs/msg/UInt8MultiArray`
- `/speech_audio`: speech-only audio segments as `std_msgs/msg/UInt8MultiArray`
- `/speech_to_text`: recognized text as `std_msgs/msg/String`
- `/sound_direction`: DoA in degrees
- `/sound_localization`: DoA as `geometry_msgs/msg/PoseStamped`
- `/is_speeching`: VAD state

## Speech-to-Text

The `speech_to_text` node can use either online Google STT or offline Vosk.

Notes:

- the `flac` command line tool must be installed because `SpeechRecognition` uses it for Google STT requests
- internet access is required for Google recognition
- offline recognition is available through `vosk`
- recognition success is published on `/speech_to_text`
- default recognition order is Korean first, then English fallback
- if recognition fails, `.wav` files can be used to inspect the actual input audio

Check recognized text:

```bash
ros2 topic echo /speech_to_text
```

Backend examples:

```bash
ros2 launch respeaker_ros respeaker_launch.py stt_backend:=google
ros2 launch respeaker_ros respeaker_launch.py stt_backend:=auto offline_model_path:=/path/to/vosk-model-small-ko-0.22
ros2 launch respeaker_ros respeaker_launch.py stt_backend:=vosk offline_model_path:=/path/to/vosk-model-small-ko-0.22
```

For Korean + English offline fallback, provide the primary Korean model as `offline_model_path` and set the English fallback model in code or parameters if needed.

## WAV Saving

By default, speech segments are saved to:

```bash
/tmp/respeaker_audio
```

Examples:

```bash
ls /tmp/respeaker_audio
aplay "$(ls -t /tmp/respeaker_audio/*.wav | head -n 1)"
```

Behavior:

- normal speech segments are saved as `speech_*.wav`
- if you stop the launch with `Ctrl+C` while speaking, a partial segment is saved as `speech_shutdown_*.wav`

## Launch Arguments

Show all launch arguments:

```bash
ros2 launch respeaker_ros respeaker_launch.py --show-args
```

Common examples:

```bash
ros2 launch respeaker_ros respeaker_launch.py audio_output_dir:=/home/$USER/respeaker_audio
ros2 launch respeaker_ros respeaker_launch.py save_audio:=false
ros2 launch respeaker_ros respeaker_launch.py main_channel:=0
ros2 launch respeaker_ros respeaker_launch.py reset_device:=true
ros2 launch respeaker_ros respeaker_launch.py stt_backend:=auto offline_model_path:=/path/to/vosk-model
```

Default speech-to-text languages:

```text
ko-KR
en-US
```

## Device Parameter Dump

Generate a YAML file from current device parameters:

```bash
ros2 run respeaker_ros respeaker_gencfg /tmp/respeaker_params.yaml
```

Then load it:

```bash
ros2 run respeaker_ros respeaker_node --ros-args --params-file /tmp/respeaker_params.yaml
```

## LED Control

```bash
ros2 topic pub /status_led std_msgs/msg/ColorRGBA "{r: 0.0, g: 0.0, b: 1.0, a: 0.3}"
```

## Known Limitations

- `speech_to_text` currently logs failures, but successful recognition is mainly observed through `/speech_to_text`
- saved `.wav` files accumulate until removed manually
- `static_transform_publisher` still uses old-style arguments and prints a deprecation warning
- external package `angles` may print `SyntaxWarning` on Python 3.12
- offline Vosk recognition requires a separately downloaded model directory

## License

[Apache License](LICENSE)
