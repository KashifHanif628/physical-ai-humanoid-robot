"""
Whisper Voice Processing for VLA Systems

This script demonstrates how to use OpenAI Whisper for voice recognition
in Vision-Language-Action robotic systems.
"""

import whisper
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import pyaudio
import wave
import threading
import queue


class WhisperVoiceProcessor:
    """
    A class to process voice commands using OpenAI Whisper for robotic applications
    """

    def __init__(self, model_size="base"):
        """
        Initialize the Whisper voice processor

        Args:
            model_size: Size of the Whisper model ('tiny', 'base', 'small', 'medium', 'large')
        """
        rospy.init_node('whisper_voice_processor')

        # Load Whisper model
        self.model = whisper.load_model(model_size)

        # Audio parameters
        self.sample_rate = 16000  # Whisper expects 16kHz
        self.chunk_size = 1024

        # Audio recording setup
        self.audio = pyaudio.PyAudio()
        self.stream = None

        # ROS publishers
        self.command_pub = rospy.Publisher('voice_command', String, queue_size=10)

        # Processing queue
        self.audio_queue = queue.Queue(maxsize=10)

        rospy.loginfo(f'Whisper voice processor initialized with {model_size} model')

    def start_listening(self):
        """
        Start listening for audio input and processing voice commands
        """
        # Start audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        # Start processing thread
        processing_thread = threading.Thread(target=self.process_audio_stream)
        processing_thread.daemon = True
        processing_thread.start()

        rospy.loginfo('Started listening for voice commands')

    def process_audio_stream(self):
        """
        Process the audio stream in a separate thread
        """
        frames = []
        silence_threshold = 500  # Adjust based on your environment
        max_frames = int(5 * self.sample_rate / self.chunk_size)  # 5 seconds max

        while not rospy.is_shutdown():
            # Read audio data
            data = self.stream.read(self.chunk_size, exception_on_overflow=False)
            audio_array = np.frombuffer(data, dtype=np.int16)

            # Check if this is "silent" audio (adjust threshold as needed)
            if np.abs(audio_array).mean() > silence_threshold:
                frames.append(data)

                # If we have accumulated enough frames, process them
                if len(frames) >= max_frames:
                    self.process_audio_chunk(frames)
                    frames = []  # Reset for next command
            else:
                # If we have accumulated frames and now detect silence, process
                if frames:
                    self.process_audio_chunk(frames)
                    frames = []  # Reset for next command

    def process_audio_chunk(self, frames):
        """
        Process a chunk of audio frames using Whisper

        Args:
            frames: List of audio frames to process
        """
        # Convert frames to numpy array
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)

        # Normalize audio to range [-1, 1]
        audio_np = audio_data.astype(np.float32) / 32768.0

        # Transcribe using Whisper
        try:
            result = self.model.transcribe(audio_np)
            text = result["text"].strip()

            if text:  # Only publish if we got meaningful text
                rospy.loginfo(f"Transcribed: {text}")

                # Publish the command
                cmd_msg = String()
                cmd_msg.data = text
                self.command_pub.publish(cmd_msg)

                # Process the command for intent
                intent = self.parse_intent(text)
                rospy.loginfo(f"Detected intent: {intent}")

        except Exception as e:
            rospy.logerr(f"Error in Whisper transcription: {e}")

    def parse_intent(self, text):
        """
        Parse intent from transcribed text

        Args:
            text: Transcribed text from Whisper

        Returns:
            String representing the detected intent
        """
        text_lower = text.lower()

        # Navigation intents
        if any(keyword in text_lower for keyword in ["go to", "move to", "navigate", "walk to", "go forward", "move forward"]):
            return "NAVIGATION"
        elif any(keyword in text_lower for keyword in ["turn", "rotate", "left", "right", "spin"]):
            return "ROTATION"
        elif any(keyword in text_lower for keyword in ["stop", "halt", "pause"]):
            return "STOP"
        elif any(keyword in text_lower for keyword in ["pick", "grasp", "take", "grab"]):
            return "MANIPULATION"
        elif any(keyword in text_lower for keyword in ["clean", "sweep", "tidy"]):
            return "CLEANING"
        else:
            return "UNKNOWN"

    def transcribe_file(self, audio_file_path):
        """
        Transcribe an audio file using Whisper

        Args:
            audio_file_path: Path to the audio file to transcribe

        Returns:
            Transcribed text
        """
        result = self.model.transcribe(audio_file_path)
        return result["text"]

    def close(self):
        """
        Clean up resources
        """
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()


def main():
    """
    Main function to demonstrate Whisper voice processing
    """
    # Initialize the voice processor
    processor = WhisperVoiceProcessor(model_size="base")

    try:
        # Start listening
        processor.start_listening()

        # Keep the node running
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down voice processor")
    finally:
        processor.close()


if __name__ == '__main__':
    main()