import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from rclpy.time import Duration

class MusicPlayer(Node):
    def __init__(self):
        super().__init__('music_player')

        self.music_publisher = self.create_publisher(AudioNoteVector, '/cmd_audio', 1)
        self.timer = self.create_timer(13, self.play_music)

        note_e = 330
        note_b = 247
        note_c = 262
        note_d = 294
        note_a_high = 440
        note_f = 349
        note_a_low = 220
        note_g = 392
        rest = 0

        dur_half = 0.4
        dur_quarter = 0.2

        self.song = AudioNoteVector(append = False,
                                    notes = [AudioNote(frequency = note_e, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_b, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_d, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_b, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_a_low, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_a_low, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_e, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_d, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_b, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_b, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_d, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_e, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_a_low, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_a_low, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = rest, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_d, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_f, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_a_high, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_g, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_f, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_e, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_e, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_d, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_b, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_b, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_quarter).to_msg()),
                                             AudioNote(frequency = note_d, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_e, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_c, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_a_low, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = note_a_low, max_runtime = Duration(seconds=dur_half).to_msg()),
                                             AudioNote(frequency = rest, max_runtime = Duration(seconds=dur_half).to_msg())])
    def play_music(self):
        self.music_publisher.publish(self.song)


def main():
    rclpy.init()
    music_player = MusicPlayer()
    rclpy.spin(music_player)

if __name__ == '__main__':
    main()