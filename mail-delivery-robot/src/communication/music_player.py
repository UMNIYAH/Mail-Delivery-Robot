import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from rclpy.time import Duration

class MusicPlayer(Node):
    def __init__(self):
        super().__init__('music_player')

        self.music_publisher = self.create_publisher(AudioNoteVector, '/cmd_audio', 1)
        self.timer = self.create_timer(13, self.play_music)

        freq_e = 330
        freq_b = 247
        freq_c = 262
        freq_d = 294
        freq_a_high = 440
        freq_f = 349
        freq_a_low = 220
        freq_g = 392
        rest = 0

        dur_half = Duration(seconds=0.4).to_msg()
        dur_quarter = Duration(seconds=0.2).to_msg()

        note_e_half = AudioNote(frequency = freq_e, max_runtime = dur_half)
        note_e_quarter = AudioNote(frequency = freq_e, max_runtime = dur_quarter)
        note_b_half = AudioNote(frequency = freq_b, max_runtime = dur_half)
        note_b_quarter = AudioNote(frequency = freq_b, max_runtime = dur_quarter)
        note_c_half = AudioNote(frequency = freq_c, max_runtime = dur_half)
        note_c_quarter = AudioNote(frequency = freq_c, max_runtime = dur_quarter)
        note_d_half = AudioNote(frequency = freq_d, max_runtime = dur_half)
        note_d_quarter = AudioNote(frequency = freq_d, max_runtime = dur_quarter)
        note_f_half = AudioNote(frequency = freq_f, max_runtime = dur_half)
        note_f_quarter = AudioNote(frequency = freq_f, max_runtime = dur_quarter)
        note_g_half = AudioNote(frequency = freq_g, max_runtime = dur_half)
        note_g_quarter = AudioNote(frequency = freq_g, max_runtime = dur_quarter)
        note_a_low_half = AudioNote(frequency = freq_a_low, max_runtime = dur_half)
        note_a_low_quarter = AudioNote(frequency = freq_a_low, max_runtime = dur_quarter)
        note_a_high_half = AudioNote(frequency = freq_a_high, max_runtime = dur_half)
        note_a_high_quarter = AudioNote(frequency = freq_a_high, max_runtime = dur_quarter)
        rest_half = AudioNote(frequency = rest, max_runtime = dur_half)
        rest_quarter = AudioNote(frequency = rest, max_runtime = dur_quarter)

        self.song = AudioNoteVector(append = False,
                                    notes = [note_e_half,
                                             note_b_quarter,
                                             note_c_quarter,
                                             note_d_half,
                                             note_c_quarter,
                                             note_b_quarter,
                                             note_a_low_half,
                                             note_a_low_quarter,
                                             note_c_quarter,
                                             note_e_half,
                                             note_d_quarter,
                                             note_c_quarter,
                                             note_b_half,
                                             note_b_quarter,
                                             note_c_quarter,
                                             note_d_half,
                                             note_e_half,
                                             note_c_half,
                                             note_a_low_half,
                                             note_a_low_half,
                                             rest_half,
                                             note_d_half,
                                             note_f_quarter,
                                             note_a_high_half,
                                             note_g_quarter,
                                             note_f_quarter,
                                             note_e_half,
                                             note_c_quarter,
                                             note_e_half,
                                             note_d_quarter,
                                             note_c_quarter,
                                             note_b_half,
                                             note_b_quarter,
                                             note_c_quarter,
                                             note_d_half,
                                             note_e_half,
                                             note_c_half,
                                             note_a_low_half,
                                             note_a_low_half,
                                             rest_half])
    def play_music(self):
        self.music_publisher.publish(self.song)


def main():
    rclpy.init()
    music_player = MusicPlayer()
    rclpy.spin(music_player)

if __name__ == '__main__':
    main()