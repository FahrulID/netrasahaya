import rosnode
import rospy
import serial as ser
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
from actionlib_msgs.msg import GoalStatusArray
from sound_play.libsoundplay import SoundClient
from typing import List
import math
from collections import deque
import threading
import time

motor_speeds: List[int] = [0, 0, 0, 0]
motor_states: List[int] = [0, 0, 0, 0]
segment_state: List[int] = [0, 0, 0, 0] # 0 = FREE, 1 = NAVIGABLE, 2 = BLOCKED
motor_toggles: List[bool] = [False, False, False, False] # 0 = pulse off, 1 = pulse on
last_toggle_time: List[float] = [0, 0, 0, 0]

class TopicMonitor:
    def __init__(self, topic_name, msg_type, timeout=1):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.timeout = timeout
        self.last_msg_time = rospy.get_time()

        self.sub = rospy.Subscriber(topic_name, msg_type, self.callback)
        self.publishing_state_before = False
        self.last_msg = None

    def callback(self, msg):
        self.last_msg = msg
        self.last_msg_time = rospy.get_time()

    def is_publishing(self):
        return (rospy.get_time() - self.last_msg_time) < self.timeout
    
    def is_covariance_low(self):
        if self.last_msg is None:
            return False
        
        if not isinstance(self.last_msg, PoseWithCovarianceStamped):
            return False
        
        cov = self.last_msg.pose.covariance
        return cov[0] < 1 and cov[7] < 1 # x and y covariance under 1 meter
    
    def is_goal_reached(self):
        if self.last_msg is None:
            return False
        
        if not isinstance(self.last_msg, GoalStatusArray):
            return False
        
        return len(self.last_msg.status_list) > 0 and self.last_msg.status_list[0].status == 3 # SUCCEEDED
    
    def is_in_navigation(self):
        if self.last_msg is None:
            return False
        
        if not isinstance(self.last_msg, GoalStatusArray):
            return False
        
        return len(self.last_msg.status_list) > 0 and self.last_msg.status_list[0].status == 1 # ACTIVE
    
    def should_notify(self):
        state_changed = self.publishing_state_before != self.is_publishing()
        self.publishing_state_before = self.is_publishing()

        if state_changed and self.is_publishing():
            return True
        
        five_secs_passed = self.last_msg_time is not None and (rospy.get_time() - self.last_msg_time) % 5 <= 0.1

        return not self.is_publishing() and five_secs_passed # Notify every 10 seconds
class AudioFeedback:
    def __init__(self, audio_path):
        self.AUDIO_PATH = audio_path
        self.sound_client = SoundClient()
        self.sound_queue = deque()
        self.is_playing = False
        
        if self.AUDIO_PATH == "None":
            self.INITIALIZED = False
            return
        
        self.INITIALIZED = True

        self.SOUNDS = {
            "goal_reached": {"file": self.AUDIO_PATH + "goal_reached.wav", "duration": 1.15},
            "sensing_ready": {"file": self.AUDIO_PATH + "sensing_ready.wav", "duration": 1.15},
            "guidance_mode": {"file": self.AUDIO_PATH + "guidance_mode.wav", "duration": 1.20},
            "sensing_mode": {"file": self.AUDIO_PATH + "sensing_mode.wav", "duration": 1.20},
            "guidance_ready": {"file": self.AUDIO_PATH + "guidance_ready.wav", "duration": 1.25},
            "sensing_not_ready": {"file": self.AUDIO_PATH + "sensing_not_ready.wav", "duration": 1.38},
            "guidance_not_ready": {"file": self.AUDIO_PATH + "guidance_not_ready.wav", "duration": 1.44},
            "navigation_started": {"file": self.AUDIO_PATH + "navigation_started.wav", "duration": 1.57}
        }

        self.current_sound_end_time = 0
        
        # Start queue processor thread
        self.queue_thread = threading.Thread(target=self._process_queue)
        self.queue_thread.daemon = True
        self.queue_thread.start()

    def _process_queue(self):
        while True:
            current_time = time.time()
            if len(self.sound_queue) > 0 and current_time >= self.current_sound_end_time:
                sound_name = self.sound_queue.popleft()
                if sound_name in self.SOUNDS:
                    sound = self.SOUNDS[sound_name]
                    self.sound_client.playWave(sound["file"])
                    self.current_sound_end_time = current_time + sound["duration"]
            time.sleep(0.1)

    def play(self, sound_name):
        if not self.INITIALIZED or sound_name not in self.SOUNDS:
            return
        self.sound_queue.append(sound_name)

    def stopAll(self):
        if not self.INITIALIZED:
            return
        self.sound_client.stopAll()
        self.sound_queue.clear()
        self.current_sound_end_time = 0

def constructByteFromTwiNibble(nibble1: int, nibble2: int) -> int:
    return (nibble1 << 4) | nibble2

def construct_command(speeds: List[int], states: List[int]) -> bytearray:
    secondByte = constructByteFromTwiNibble(states[0], states[1])
    thirdByte = constructByteFromTwiNibble(states[2], states[3])
    fourthByte = speeds[0]
    fifthByte = speeds[1]
    sixthByte = speeds[2]
    seventhByte = speeds[3]
    hashByte = secondByte ^ thirdByte ^ fourthByte ^ fifthByte ^ sixthByte ^ seventhByte

    command: bytearray = bytearray([0x66, secondByte, thirdByte, fourthByte, fifthByte, sixthByte, seventhByte, hashByte, 0x99])
    return command

MOTOR_KIRI = 1
MOTOR_KANAN = 2
MOTOR_DEPAN_KIRI = 0
MOTOR_DEPAN_KANAN = 3

PULSE_INTERVAL = 0.25 # in seconds

def should_toggle(motor_index):
    global last_toggle_time, motor_toggles, segment_state

    current_time = rospy.get_time()
    if current_time - last_toggle_time[motor_index] >= PULSE_INTERVAL and segment_state[motor_index] == 1:
        last_toggle_time[motor_index] = current_time
        motor_toggles[motor_index] = not motor_toggles[motor_index]
    else:
        motor_toggles[motor_index] = True

current_mode = "none"

def process_distance(distance: float) -> int:
    if distance == -1:
        return 0  # Stop
    elif 0 <= distance <= 0.75:
        return 3  # Close range
    elif 0.75 < distance <= 1.5:
        return 2  # Far range
    return 0  # Default stop

def front_left_distance_cb(msg: Float32):
    global motor_states

    if not current_mode == "sensing":
        return
    
    if motor_toggles[MOTOR_DEPAN_KIRI] == False:
        motor_states[MOTOR_DEPAN_KIRI] = 0
        return

    motor_states[MOTOR_DEPAN_KIRI] = process_distance(msg.data)

def front_right_distance_cb(msg: Float32):
    global motor_states

    if not current_mode == "sensing":
        return
    
    if motor_toggles[MOTOR_DEPAN_KANAN] == False:
        motor_states[MOTOR_DEPAN_KANAN] = 0
        return
    
    motor_states[MOTOR_DEPAN_KANAN] = process_distance(msg.data)

def left_distance_cb(msg: Float32):
    global motor_states

    if not current_mode == "sensing":
        return
    
    if motor_toggles[MOTOR_KIRI] == False:
        motor_states[MOTOR_KIRI] = 0
        return
    
    motor_states[MOTOR_KIRI] = process_distance(msg.data)

def right_distance_cb(msg: Float32):
    global motor_states

    if not current_mode == "sensing":
        return
    
    if motor_toggles[MOTOR_KANAN] == False:
        motor_states[MOTOR_KANAN] = 0
        return
    
    motor_states[MOTOR_KANAN] = process_distance(msg.data)

# State callbacks (boilerplate)
def front_left_state_cb(msg: Int32):
    global segment_state

    if not current_mode == "sensing":
        return
    
    segment_state[MOTOR_DEPAN_KIRI] = msg.data

def front_right_state_cb(msg: Int32):
    global segment_state

    if not current_mode == "sensing":
        return
    
    segment_state[MOTOR_DEPAN_KANAN] = msg.data

def left_state_cb(msg: Int32):
    global segment_state

    if not current_mode == "sensing":
        return
    
    segment_state[MOTOR_KIRI] = msg.data

def right_state_cb(msg: Int32):
    global segment_state

    if not current_mode == "sensing":
        return
    
    segment_state[MOTOR_KANAN] = msg.data

current_mode = None
soundplay_initialized = False
delay_for_soundplay = 3 # seconds
time_soundplay_initialized = 0
soundplay_ready = False

def handle_wait_soundplay():
    global soundplay_initialized, time_soundplay_initialized, delay_for_soundplay, soundplay_ready

    if not soundplay_initialized:
        soundplay_initialized = rosnode.rosnode_ping("/feedback/soundplay_node", max_count=1)
        if soundplay_initialized:
            time_soundplay_initialized = rospy.get_time()
        return
    
    if soundplay_initialized and rospy.get_time() - time_soundplay_initialized < delay_for_soundplay:
        return

    soundplay_ready = True

def handle_mode_change(mode: str, audio_feedback: AudioFeedback):
    global current_mode, soundplay_ready

    if not soundplay_ready:
        return

    mode_changed = current_mode != mode

    if mode_changed:
        if mode == "sensing":
            audio_feedback.play("sensing_mode")
        elif mode == "guidance":
            audio_feedback.play("guidance_mode")

    current_mode = mode

def handle_sensing_ready(t265_monitor: TopicMonitor, d435_monitor: TopicMonitor, audio_feedback: AudioFeedback):
    global current_mode, soundplay_ready

    if not current_mode == "sensing" or not soundplay_ready:
        return
    
    should_notify = t265_monitor.should_notify() or d435_monitor.should_notify()
    is_publishing = t265_monitor.is_publishing() and d435_monitor.is_publishing()

    if not should_notify:
        return
    
    if is_publishing:
        audio_feedback.play("sensing_ready")
    else:
        audio_feedback.play("sensing_not_ready")

def handle_guidance_ready(t265_monitor: TopicMonitor, d435_monitor: TopicMonitor, rtabmap_monitor: TopicMonitor, audio_feedback: AudioFeedback):
    global current_mode, soundplay_ready

    if not current_mode == "guidance" or not soundplay_ready:
        return
    
    should_notify = t265_monitor.should_notify() or d435_monitor.should_notify() or rtabmap_monitor.should_notify()
    is_publishing = t265_monitor.is_publishing() and d435_monitor.is_publishing() and rtabmap_monitor.is_publishing()
    covariance_low = rtabmap_monitor.is_covariance_low()

    if not should_notify:
        return
    
    if is_publishing and not covariance_low:
        audio_feedback.play("guidance_ready")
    else:
        audio_feedback.play("guidance_not_ready")

goal_reach_state_before = False
navigation_state_before = False
def handle_navigation(goal_monitor: TopicMonitor, audio_feedback: AudioFeedback):
    global current_mode, soundplay_ready, goal_reach_state_before, navigation_state_before

    if not current_mode == "guidance" or not soundplay_ready:
        return
    
    goal_reached = goal_monitor.is_goal_reached()
    in_navigation = goal_monitor.is_in_navigation()

    state_changed = goal_reach_state_before != goal_reached or navigation_state_before != in_navigation
    goal_reach_state_before = goal_reached
    navigation_state_before = in_navigation

    if not state_changed:
        return
    
    if goal_reached:
        audio_feedback.play("goal_reached")

    if in_navigation:
        audio_feedback.play("navigation_started")

def remap(value, leftMin, leftMax, rightMin, rightMax):
    if value < leftMin or value > leftMax:
        return 0

    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def handle_guidance(velocity: Twist):
    global motor_states, motor_speeds, current_mode

    if not current_mode == "guidance":
        return
    
    motor_states = [1, 1, 1, 1] # use manual speed control

    if velocity.linear.x == 0.0 and velocity.linear.y == 0.0:
        motor_speeds = [0, 0, 0, 0]
        return
    
    if velocity.linear.y > 0 and velocity.linear.x == 0.0:
        motor_speeds = [0, 0, 0, 0]
        motor_speeds[MOTOR_KIRI] = 255
        return
    
    if velocity.linear.y < 0 and velocity.linear.x == 0.0:
        motor_speeds = [0, 0, 0, 0]
        motor_speeds[MOTOR_KANAN] = 255
        return
    
    r = math.atan(velocity.linear.y / velocity.linear.x)
    motor_speeds[MOTOR_KIRI] = remap(r, math.pi/4, math.pi/2, 50, 255)

    motor_speeds[MOTOR_DEPAN_KIRI] = remap(r, math.pi/6, 5/12*math.pi, 255, 50) or remap(r, -1/12*math.pi, math.pi/6, 50, 255)

    motor_speeds[MOTOR_DEPAN_KANAN] = remap(r, -5/12*math.pi, -math.pi/6,  50, 255) or remap(r, -math.pi/6, 1/12*math.pi, 50, 255)

    motor_speeds[MOTOR_KANAN] = remap(r, -math.pi/2, -1/4*math.pi, 255, 50)

if __name__ == '__main__':
    rospy.init_node('feedback_driver')

    # Add these new subscribers
    front_left_distance_sub = rospy.Subscriber('/front_left/distance', Float32, front_left_distance_cb)
    front_right_distance_sub = rospy.Subscriber('/front_right/distance', Float32, front_right_distance_cb)
    left_distance_sub = rospy.Subscriber('/left/distance', Float32, left_distance_cb)
    right_distance_sub = rospy.Subscriber('/right/distance', Float32, right_distance_cb)

    front_left_state_sub = rospy.Subscriber('/front_left/state', Int32, front_left_state_cb)
    front_right_state_sub = rospy.Subscriber('/front_right/state', Int32, front_right_state_cb)
    left_state_sub = rospy.Subscriber('/left/state', Int32, left_state_cb)
    right_state_sub = rospy.Subscriber('/right/state', Int32, right_state_cb)

    velocity_sub = rospy.Subscriber('/cmd_vel', Twist, handle_guidance)

    audio_feedback = AudioFeedback(rospy.get_param("/feedback/audio_path", "None"))

    t265_monitor = TopicMonitor('/t265/odom/sample', Odometry)
    d435_monitor = TopicMonitor('/d400/depth/image_rect_raw', Image)
    rtabmap_localization_monitor = TopicMonitor('/rtabmap/localization_pose', PoseWithCovarianceStamped)
    goal_monitor = TopicMonitor('/move_base/status', GoalStatusArray)

    s = None

    try:
        s = ser.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_14101-if00', 
            baudrate=115200, 
            parity=ser.PARITY_NONE, 
            stopbits=ser.STOPBITS_ONE,
            bytesize=ser.EIGHTBITS, 
            timeout=1
        )
        if not s.isOpen():
            print("Serial port is not open, exiting...")
    except Exception as e:
        print(f'Error: {e}')

    rate = rospy.Rate(20.0)
    
    while not rospy.is_shutdown():
        try:
            handle_wait_soundplay()
            handle_sensing_ready(t265_monitor, d435_monitor, audio_feedback)
            handle_guidance_ready(t265_monitor, d435_monitor, rtabmap_localization_monitor, audio_feedback)
            handle_navigation(goal_monitor, audio_feedback)
            handle_mode_change(rospy.get_param("/mode", "none"), audio_feedback)

            should_toggle(MOTOR_KIRI)
            should_toggle(MOTOR_DEPAN_KIRI)
            should_toggle(MOTOR_DEPAN_KANAN)
            should_toggle(MOTOR_KANAN)

            if current_mode == "guidance" and not goal_monitor.is_in_navigation():
                motor_states = [0, 0, 0, 0]

            print(f'Motor speeds: {motor_speeds}, Motor states: {motor_states}')

            if s and s.isOpen():
                command = construct_command(motor_speeds, motor_states)
                s.write(command)
                print(f'Sent command: {command}')
        except Exception as e:
            print(f'Error: {e}')
            break

        rate.sleep()