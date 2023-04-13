import threading
import rospy
import os
from std_msgs.msg import Int32


HOST_NAME = os.environ["VEHICLE_NAME"]
STALL_TAGS = [207, 226, 228, 75]  # in the order of stall 1 to 4


# define state ids
P1_START = 0
P1_FIRST_TAG = 0
P1_STRAIGHT_0 = 10
P1_STRAIGHT_1 = 11
P1_RIGHT_0 = 20
P1_RIGHT_1 = 21
P1_END = 29

P2_START = 30
P2_CROSSWALK_0 = 30
P2_CROSSWALK_1 = 31
P2_CROSSWALK_2 = 32
P2_END = 39

P3_START = 40
P3_ENTER = 40
P3_FORWARD = 41
P3_TURN_AROUND = 42
P3_BACKING = 43
P3_END = 49

TASK_END = 50


class BotState:
    """
    defines all states needed to execute the entire project
    """
    
    def __init__(self, goal_stall):
        """
        goal_stall (int) - between 1 and 4, specifies the goal stall for parking
        """
        self.stateid_expected_tags = {
            # part1 (ends when the robot exits the circle)
            P1_FIRST_TAG: [48, 56],  # before first apriltag
            # go straight
            P1_STRAIGHT_0: [48],
            P1_STRAIGHT_1: [50],
            # turn right
            P1_RIGHT_0: [50],
            P1_RIGHT_1: [56],

            # part2 (ends when the robot sees the parking lot entry)
            P2_CROSSWALK_0: [163],  # before first crosswalk
            P2_CROSSWALK_1: [163],  # before second crosswalk
            P2_CROSSWALK_2: [38],  # before seeing the entry apriltag to the parking lot

            # part3 (have a separate variable to indicate which stall is the goal)
            P3_ENTER: [227],  # before seeing the tag inside the parking lot
            P3_FORWARD: STALL_TAGS,  # moving forward towards the goal stall
            P3_TURN_AROUND: STALL_TAGS,  # turn around 180 degrees
            P3_BACKING: STALL_TAGS,  # backing into the goal stall
        }

        self.tag_to_turn = {  # all tags associated with red stopline should indicate us where we should turn towards
            50: 0,
            56: 1,
            48: 2,
            38: 1,
        }

        self.new_state_after_turn_at_state = {
            P1_FIRST_TAG: {56: P1_STRAIGHT_0, 48: P1_RIGHT_0},
            # go straight
            P1_STRAIGHT_0: P1_STRAIGHT_1,
            P1_STRAIGHT_1: P2_CROSSWALK_0,
            # turn right
            P1_RIGHT_0: P1_RIGHT_1,
            P1_RIGHT_1: P2_CROSSWALK_0,

            # part2
            P2_CROSSWALK_0: P2_CROSSWALK_1,
            P2_CROSSWALK_1: P2_CROSSWALK_2,
            P2_CROSSWALK_2: P3_ENTER,

            # part3
            P3_ENTER: P3_FORWARD,
            P3_FORWARD: P3_TURN_AROUND,
            P3_TURN_AROUND: P3_BACKING,
            P3_BACKING: TASK_END,
        }

        self.lock = threading.Lock()
        self.goal_stall = goal_stall
        self.stateid = None
        self.last_seen_apriltag = None
        self.update_state(P1_FIRST_TAG)

        # handy flags to turn on/off some robot behaviors
        self.lane_follow = True  # if true, do lane following
        self.crosswalk_waiting = False  # if true, don't drive until the peduckstrains have crossed
        self.is_expecting_red_stopline = True
        self.is_expecting_crosswalk = False

        self.tag_sub = rospy.Subscriber(f'/{HOST_NAME}/detected_tagid', 
                                    Int32, 
                                    self.tag_callback,
                                    queue_size=1)
        
    def tag_callback(self, msg):
        id = msg.data
        if id in self.get_expected_tags():
            self.sees_tag(id)
    
    def sees_tag(self, tagid):
        self.lock.acquire()
        self.last_seen_apriltag = tagid
        self.lock.release()
    
    def is_legal_stateid(self, stateid):
        return stateid in self.stateid_expected_tags

    def get_expected_tags(self):
        return self.stateid_expected_tags[self.stateid]
    
    def get_flags(self):
        self.lock.acquire()
        flags = {
            'lane_follow': self.lane_follow,
            'crosswalk_waiting': self.crosswalk_waiting,
            'is_expecting_red_stopline': self.is_expecting_red_stopline,
            'is_expecting_crosswalk': self.is_expecting_crosswalk,
        }
        self.lock.release()
        return flags

    def get_lane_following_flag(self):
        self.lock.acquire()
        flag = self.lane_follow
        self.lock.release()
        return flag

    def decide_turn_at_red_stopline(self):
        # the robot encounters a red stopline, choose between left_turn=0, straight=1, right_turn=2

        self.lock.acquire()
        tagid = self.last_seen_apriltag
        cur_stateid = self.stateid

        turn_idx = None
        if P1_START <= cur_stateid <= P1_END or cur_stateid == P2_CROSSWALK_2:
            if tagid in self.tag_to_turn:
                turn_idx = self.tag_to_turn[tagid]
        else:
            print(f'ERROR: incompatible tagid {tagid} and stateid {cur_stateid} for turning')
        self.lock.release()

        return turn_idx
    
    # apriltag detection is sometimes unreliable, so we need to infer the tag if we can
    # infer using the new stateid of the robot
    def predict_tag(self, new_stateid):
        if new_stateid <= P3_ENTER:
            # before entering the parking stall, we just take the first tag in the expected tag array
            predicted_tag = self.stateid_expected_tags[new_stateid][0]
        else:
            # expect the tag that is associated with the goal stall
            predicted_tag = STALL_TAGS[self.goal_stall - 1]
        return predicted_tag

    def update_state(self, new_stateid):
        self.stateid = new_stateid
        self.last_seen_apriltag = self.predict_tag(new_stateid)

        # lane following is on until the parking (part 3)
        self.lane_follow = new_stateid < P3_START

        # wait for crosswalk if in part 2 after seeing crosswalk
        self.crosswalk_waiting = P2_CROSSWALK_1 <= new_stateid <= P2_CROSSWALK_2

        # same as lane following except when in the second part when we encounter crosswalk
        self.is_expecting_red_stopline = self.lane_follow and (new_stateid < P2_CROSSWALK_0 or new_stateid > P2_CROSSWALK_1)
        
        self.is_expecting_crosswalk = self.lane_follow and (not self.is_expecting_red_stopline)
        print(self.lane_follow, self.is_expecting_crosswalk)


    def advance_state(self):
        self.lock.acquire()
        next_state_info = self.new_state_after_turn_at_state[self.stateid]
        if type(next_state_info) is dict:
            new_stateid = next_state_info[self.last_seen_apriltag]
        else:
            new_stateid = next_state_info
        self.update_state(new_stateid)
        self.lock.release()
        return new_stateid
