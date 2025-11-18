#!/usr/bin/env python3
"""
Python version of usv and uav mavlink joystick control
Controls both UAV and USV via joystick with MAVLink
"""
import sys
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64, Bool
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSet, ParamGet
from std_srvs.srv import Trigger
import copy

class Teleop:
    """Teleop class for controlling UAV and USV via joystick"""
    def __init__(self):
        rospy.init_node("wamv_mavlink_joy_teleop", anonymous=False)

        # Get parameters
        self.get_params()

        # Initialize variables
        self.enable = False # Tracks if LB (manual override) is currently pressed
        self.last_manual_control = False # Track  if we were manually controlling
        self.enable_mavlink = False
        self.speed_mode = "high"
        self.vehicle_type = "drone" # can be drone or usv
        self.input_standard = "XINPUT"

        # Initialize axes and buttons storage
        self.axes = [0.0] * 8
        self.buttons = [0] * 12

        # Initialize messages
        self.enable_auto_mode = Bool(data=False)
        self.ready_to_track = Bool(data=True)
        self.ready_to_land = Bool(data=False)

        # Initialize mappings
        self.init_maps()

        # Initialize publishers
        self.setup_publishers()

        # Initialize subscribers
        self.setup_subscribers()

        # Initialize service clients
        self.setup_service_clients()

        # Setup timer for periodic publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.timer_callback)

        rospy.loginfo(f"Teleop node initialized at {self.control_rate} Hz")

    def get_params(self):
        """Get ROS parameters"""
        # throttle settings are only for UAV
        # High throttle settings
        self.throttle = {
            "high": {
                "throttle": rospy.get_param("~throttle_high_throttle", 2.0), # for altitude
                "linear": rospy.get_param("~throttle_high_linear", 2.0), # for horizontal movement
                "angular": rospy.get_param("~throttle_high_angular", 1.0) # for horizontal rotation
            },
            "low": {
                "throttle": rospy.get_param('~throttle_low_throttle', 0.5),
                "linear": rospy.get_param('~throttle_low_linear', 0.5),
                "angular": rospy.get_param('~throttle_low_angular', 0.5)
            }
        }

        self.control_rate = rospy.get_param("~control_rate", 10.0)

        rospy.loginfo(f"throttle_high_throttle: {self.throttle['high']['throttle']}")
        rospy.loginfo(f"throttle_high_linear: {self.throttle['high']['linear']}")
        rospy.loginfo(f"throttle_high_angular: {self.throttle['high']['angular']}")
        rospy.loginfo(f"throttle_low_throttle: {self.throttle['low']['throttle']}")
        rospy.loginfo(f"throttle_low_linear: {self.throttle['low']['linear']}")
        rospy.loginfo(f"throttle_low_angular: {self.throttle['low']['angular']}")
        rospy.loginfo(f"control_rate: {self.control_rate}")

    def setup_publishers(self):
        """setup ROS publishers"""
        self.pub_twist_uav = rospy.Publisher("~twist_uav", Twist, queue_size=10)
        self.pub_twist_usv = rospy.Publisher("~twist_usv", Twist, queue_size=10)
        # Note: WAMV also uses twist publisher - a separate node handles twist to thrust conversion
        self.pub_enable_auto_mode = rospy.Publisher('/enable_auto_mode_success', Bool, queue_size=10)
        self.pub_ready_to_track = rospy.Publisher('/ready_to_track_success', Bool, queue_size=10)
        self.pub_ready_to_land = rospy.Publisher('/ready_to_land_success', Bool, queue_size=10)

    def setup_subscribers(self):
        """Setup ROS subscribers"""
        self.sub_joy = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=10)

    def setup_service_clients(self):
        """Setup ROS service clients"""
        self.srv_arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.srv_land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.srv_offboard = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.srv_param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.srv_param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        self.srv_kill = rospy.ServiceProxy('/drone/kill', Trigger) 

    def init_maps(self):
        """Initialize joystick button ans axis mappings"""
        # Function mapping
        self.function_map = {
            "THROTTLE": "LY",
            "YAW": "LX",
            "X": "RY",
            "Y": "RX",
            "AUTO": "START",
            "MANUAL": "BACK",
            "LAND": "Y",
            "DISARM": "X",
            "ARM_OFFBOARD": "A",
            "AUTO_TRACK": "L3",
            "AUTO_LAND": "R3",
            "KILL": "B"
        }

        if self.input_standard == "XINPUT":
            # XINPUT axes mapping
            self.axes_map = {
                "LX": 0,
                "LY": 1,
                "LT": 2,
                "RX": 3,
                "RY": 4,
                "RT": 5,
                "DPAD_X": 6,
                "DPAD_Y": 7,
            }

            self.button_map = {
                "A": 0,
                "B": 1,
                "X": 2,
                "Y": 3,
                "LB": 4,
                "RB": 5,
                "BACK": 6,
                "START": 7,
                "LOGO": 8,
                "L3": 9,
                "R3": 10
            }
        elif self.input_standard == "DIRECTINPUT":
            # DIRECTINPUT axes mapping
            self.axes_map = {
                "LX": 0,
                "LY": 1,
                "RX": 2,
                "RY": 3,
                "DPAD_X": 4,
                "DPAD_Y": 5
            }

            # DIRECTINPUT button mapping
            self.button_map = {
                "X": 0,
                "A": 1,
                "B": 2,
                "Y": 3,
                "LB": 4,
                "RB": 5,
                "LT": 6,
                "RT": 7,
                "BACK": 8,
                "START": 9,
                "L3": 10,
                "R3": 11
            }
        else:
            rospy.logerr("Joystick input standard error")

    def update_input_standard(self, msg):
        """Update joystick input standard based on message size"""
        axes_size = len(msg.axes)
        buttons_size = len(msg.buttons)

        if axes_size == 6 and buttons_size == 12:
            if self.input_standard == "XINPUT":
                rospy.loginfo("Joystick input standard changed to DIRECTINPUT")
                self.input_standard = "DIRECTINPUT"
                self.init_maps()
                self.axes = [0.0] * axes_size
                self.buttons = [0] * axes_size
                return True
        elif axes_size == 8 and buttons_size == 11:
            if self.input_standard == "DIRECTINPUT":
                rospy.loginfo("Joystick input standard changed to XINPUT")
                self.input_standard = "XINPUT"
                self.init_maps()
                self.axes = [0.0] * axes_size
                self.buttons = [0] * buttons_size
                return True
        else:
            rospy.logerr("Joystick input standard error")
            return False
        return False
    
    def is_pressed(self, function, msg):
        """Check if a button is pressed"""
        if function in self.function_map:
            button_name = self.function_map[function]
            return msg.buttons[self.button_map[button_name]]
        else:
            return msg.buttons[self.button_map[function]]
        
    def is_trigger(self, function, msg):
        """Check if a button is triggered (pressed this frame but not last frame)"""
        if function in self.function_map:
            button_name = self.function_map[function]
            button_idx = self.button_map[button_name]
            return self.is_pressed(function, msg) and not self.buttons[button_idx]
        else:
            button_idx = self.button_map[function]
            return self.is_pressed(function, msg) and not self.buttons[button_idx]
        
    def joy_callback(self, msg):
        """Joystick callback function"""
        self.update_input_standard(msg)

        # Handle LOGO button - toggle auto mode # LOGO only defined in XINPUT mode
        if self.is_trigger("LOGO", msg):
            rospy.loginfo("LOGO")
            self.enable_auto_mode.data = not self.enable_auto_mode.data
            rospy.loginfo(f"Track trigger: {self.enable_auto_mode.data}")

        # Handle ARM_OFFBOARD button (X button) - Arm and set offboard mode
        if self.is_trigger("ARM_OFFBOARD", msg):
            rospy.loginfo("A - ARM_OFFBOARD")
            self.arm_and_offboard()

        # Handle DISARM button 
        if self.is_trigger("DISARM", msg):
            rospy.loginfo("X - DISARM triggered")
            self.disarm_drone()

        # Handle LAND button (Y button) - Land the drone
        if self.is_trigger("LAND", msg):
            rospy.loginfo("Y - LAND")
            self.land_drone()

        # Handle RB button - toggle vehicle type
        if self.is_trigger("RB", msg):
            rospy.loginfo("RB")
            if self.vehicle_type == "drone":
                self.vehicle_type = "usv"
            else:
                self.vehicle_type = "drone"
            rospy.loginfo(f"Vehicle type: {self.vehicle_type}")
        
        # Handle START button - set auto mode 
        if self.is_trigger("AUTO", msg):
            rospy.loginfo("START - AUTO")
            # Reset twist to zero when entering auto mode
            zero_twist = Twist()
            zero_twist.linear.x = 0.0
            zero_twist.linear.y = 0.0
            zero_twist.linear.z = 0.0
            zero_twist.angular.x = 0.0
            zero_twist.angular.y = 0.0
            zero_twist.angular.z = 0.0
            self.pub_twist_uav.publish(zero_twist)

        # Handle BACK button - set manual mode
        if self.is_trigger("MANUAL", msg):
            rospy.loginfo("BACK - MANUAL")
            # Reset twist to zero when entering manual mode
            zero_twist = Twist()
            zero_twist.linear.x = 0.0
            zero_twist.linear.y = 0.0
            zero_twist.linear.z = 0.0
            zero_twist.angular.x = 0.0
            zero_twist.angular.y = 0.0
            zero_twist.angular.z = 0.0
            self.pub_twist_uav.publish(zero_twist)

        # Handle LB button - pressed for manual override
        # Check if LB is currently pressed (not triggered)
        self.enable = self.is_pressed("LB", msg)
        if self.enable and not self.last_manual_control:
            rospy.loginfo("LB pressed - Manual override activated")
        elif not self.enable and self.last_manual_control:
            rospy.loginfo("LB released - Reverting to previous mode")
        self.last_manual_control = self.enable

        # Handle AUTO_TRACK button (L3) - toggle ready to track
        if self.is_trigger("AUTO_TRACK", msg):
            rospy.loginfo("L3 - AUTO_TRACK")
            self.ready_to_track.data = not self.ready_to_track.data
            rospy.loginfo(f"Ready to track: {self.ready_to_track.data}")
        
        # Handle AUTO_LAND button (R3) - toggle ready to land
        if self.is_trigger("AUTO_LAND", msg):
            rospy.loginfo("R3 - AUTO_LAND")
            self.ready_to_land.data = not self.ready_to_land.data
            rospy.loginfo(f"Ready to land: {self.ready_to_land.data}")
        
        # Handle KILL button (B) - emergency kill
        if self.is_trigger("KILL", msg):
            rospy.loginfo("B - KILL")
            self.kill_drone()
        
        # Stroe current button states for next callback
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        

    def arm_and_offboard(self):
        """Arm the drone and set it to offboard mode"""
        try:
            # Wait for services to be available
            rospy.wait_for_service('/mavros/cmd/arming', timeout=1.0)
            rospy.wait_for_service('/mavros/set_mode', timeout=1.0)

            try:
                rospy.wait_for_service('/mavros/param/set', timeout=1.0)

                # Try to set parameters 
                try:
                    # Set COM_RCL_EXCEPT parameter
                    param_value = ParamValue()
                    param_value.integer = 4
                    param_value.real = 0.0
                    response = self.srv_param_set("COM_RCL_EXCEPT", param_value)
                    if response.success:
                        rospy.loginfo(f"Set COM_RCL_EXCEPT: 4. Get COM_RCL_EXCEPT: {response.value.integer}")                
                except Exception as e:
                    rospy.logwarn(f"Could not set COM_RCL_EXCEPT (may not exist on this FCU): {e}")
                
                try:
                # Set COM_OBL_ACT parameter
                    param_value = ParamValue()
                    param_value.integer = -1
                    param_value.real = 0.0
                    response = self.srv_param_set("COM_OBL_ACT", param_value)
                    if response.success:
                        rospy.loginfo(f"Set COM_OBL_ACT: -1. Get COM_OBL_ACT: {response.value.integer}")
                except Exception as e:
                    rospy.logwarn(f"Could not set COM_OBL_ACT (may not exist on this FCU): {e}")

            except rospy.ROSException:
                rospy.logwarn(f"Parameter service not available, skipping parameter setup")

            # Set offboard mode
            rospy.loginfo("Setting OFFBOARD mode...")
            try:
                offboard_response = self.srv_offboard(custom_mode="OFFBOARD")
                if offboard_response.mode_sent:
                    rospy.loginfo("Offboard mode set")
                else:
                    rospy.logerr("Offboard mode failed")
            except Exception as e:
                rospy.logerr(f"OFFBOARD mode failed: {e}")
                rospy.loginfo("Trying alternative mode setting...")
                try:
                    # Some systems use MAV_MODE instead
                    offboard_response = self.srv_offboard(base_mode=0, custom_mode="OFFBOARD")
                    rospy.loginfo("Alternative mode setting attempted")
                except Exception as e2:
                    rospy.logerr(f"Alternative mode also failed: {e2}")
            
            # Small delay to let mode change propagate
            rospy.sleep(0.5)

            # Arm the drone
            rospy.loginfo("Arming...")
            try:
                arm_response = self.srv_arming(value=True)
                if arm_response.success:
                    rospy.loginfo("✓ Arming successful")
                else:
                    rospy.logerr(f"✗ Arming failed: {arm_response.result}")
            except Exception as e:
                rospy.logerr(f"✗ Arming service call failed: {e}")
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"Service timeout: {e}")
            rospy.logerr("Make sure MAVROS is running and connected to FCU")
        except Exception as e:
            rospy.logerr(f"Unexpected error in arm_and_offboard: {e}")

    def disarm_drone(self):
        """Disarm the drone with safety checks"""
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=1.0)

            # SAFETY CHECK: Only disarm if on ground or in manual mode
            # You could check altitude, flight mode, or require confirmation

            # Require double-press
            if not hasattr(self, 'disarm_confirm'):
                self.disarm_comfirm = rospy.Time.now()
                rospy.logwarn("Press DISARM again within 2 seconds to confirm")
                return
            
            if (rospy.Time.now() - self.disarm_confirm).to_sec() > 2.0:
                self.disarm_confirm = rospy.Time.now()
                rospy.logwarn("Press DISARM again within 2 seconds to confirm")
                return
            
            del self.disarm_confirm

            # Disarm command
            disarm_response = self.srv_arming(False)
            if disarm_response.success:
                rospy.loginfo("Disarm successful")
            else:
                rospy.logerr("Disarm failed")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"Service timeout: {e}")

    def land_drone(self):
        """Land the drone"""
        try:
            rospy.wait_for_service('/mavros/cmd/land', timeout=1.0)

            land_response = self.srv_land(
                min_pitch=0.0,
                yaw=0.0,
                latitude=0.0,
                longitude=0.0,
                altitude=0.0
            )

            if land_response.success:
                rospy.loginfo("Landing command sent")
            else:
                rospy.logerr("Landing failed")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"Service timeout: {e}")

    def kill_drone(self):
        """Emergency kill the drone"""
        try:
            rospy.wait_for_service('/drone/kill', timeout=1.0)

            kill_response = self.srv_kill()
            if kill_response.success:
                rospy.loginfo("Kill successful")
            else:
                rospy.logerr("Kill failed")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Kill service does not exist: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"Service timeout: {e}")
        
    def timer_callback(self, event):
        """Timer callback for periodic publishing"""
        # Only publish control commands if not in auto mode or if enable is True
        in_manual_mode = not self.enable_auto_mode.data
        manual_override_active = self.enable # LB is pressed

        if in_manual_mode or manual_override_active:
            # Publish joystick commands (drone or usv)
            if self.vehicle_type == "drone":
                # Control UAV (Drone)
                twist_uav = Twist()
                twist_uav.linear.x = self.throttle[self.speed_mode]["linear"] * self.axes[self.axes_map["RY"]]
                twist_uav.linear.y = self.throttle[self.speed_mode]["linear"] * self.axes[self.axes_map["RX"]]
                twist_uav.linear.z = self.throttle[self.speed_mode]["throttle"] * self.axes[self.axes_map["LY"]]
                twist_uav.angular.z = self.throttle[self.speed_mode]["angular"] * self.axes[self.axes_map["LX"]]

                self.pub_twist_uav.publish(twist_uav)

                # Throttled logging (every 1 second)
                if rospy.get_time() % 1.0 < (1.0 / self.control_rate):
                    log_msg = (f"drone: LX: {self.axes[self.axes_map['LX']]:.2f}, "
                               f"LY: {self.axes[self.axes_map['LY']]:.2f}, "
                               f"RX: {self.axes[self.axes_map['RX']]:.2f}, "
                               f"RY: {self.axes[self.axes_map['RY']]:.2f}")
                    sys.stdout.write(f"\r{log_msg}")
                    sys.stdout.flush()  
                    
            elif self.vehicle_type == "usv":
                # Control USV - publish Twist message
                # Another node will convert twist2thrust commands
                usv_twist = Twist()
                usv_twist.linear.x = self.axes[self.axes_map["LY"]] * 0.3
                usv_twist.linear.y = 0.0
                usv_twist.linear.z = 0.0
                usv_twist.angular.x = 0.0
                usv_twist.angular.y = 0.0
                usv_twist.angular.z = self.axes[self.axes_map["RX"]]

                self.pub_twist_usv.publish(usv_twist)

                # Throttled logging (every 1 second)
                if rospy.get_time() % 1.0 < (1.0 / self.control_rate):
                    log_msg = (f"usv: linear.x: {usv_twist.linear.x:.2f}, "
                               f"angular.z: {usv_twist.angular.z:.2f}")
                    sys.stdout.write(f"\r{log_msg}")
                    sys.stdout.flush()
        
        else:
            # When LB released, bahavior depends on mode
            if in_manual_mode:
                # In manual mode: Send zero twist to HOVER
                zero_twist = Twist()
                zero_twist.linear.x = 0.0
                zero_twist.linear.y = 0.0
                zero_twist.linear.z = 0.0
                zero_twist.angular.x = 0.0
                zero_twist.angular.y = 0.0
                zero_twist.angular.z = 0.0
                if self.vehicle_type == "usv":
                    self.pub_twist_usv.publish(zero_twist)
                elif self.vehicle_type == "drone":
                    self.pub_twist_uav.publish(zero_twist)
            # else: In auto mode - let autonomous system control

        # always publish status messages
        self.pub_enable_auto_mode.publish(self.enable_auto_mode)
        self.pub_ready_to_land.publish(self.ready_to_land)
        self.pub_ready_to_track.publish(self.ready_to_track)

    def run(self):
        rospy.spin()

def main():
    """Main function"""
    try:
        teleop = Teleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()