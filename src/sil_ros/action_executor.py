#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf
import math
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from dynamic_reconfigure.client import Client
from sil_ros.destination_resolver import DestinationResolver
import rospkg, os
import numpy as np
from datetime import datetime, timedelta

class ActionExecutor:
    def __init__(self, data_logger, perception_module, llm_interface):
        self.data_logger = data_logger
        self.perception_module = perception_module
        self.llm_interface = llm_interface
        self.move_base_action = rospy.get_param("topics/move_base_action", "move_base")
        self.move_base_client = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base")
        self.response_publisher = rospy.Publisher(
            rospy.get_param("topics/llm_output", "/llm_output"), String, queue_size=10)
        self.tts_publisher = rospy.Publisher(
            rospy.get_param("topics/tts_text", "/tts_text"), String, queue_size=10)
        self.image_publisher = rospy.Publisher(
            rospy.get_param("topics/llm_image_output", "/llm_image_output"), Image, queue_size=10)
        self.linear_speed = rospy.get_param("speeds/default_linear_speed", 0.2)
        self.angular_speed = rospy.get_param("speeds/default_angular_speed", 0.5)
        self.max_speed = rospy.get_param("speeds/maximum_speed", 1.0)
        self.min_speed = rospy.get_param("speeds/minimum_speed", 0.2)
        self.action_delay = float(rospy.get_param("execution/action_delay", 0.5))
        self.progress_update_interval = float(
            rospy.get_param("execution/progress_update_interval", 30.0))
        self.control_rate_hz = float(rospy.get_param("execution/control_rate_hz", 10))
        self.nav_complexity = float(rospy.get_param("execution/navigation_complexity", 1.5))
        self.default_complexity = float(rospy.get_param("execution/default_complexity", 1.0))
        self.emergency_reset_delay = float(rospy.get_param("execution/action_delay", 0.5))
        self.goal_frame = rospy.get_param("execution/goal_frame", "map")
        self.navigation_timeout = float(rospy.get_param("execution/navigation_timeout", 0.0))

        action_defaults = {
            "forward": rospy.get_param("sil/action_generation/default_distances/forward", 1.0),
            "backward": rospy.get_param("sil/action_generation/default_distances/backward", 1.0),
            "turn": rospy.get_param("sil/action_generation/default_angles/turn", 90.0),
            "rotate": rospy.get_param("sil/action_generation/default_angles/rotate", 360.0),
            "circle_radius": rospy.get_param("sil/action_generation/default_circle_radius", 0.5),
        }
        from sil_ros.skills import build_registry
        self.skill_registry = build_registry(action_defaults)
        self.skill_registry.filter(
            enabled=rospy.get_param("sil/skills/enabled", None),
            disabled=rospy.get_param("sil/skills/disabled", None),
        )
        self.current_position = None
        self.current_orientation = None
        self.last_valid_detections = {} 
        self.performance_tracker = {
            'task_start_time': None,
            'completion_times': [],
            'success_count': 0,
            'total_actions': 0,
            'distance_traveled': 0.0,
            'error_count': 0,
            'accuracy_scores': [],
            'learning_events': []
        }
        rospy.Subscriber(rospy.get_param("topics/odom", "/odom"), Odometry, self.odom_callback)
        self.cmd_vel_publisher = rospy.Publisher(
            rospy.get_param("topics/cmd_vel", "/cmd_vel"), Twist, queue_size=10)
        pkg_path = rospkg.RosPack().get_path('sil_ros')
        yaml_path = os.path.join(pkg_path, "config/sil_config.yaml")
        self.dest_resolver = DestinationResolver(yaml_path)
        self.planner_ns = rospy.get_param(
            "execution/planner_reconfigure_ns", "/move_base/DWAPlannerROS")
        self.planner_speed_param = rospy.get_param(
            "execution/planner_speed_param", "max_vel_x")
        move_base_timeout = float(rospy.get_param("execution/move_base_timeout", 5.0))
        try:
            self.dyn_client = Client(self.planner_ns, timeout=move_base_timeout)
        except Exception as e:
            rospy.logwarn(f"[Executor] Dynamic reconfigure unavailable on {self.planner_ns}: {e}")
            self.dyn_client = None
        self.emergency_stop = False
        self.current_action_thread = None
        rospy.Subscriber(
            rospy.get_param("topics/emergency_stop", "/emergency_stop"),
            String, self.handle_emergency_stop)

    def odom_callback(self, msg):
        new_position = msg.pose.pose.position
        if self.current_position:
            dx = new_position.x - self.current_position.x
            dy = new_position.y - self.current_position.y
            distance_delta = math.sqrt(dx**2 + dy**2)
            self.performance_tracker['distance_traveled'] += distance_delta
        self.current_position = new_position
        self.current_orientation = msg.pose.pose.orientation

    def generate_performance_metrics(self, duration, success, accuracy):
        try:
            metrics_data = {
                'success_rate': self.performance_tracker['success_count'] / max(1, self.performance_tracker['total_actions']),
                'avg_completion_time': np.mean(self.performance_tracker['completion_times']) if self.performance_tracker['completion_times'] else duration,
                'total_tasks': self.performance_tracker['total_actions']
            }
            rospy.loginfo(f"[Executor] Generating performance report with: {metrics_data}")
            metrics_prompt = f"""
            Generate a natural performance report: Task completed in {duration:.1f}s, 
            Success rate: {metrics_data['success_rate']:.1%}, Average time: {metrics_data['avg_completion_time']:.1f}s.
            Create 1-2 sentences highlighting relevant metrics.
            """ 
            report = self.llm_interface.call_llm(
                "You are providing performance feedback.", metrics_prompt
            ).strip()
            rospy.loginfo(f"[Executor] Generated performance report: {report}")
            return report
        except Exception as e:
            rospy.logerr(f"[Executor] Error generating performance metrics: {e}")
            return f"Task completed in {duration:.1f} seconds with {'success' if success else 'some issues'}."

    def generate_dynamic_error_report(self, error, context):
        error_prompt = f"""
        Create a helpful error message for: {str(error)} in context: {context}
        Be friendly and suggest solutions.
        """
        
        return self.llm_interface.call_llm(
            "You are explaining an error helpfully.", error_prompt
        ).strip()
    
    def speak(self, text):
        if self.response_publisher:
            self.response_publisher.publish(String(data=text))
        if self.tts_publisher:
            self.tts_publisher.publish(String(data=text))

    def handle_emergency_stop(self, msg):
        if msg.data.lower() in [w.lower() for w in rospy.get_param('safety/emergency_stop_keywords', ['stop', 'halt', 'cancel'])]:
            rospy.logwarn("[Executor] Emergency stop triggered!")
            self.emergency_stop = True
            if hasattr(self, 'move_base_client'):
                self.move_base_client.cancel_all_goals()
            self.cmd_vel_publisher.publish(Twist())
            rospy.Timer(rospy.Duration(self.emergency_reset_delay), lambda x: setattr(self, 'emergency_stop', False), oneshot=True)

    def execute_actions(self, actions):
        if not actions:
            return
        self._in_sequence_timing = True
        complexity_score = len(actions) * 1.0 + sum(1.2 if 'navigate' in str(action).lower() else 1.0 for action in actions)
        self.start_task_timing(f"sequence_of_{len(actions)}_actions", complexity_score)
        success_count = 0
        total_accuracy = 0.0
        start_time = time.time()
        last_progress_time = start_time
        
        for idx, action in enumerate(actions, 1):
            if action:
                rospy.loginfo(f"[Executor] Sequence step {idx}/{len(actions)}: {action['action']}")
                current_time = time.time()
                if current_time - last_progress_time >= self.progress_update_interval:
                    progress_msg = f"Progress update: Completing step {idx} of {len(actions)} - {action['action']}"
                    self.speak(progress_msg)
                    last_progress_time = current_time
                action_start = time.time()
                success = self.execute_action(action)
                action_duration = time.time() - action_start
                
                if success:
                    success_count += 1
                accuracy = self.calculate_action_accuracy(action, success, action_duration)
                total_accuracy += accuracy
                rospy.sleep(self.action_delay)
        total_duration = time.time() - start_time
        overall_success = success_count == len(actions)
        overall_accuracy = total_accuracy / len(actions) if actions else 0.0
        if hasattr(self, '_in_sequence_timing'):
            delattr(self, '_in_sequence_timing')
        performance_report = self.end_task_timing(overall_success, overall_accuracy)
        performance_metrics = self.end_task_timing(overall_success, overall_accuracy)

        if performance_report and len(performance_report.strip()) > 5:
            rospy.loginfo(f"[Executor] Sequence completed - speaking final report: {performance_report}")
            #self.speak(performance_report)
            return performance_metrics
        else:
            rospy.logwarn("[Executor] No sequence performance report generated")
            #self.speak(f"Sequence of {len(actions)} tasks completed with {success_count}/{len(actions)} successful.")

    def execute_action(self, action):
        """Execute with enhanced error handling and performance tracking."""
        if hasattr(self, 'emergency_stop') and self.emergency_stop:
            rospy.loginfo("[Executor] Action cancelled before start")
            self.speak("Action cancelled.")
            self.emergency_stop = False
            return False
        
        action_type = action.get('action')
        rospy.loginfo(f"[Executor] Executing action: {action_type} with params: {action}")
        
        individual_action = not hasattr(self, '_in_sequence_timing')
        if individual_action:
            complexity = self.nav_complexity if 'NAVIGATE' in action_type else self.default_complexity
            self.start_task_timing(action_type, complexity)
        
        try:
            success = self.skill_registry.execute(self, action)
            if success is None:
                rospy.logwarn(f"Unknown action type: {action_type}")
                error_msg = self.generate_dynamic_error_report(
                    f"Unknown action: {action_type}",
                    f"Attempted to execute unrecognized action type"
                )
                self.speak(error_msg)
                success = False
            if individual_action:
                accuracy = self.calculate_action_accuracy(action, success, 0.0)
                self.end_task_timing(success, accuracy)
            return success
        except Exception as e:
            rospy.logerr(f"Error executing {action_type}: {e}")
            error_msg = self.generate_dynamic_error_report(e, f"while executing {action_type}")
            self.speak(error_msg)
            if individual_action:
                self.end_task_timing(False, 0.0)
            return False

    def start_task_timing(self, action_type, complexity_score=1.0):
        self.performance_tracker['task_start_time'] = time.time()
        self.performance_tracker.setdefault('task_complexity_scores', []).append(complexity_score)
        self.performance_tracker.setdefault('action_history', []).append({
            'action': action_type,
            'start_time': self.performance_tracker['task_start_time'],
            'complexity': complexity_score,
            'status': 'started'
        })

    def end_task_timing(self, success=True, accuracy=None):
        if not self.performance_tracker.get('task_start_time'):
            return None
        end_time = time.time()
        duration = end_time - self.performance_tracker['task_start_time']
        self.performance_tracker['completion_times'].append(duration)
        self.performance_tracker['total_actions'] += 1 
        if success:
            self.performance_tracker['success_count'] += 1
        else:
            self.performance_tracker['error_count'] += 1
        if accuracy is not None:
            self.performance_tracker['accuracy_scores'].append(accuracy)
        if self.performance_tracker.get('action_history'):
            self.performance_tracker['action_history'][-1].update({
                'end_time': end_time,
                'duration': duration,
                'success': success,
                'accuracy': accuracy,
                'status': 'completed'
            })
        
        self.performance_tracker['task_start_time'] = None
        return {
            'duration': duration,
            'success': success,
            'accuracy': accuracy,
            'success_rate': self.performance_tracker['success_count'] / max(1, self.performance_tracker['total_actions']),
            'avg_completion_time': np.mean(self.performance_tracker['completion_times']) if self.performance_tracker['completion_times'] else duration
        }

    def calculate_action_accuracy(self, action, success, duration):
        if not success:
            return 0.0
        
        action_type = action.get('action', '')
        if 'NAVIGATE' in action_type or 'GO_TO' in action_type:
            return 0.95 if success else 0.0
        elif action_type in ['FORWARD', 'BACKWARD']:
            return 0.9 if success else 0.0
        elif 'TURN' in action_type or 'ROTATE' in action_type:
            return 0.85 if success else 0.0
        else:
            return 0.8 if success else 0.0
    
    def stop_all(self):
        self.cmd_vel_publisher.publish(Twist())
        self.move_base_client.cancel_all_goals()
        self.speak("Stopped all movement.")

    def send_image(self):
        ok = self.perception_module.send_latest_image()
        if ok:
            self.speak("Sending current view.")
        else:
            self.speak("I don't have a camera image available right now.")
        return ok

    def describe_surroundings(self):
        try:
            detections = self.perception_module.detect_objects(prob_thresh=0.1, color_thresh=0.1)

            if detections is None:
                self.speak("I can't perceive my surroundings right now.")
                return False

            if not detections:
                self.speak("No objects detected.")
                return True
            self.last_valid_detections = {}
            response = ["I can see:"]
            for det in detections:
                label = det.get('label', 'unknown').lower()
                pose = det.get('pose')
                confidence = det.get('confidence', 0.0)
                if pose:
                    self.last_valid_detections[label] = {
                        'pose': pose,
                        'confidence': confidence
                    }
                    response.append(f"{label} (confidence: {confidence*100:.0f}%)")
            self.speak("\n".join(response))
            return True

        except Exception as e:
            rospy.logerr(f"Describe failed: {e}")
            self.speak("Failed to describe surroundings.")
            return False

    def report_coordinates(self):
        try:
            if self.current_position:
                x = round(self.current_position.x, 2)
                y = round(self.current_position.y, 2)
                z = round(self.current_position.z, 2)
                
                message = f"My current coordinates are: x={x}, y={y}, z={z}"
                distance_from_origin = math.sqrt(x**2 + y**2)
                message += f". Distance from origin: {distance_from_origin:.2f} meters"
                rospy.loginfo(f"[Executor] Reporting: {message}")
                self.speak(message)
            else:
                message = "Current position data is not available. Please wait for odometry data."
                rospy.logwarn("[Executor] Position data unavailable")
                self.speak(message)
        except Exception as e:
            rospy.logerr(f"Failed to report coordinates: {e}")
            self.speak("Error reporting coordinates.")

    def report_orientation(self):
        try:
            if self.current_orientation:
                q = self.current_orientation
                euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
                roll_deg = math.degrees(euler[0])
                pitch_deg = math.degrees(euler[1])
                yaw_deg = math.degrees(euler[2])
                direction = self.get_cardinal_direction(yaw_deg)
                message = (f"My current orientation: yaw={yaw_deg:.1f}° (facing {direction}), "
                        f"roll={roll_deg:.1f}°, pitch={pitch_deg:.1f}°")
                if self.current_position:
                    message += f". Position: x={self.current_position.x:.2f}, "
                    message += f"y={self.current_position.y:.2f}, z={self.current_position.z:.2f}"
                rospy.loginfo(f"[Executor] Reporting: {message}")
                self.speak(message)
            else:
                message = "Orientation data is not available. Please wait for odometry data."
                rospy.logwarn("[Executor] Orientation data unavailable")
                self.speak(message)   
        except Exception as e:
            rospy.logerr(f"Failed to report orientation: {e}")
            self.speak("Error reporting orientation.")

    def move_linear(self, distance, speed=None):
        actual_speed = speed if speed else self.linear_speed
        direction = "forward" if distance > 0 else "backward"
        self.speak(f"Moving {direction} {abs(distance)}m")
        twist = Twist()
        twist.linear.x = actual_speed if distance > 0 else -actual_speed
        duration = abs(distance / actual_speed)
        return self._publish_twist_for_duration(twist, duration)
    
    def turn(self, angle, direction):
        self.speak(f"Turning {direction} {angle}°")
        twist = Twist()
        twist.angular.z = self.angular_speed if direction == 'left' else -self.angular_speed
        duration = math.radians(abs(angle)) / abs(self.angular_speed)
        return self._publish_twist_for_duration(twist, duration)

    def rotate(self, angle):
        direction = "left" if angle > 0 else "right"
        return self.turn(abs(angle), direction)

    def circular_motion(self, action):
        radius = action.get('radius', 1.0)
        angle = action.get('angle', 360.0)
        speed = action.get('speed', self.linear_speed)
        self.speak(f"Moving in arc: radius={radius}m, angle={angle}°")
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = speed / radius
        arc_length = (angle / 360.0) * (2 * math.pi * radius)
        duration = arc_length / speed
        return self._publish_twist_for_duration(twist, duration)

    def _publish_twist_for_duration(self, twist, duration):
        rate = rospy.Rate(self.control_rate_hz)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration:
            if self.emergency_stop:
                self.cmd_vel_publisher.publish(Twist())
                self.emergency_stop = False
                break
            self.cmd_vel_publisher.publish(twist)
            rate.sleep()
        self.cmd_vel_publisher.publish(Twist())
        return True

    def navigate_to(self, destination_name, speed=None):
        if not destination_name:
            self.speak("No destination specified.")
            return False
        if destination_name.lower() in self.last_valid_detections:
            return self.move_to_object(destination_name)
        slug, _ = self.dest_resolver.resolve(destination_name)
        if slug:
            coords = self.dest_resolver.coords[slug]
            self.speak(f"Navigating to {destination_name}")
            return self.go_to_coordinates({'x': coords['x'], 'y': coords['y'], 'z': 0}, speed)
        else:
            error_msg = self.generate_dynamic_error_report(
                f"Unknown destination: {destination_name}",
                f"Destination '{destination_name}' not found in known locations"
            )
            self.speak(error_msg)
            return False

    def go_to_coordinates(self, coordinates, speed=None):
        if not coordinates and coordinates != 0:
            self.speak("No coordinates provided.")
            return  False
        if not isinstance(coordinates, dict):
            try:
                from sil_ros.skills import normalize_coordinates
                coordinates = normalize_coordinates(coordinates)
            except Exception:
                self.speak("I couldn't understand those coordinates.")
                return False
        x = float(coordinates.get('x', 0.0))
        y = float(coordinates.get('y', 0.0))
        z = float(coordinates.get('z', 0.0))
        rospy.loginfo(f"[Executor] Navigating to coordinates: x={x}, y={y}, z={z}")
        self.speak(f"Navigating to coordinates: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        estimated_time = 0
        if self.current_position:
            dx = x - self.current_position.x
            dy = y - self.current_position.y
            distance = math.sqrt(dx**2 + dy**2)
            estimated_time = distance / 0.3  
            #if estimated_time > 5.0:
             #   self.speak(f"This will take approximately {estimated_time:.0f} seconds")
        if speed:
            self.adjust_navigation_speed(speed)
        if self.current_position:
            rospy.loginfo(f"[Executor] Pose before goal: "
                          f"x={self.current_position.x:.2f}, y={self.current_position.y:.2f}")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.goal_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        yaw = 0.0
        if self.current_position:
            yaw = math.atan2(y - self.current_position.y, x - self.current_position.x)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*quaternion)

        rospy.loginfo(f"[Executor] Sending move_base goal in frame '{self.goal_frame}': "
                      f"x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.0f}deg")
        self.move_base_client.send_goal(goal)
        rate = rospy.Rate(self.control_rate_hz)
        start_time = rospy.Time.now()
        last_progress_time = start_time
        while not rospy.is_shutdown():
            if self.emergency_stop:
                self.move_base_client.cancel_all_goals()
                self.cmd_vel_publisher.publish(Twist())
                self.emergency_stop = False
                self.speak("Navigation cancelled due to emergency stop.")
                return False
            
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()
            if (not hasattr(self, '_in_sequence_timing') and 
                elapsed > 5.0 and 
                (current_time - last_progress_time).to_sec() >= self.progress_update_interval):
                if self.current_position:
                    current_dist = math.sqrt((x - self.current_position.x)**2 + (y - self.current_position.y)**2)
                    progress_msg = f"Navigation progress: {current_dist:.1f}m remaining after {elapsed:.0f}s"
                else:
                    progress_msg = f"Navigation in progress: {elapsed:.0f}s elapsed"
                self.speak(progress_msg)
                last_progress_time = current_time
            
            state = self.move_base_client.get_state()
            if state in [actionlib.GoalStatus.SUCCEEDED,
                        actionlib.GoalStatus.ABORTED,
                        actionlib.GoalStatus.REJECTED,
                        actionlib.GoalStatus.PREEMPTED]:
                break
            if self.navigation_timeout > 0 and \
                    (rospy.Time.now() - start_time).to_sec() > self.navigation_timeout:
                rospy.logwarn(f"[Executor] Navigation timed out after "
                              f"{self.navigation_timeout}s; cancelling goal.")
                self.move_base_client.cancel_all_goals()
                break
            rate.sleep()
        if speed:
            self.reset_navigation_speed()
        final_state = self.move_base_client.get_state()
        state_name = {
            actionlib.GoalStatus.SUCCEEDED: "SUCCEEDED",
            actionlib.GoalStatus.ABORTED: "ABORTED",
            actionlib.GoalStatus.REJECTED: "REJECTED",
            actionlib.GoalStatus.PREEMPTED: "PREEMPTED",
            actionlib.GoalStatus.ACTIVE: "ACTIVE",
            actionlib.GoalStatus.PENDING: "PENDING",
        }.get(final_state, str(final_state))
        if self.current_position:
            rospy.loginfo(f"[Executor] move_base result: {state_name}; "
                          f"pose after: x={self.current_position.x:.2f}, "
                          f"y={self.current_position.y:.2f}")
        else:
            rospy.loginfo(f"[Executor] move_base result: {state_name}")

        if final_state == actionlib.GoalStatus.SUCCEEDED:
            self.speak(f"Successfully arrived at coordinates: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            return True
        else:
            error_msg = self.generate_dynamic_error_report(
                f"Navigation ended with state {state_name}",
                f"while navigating to coordinates ({x:.2f}, {y:.2f})"
            )
            self.speak(error_msg)
            return False

    def move_to_object(self, object_name):
        object_name = object_name.lower()
        try:
            dets = self.perception_module.detect_objects(queries=[object_name])
            for det in (dets or []):
                lbl = str(det.get('label', '')).lower()
                if det.get('pose') is not None:
                    self.last_valid_detections[lbl] = {
                        'pose': det['pose'],
                        'confidence': det.get('confidence', 0.0),
                    }
        except Exception as e:
            rospy.logwarn(f"[Executor] Object re-detection failed: {e}")
        if object_name in self.last_valid_detections:
            obj_data = self.last_valid_detections[object_name]
            pose = obj_data['pose']
            self.speak(f"Moving to {object_name}")
            if self.move_to_pose(pose):
                self.speak(f"Arrived at {object_name}")
                return True
            else:
                error_msg = self.generate_dynamic_error_report(
                    f"Failed to reach {object_name}",
                    f"navigation to detected object failed"
                )
                self.speak(error_msg)
                return False
        else:
            error_msg = self.generate_dynamic_error_report(
                f"Cannot locate {object_name}",
                "object not currently visible or detected"
            )
            self.speak(error_msg)
            return False

    def move_to_pose(self, pose_stamped):
        if not isinstance(pose_stamped, PoseStamped):
            return False
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        self.move_base_client.send_goal(goal)
        rate = rospy.Rate(self.control_rate_hz)
        while not rospy.is_shutdown():
            if self.emergency_stop:
                self.move_base_client.cancel_all_goals()
                return False
            state = self.move_base_client.get_state()
            if state in [actionlib.GoalStatus.SUCCEEDED,
                        actionlib.GoalStatus.ABORTED,
                        actionlib.GoalStatus.REJECTED]:
                break
            rate.sleep()
        return self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED

    def adjust_navigation_speed(self, speed):
        if self.dyn_client is None:
            rospy.logwarn("[Executor] No dynamic-reconfigure client; cannot adjust speed.")
            return
        try:
            params = {
                self.planner_speed_param: speed,
                'min_vel_x': -speed,
                'max_vel_trans': speed,
                'min_vel_trans': self.min_speed,
            }
            self.dyn_client.update_configuration(params)
        except Exception as e:
            rospy.logerr(f"Failed to adjust speed: {e}")

    def reset_navigation_speed(self):
        self.adjust_navigation_speed(self.linear_speed)

    def get_cardinal_direction(self, yaw_degrees):
        yaw = yaw_degrees % 360
        if yaw < 0:
            yaw += 360
        directions = [
            (337.5, 22.5, "north"),
            (22.5, 67.5, "northeast"),
            (67.5, 112.5, "east"),
            (112.5, 157.5, "southeast"),
            (157.5, 202.5, "south"),
            (202.5, 247.5, "southwest"),
            (247.5, 292.5, "west"),
            (292.5, 337.5, "northwest")
        ]   
        for start, end, direction in directions:
            if start > end: 
                if yaw >= start or yaw < end:
                    return direction
            elif start <= yaw < end:
                return direction
        
        return "north" 

    def get_destinations(self):
        mapping = {}
        for slug, info in self.dest_resolver.raw.items():
            display = info.get("display_name", slug.replace('_', ' ').title())
            mapping[display] = slug
        return mapping
