#!/usr/bin/env python3
import threading
import time
from functools import reduce, wraps
from typing import Any, Callable, Dict, Generator, List, Optional

import rospy
import yaml
from abb_egm_msgs.msg import EGMState
from abb_rapid_sm_addin_msgs.msg import RuntimeState, StateMachineState
from abb_rapid_sm_addin_msgs.srv import GetEGMSettings, SetEGMSettings
from abb_robot_msgs.msg import ServiceResponses, SystemState
from abb_robot_msgs.srv import TriggerWithResultCode
from controller_manager_msgs.srv import SwitchController


class ABBServiceFailed(Exception):
    """
    The ABB RWS/EGM service call has failled

    The status code and message field provides more information on why the service failed. These values are provided
    by the service.
    """

    def __init__(self, service_name, status_code, message) -> None:
        self.status_code = status_code
        self.message = message
        super().__init__(f"Service [{service_name}] failed with status code: {status_code}, message: {message}")


class ABBServiceTimeout(Exception):
    """The ABB RWS/EGM service call did not lead to the requested change within the timeout period"""

    def __init__(self, service_name, timeout) -> None:
        self.service_name = service_name
        self.timeout = timeout
        super().__init__(f"Service [{service_name}] did not lead to the expected change, waited {timeout} seconds")


class ABBRobotDriverManager:
    """
    Utility class to help manage the `abb_robot_driver` interfaces

    This class provides convenience function to trigger service calls and check if the service has been processed by
    monitoring the appropriate topics. The services called via this class also have fault checking, throwing
    ABBServiceFailed exceptions if the status is not `OK`.

    Note that this class is not aware of the robot controller being in the correct state to process your request. A
    state machine implementation (like provided in this package) should be responsible for ensuring calls can be
    executed. Calling a service in an incorrect state, most likely results in a ABBServiceFailed exception.
    """

    # Default settings
    TASKNAME = "T_ROB1"
    RWS_NAMESPACE = "/rws"
    EGM_NAMEPSACE = "/egm"

    def __init__(self) -> None:
        # Retrieve ROS parameters
        self.rws_namespace = rospy.get_param("rws_namespace", default=self.RWS_NAMESPACE)
        self.egm_namespace = rospy.get_param("egm_namespace", self.EGM_NAMEPSACE)
        self.taskname = rospy.get_param("taskname", default=self.TASKNAME)

    def initialize(self) -> None:
        """Creates service handles for the ABB/EGM services and reads the egm settings file if provided"""

        # Set up services
        self._create_services(timeout=5)

        # Load settings from YAML
        filename = rospy.get_param("egm_settings", "")
        if filename:
            with open(filename, mode="r") as file_:
                try:
                    self.egm_settings_dict = yaml.safe_load(file_)
                except yaml.YAMLError as error:
                    raise ValueError(f"Unable to load settings from '{filename}': {error}")

        # List of active controllers
        self.active_controllers = [""]

        # Clean-up on exit
        # atexit.register(self.exit) # __del__() does not work

    def set_motors_on(self, timeout: float = 5) -> None:
        """Turn on the motors of the ABB manipulator (blocking untill motors are on)"""

        if not self.motors_on:
            self.set_motors_on_srv()
            monitor = MonitorTopic(
                topic=self.rws_namespace + "/system_states",
                msg_type=SystemState,
                condition=lambda msg: msg.motors_on,
            )
            monitor.wait(timeout=timeout)

            rospy.logdebug("Motors turned on succesfully")
        else:
            rospy.logdebug("Motors already on, ignoring request..")

    def set_motors_off(self, timeout: float = 5) -> None:
        """Turn off the motors of the ABB manipulator (blocking untill motors are off)"""

        if self.motors_on:
            self.set_motors_off_srv()
            monitor = MonitorTopic(
                topic=self.rws_namespace + "/system_states",
                msg_type=SystemState,
                condition=lambda msg: not msg.motors_on,
            )
            monitor.wait(timeout=timeout)

            rospy.logdebug("Motors turned off succesfully")
        else:
            rospy.logdebug("Motors already off, ignoring request..")

    def start_rapid_motion_task(self, timeout: float = 5) -> None:
        """
        Start the main RAPID motion task (blocking untill RAPID is reported as running)

        Equivalent to pressing the 'play' button on the teach pendant
        """

        # Restart if running already
        self.stop_rapid_motion_task(timeout)

        # Start RAPID
        self.start_rapid_srv()
        monitor = MonitorTopic(
            topic=self.rws_namespace + "/system_states",
            msg_type=SystemState,
            condition=lambda msg: msg.rapid_running,
        )
        monitor.wait(timeout=timeout)
        rospy.logdebug("RAPID started")

    def stop_rapid_motion_task(self, timeout: float = 5) -> None:
        """
        Stop the main RAPID motion task (blocking untill RAPID is reported as not running)

        Equivalent to pressing the 'stop' button on the teach pendant followed by pp to main
        """
        if self.rapid_running:
            try:
                self.stop_rapid_srv()
            except ABBServiceFailed as error:
                if error.status_code != ServiceResponses.RC_RAPID_NOT_RUNNING:
                    raise

            monitor = MonitorTopic(
                topic=self.rws_namespace + "/system_states",
                msg_type=SystemState,
                condition=lambda msg: not msg.rapid_running,
            )
            monitor.wait(timeout=timeout)
            rospy.logdebug("RAPID stopped")
        else:
            rospy.logdebug("RAPID already stopped, ignoring request..")

        # Restart program
        self.pp_to_main_srv()

    def activate_egm_session(self, timeout: float = 5) -> None:
        """Activate the EGM session"""
        self.start_egm_joint_srv()
        monitor = MonitorTopic(
            topic=self.egm_namespace + "/egm_states",
            msg_type=EGMState,
            condition=lambda msg: msg.egm_channels[0].active,
        )
        monitor.wait(timeout=timeout)

        # Workaround for "EGM is active, but not in running mode (i.e. listening for command references)!" error
        time.sleep(1.0)
        rospy.logdebug("EGM session activated")

    def deactivate_egm_session(self) -> None:
        """Deactivate the EGM session"""
        self.stop_egm_srv()
        rospy.logdebug("EGM session stopped")

    def monitor_egm_active(self) -> None:
        """
        Utility function to monitor if the EGM session is still active (blocking)

        The function blocks until the EGM session is no longer reported as active. This function is implemented for use
        with smach_ros.MonitorState.
        """

        monitor = MonitorTopic(
            topic=self.egm_namespace + "/egm_states",
            msg_type=EGMState,
            condition=lambda msg: not msg.egm_channels[0].active,
        )
        monitor.wait()

    def _switch_controller(self, start_controllers: List[str] = [""], stop_controllers: List[str] = [""]) -> None:
        """Switch between ROS controllers"""

        # Call service and update list of active controllers
        result = self.switch_controller_srv(
            start_controllers=start_controllers,
            stop_controllers=stop_controllers,
            strictness=1,
            start_asap=False,
            timeout=0.0,
        )
        if result.ok:
            self.active_controllers = start_controllers
        else:
            raise RuntimeError("Unable to swich controllers")

    def start_velocity_controller(self) -> None:
        """Start the 'joint_group_velocity_controller' controller"""
        self._switch_controller(
            start_controllers=["joint_group_velocity_controller"], stop_controllers=self.active_controllers
        )
        rospy.loginfo("Velocity controller activated")

    def start_position_controller(self) -> None:
        """Start the 'joint_position_trajectory_controller' controller"""
        self._switch_controller(
            start_controllers=["joint_position_trajectory_controller"], stop_controllers=self.active_controllers
        )
        rospy.loginfo("Position controller activated")

    def stop_controllers(self) -> None:
        """Stop all active controllers"""
        self._switch_controller(stop_controllers=self.active_controllers)
        rospy.loginfo("Stopped all active controllers")

    def load_settings(self, settings_dict: Dict, verbose: bool = False) -> None:
        """
        Load the EGM settings from the settings dictionary (which can be loaded from a file)

        Missing settings are taken from the current settings on the ABB controller.
        """

        # Retrieve current setting
        settings = self.get_egm_settings_srv(task=self.taskname).settings

        # Write the loaded settings to the robot
        for setting_key, value in flatten(settings_dict):
            set_attribute(settings, setting_key, value)

        if verbose:
            rospy.loginfo(f"Loaded settings:\n{settings}")

        return settings

    def write_settings(self, timeout: float = 5) -> None:
        """
        Write the EGM settings to the ABB controller

        The egm settings need to be stored in as a class atribute named 'egm_settings_dict'
        """
        if not hasattr(self, "egm_settings_dict"):
            return

        def _condition(msg):
            """Check if the state machine is initialized (and therefore idle)"""
            return msg.state_machines[0].sm_state == StateMachineState.SM_STATE_IDLE

        monitor = MonitorTopic(self.rws_namespace + "/sm_addin/runtime_states", RuntimeState, _condition)
        monitor.wait(timeout=timeout)

        # Write the settings to the controller
        settings = self.load_settings(self.egm_settings_dict)
        self.set_egm_settings_srv(task=self.taskname, settings=settings)

    @property
    def auto_mode(self) -> bool:
        """Check if auto mode is active"""
        system_states = rospy.wait_for_message(self.rws_namespace + "/system_states", SystemState, timeout=5)
        return system_states.auto_mode

    @property
    def motors_on(self) -> bool:
        """Check if the motors are on"""
        system_states = rospy.wait_for_message(self.rws_namespace + "/system_states", SystemState, timeout=5)
        return system_states.motors_on

    @property
    def rapid_running(self) -> bool:
        """Check if a RAPID program is running"""
        system_states = rospy.wait_for_message(self.rws_namespace + "/system_states", SystemState, timeout=5)
        return system_states.rapid_running

    def _create_services(self, timeout=5):
        """Create the service handles for all services used"""

        rospy.wait_for_service(self.rws_namespace + "/set_motors_on", timeout)
        self.set_motors_on_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/set_motors_on", TriggerWithResultCode),
            "set_motors_on",
        )
        rospy.wait_for_service(self.rws_namespace + "/set_motors_off", timeout)
        self.set_motors_off_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/set_motors_off", TriggerWithResultCode),
            "set_motors_off",
        )

        rospy.wait_for_service(self.rws_namespace + "/sm_addin/get_egm_settings", timeout)
        self.get_egm_settings_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/sm_addin/get_egm_settings", GetEGMSettings),
            "get_egm_settings",
        )
        rospy.wait_for_service(self.rws_namespace + "/sm_addin/set_egm_settings", timeout)
        self.set_egm_settings_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/sm_addin/set_egm_settings", SetEGMSettings),
            "set_egm_settings",
        )
        rospy.wait_for_service(self.rws_namespace + "/start_rapid", timeout)
        self.start_rapid_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/start_rapid", TriggerWithResultCode),
            "start_rapid",
        )
        rospy.wait_for_service(self.rws_namespace + "/stop_rapid", timeout)
        self.stop_rapid_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/stop_rapid", TriggerWithResultCode),
            "stop_rapid",
        )
        rospy.wait_for_service(self.rws_namespace + "/pp_to_main", timeout)
        self.pp_to_main_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/pp_to_main", TriggerWithResultCode),
            "pp_to_main",
        )
        rospy.wait_for_service(self.rws_namespace + "/sm_addin/start_egm_joint", timeout)
        self.start_egm_joint_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/sm_addin/start_egm_joint", TriggerWithResultCode),
            "start_egm_joint",
        )
        rospy.wait_for_service(self.rws_namespace + "/sm_addin/stop_egm", timeout)
        self.stop_egm_srv = self._add_srv_check(
            rospy.ServiceProxy(self.rws_namespace + "/sm_addin/stop_egm", TriggerWithResultCode),
            "stop_egm",
        )
        rospy.wait_for_service(self.egm_namespace + "/controller_manager/switch_controller", timeout)
        self.switch_controller_srv = rospy.ServiceProxy(
            self.egm_namespace + "/controller_manager/switch_controller", SwitchController
        )
        rospy.loginfo("All services found")

    def _add_srv_check(self, service_handle: Callable[..., Any], name: str) -> Callable[..., Any]:
        """
        Decorator to automatically check if the ABB service succeeded when calling
        a service.

        Raises
        ------
        ABBServiceFailed
            If the result from the service call does not have status code
            abb_robot_msgs/ServiceResponses.RC_SUCCESS
        """

        @wraps(service_handle)
        def wrapper(*args, **kwargs):
            """Check if service is succesful and raise ABBServiceFailed otherwise"""
            result = service_handle(*args, **kwargs)
            rospy.logdebug(f"Service [{name}] result status code: {result.result_code}, message: {result.message}")
            if result.result_code != ServiceResponses.RC_SUCCESS:
                raise ABBServiceFailed(name, result.result_code, result.message)
            return result

        return wrapper


class MonitorTopic:
    """Monitors a topic until the condition is met and returns true"""

    def __init__(self, topic: str, msg_type: Any, condition: Callable[[Any], bool]) -> None:
        self.topic = topic
        self.msg_type = msg_type
        self.condition = condition
        self._trigger = threading.Event()

    def wait(self, timeout: Optional[float] = None) -> None:
        """Wait till the provided condition returns true, or the timeout is reached"""
        # Create subscriber
        self._trigger.clear()
        sub = rospy.Subscriber(self.topic, self.msg_type, self._cb)

        # Wait till until triggered (or timeout)
        succesful = self._trigger.wait(timeout=timeout)

        # Raise exception on timeout
        if not succesful:
            raise ABBServiceTimeout(self.topic, timeout)

    def _cb(self, msg: Any):
        if self.condition(msg) or rospy.is_shutdown():
            self._trigger.set()


def get_attribute(obj: object, attr: str) -> Any:
    """
    Get an attribute of a nested object using a dotted notation.

    Parameters
    ----------
    obj : object
        The parent object of which the (nested) attribute is retrieved.
    attribute : str
        The attribute string, e.g. 'a.b', of the attribute to retrieve.
    """
    return reduce(getattr, [obj] + attr.split("."))


def set_attribute(obj: object, attr: str, val) -> None:
    """
    Set an attribute of a nested object using a dotted notation.

    Parameters
    ----------
    obj : object
        The parent object of which the (nested) attribute is set.
    attr : str
        The attribute string, e.g. 'a.b', of the attribute to set.
    val : Any
        The requested value of the attribute.
    """
    attr_prefix_, _, attr_ = attr.rpartition(".")
    return setattr(get_attribute(obj, attr_prefix_) if attr_prefix_ else obj, attr_, val)


def flatten(dictionary: dict, separator: str = ".", _prefix: str = "") -> Generator:
    """
    Generator to flatten a dictionary by combining the key names of nested items.

    Parameters
    ----------
    dictionary : dict
        The dictionary object to flatten
    separator : str
        The seperator to use when resolving the flattened key of nested dictionaries

    Yields
    ----------
    key : str
        The resolved key for the flattened dictionary item
    value : Any
        The value corresponding to the flattened dictionary item

    Returns:
    ----------
    Generator : [str, Any]
        Generator which returns the flattened key, value tuples
    """

    # Loop over the items in the dictionary
    for key, value in dictionary.items():

        # Get the full attribute name by combining the prefix and key strings
        full_key = _prefix + separator + key if _prefix else key

        if type(value) is dict:
            # Recursively call the function again if the value is a dict (it is nested).
            yield from flatten(value, separator, _prefix=full_key)
        else:
            # Return the flattened key, value tuple
            yield (full_key, value)
