#!/usr/bin/env python3
import signal
from typing import Any

import rospy
import smach
import smach_ros
from abb_egm_msgs.msg import EGMState
from abb_robot_msgs.msg import SystemState
from rospy import ROSException

from .core import ABBRobotDriverManager, ABBServiceFailed, ABBServiceTimeout


class ABBRobotDriverManagerSM(ABBRobotDriverManager):
    """
    State Machine implementation for use with the ABB Robot Driver using EGM.

    This state machine manages the robot controller state using the abb_robot_driver interfaces, simplifying the use
    of the driver for motion applications like MoveIt.
    """

    class InitializeState(smach.State):
        """
        Initial state of the state machine

        Initialize the state machine and ABBRobotDriverManager. After initialization, this state will request to turn
        on the motors and transition to the MotorsOnState if successful.
        """

        outcomes = ["motors_on", "exit"]

        def __init__(self) -> None:
            super().__init__(outcomes=self.outcomes, io_keys=["abb_robot_driver_manager"])

        def execute(self, userdata: Any) -> str:
            # Initialize abb_robot_driver_manager
            try:
                userdata.abb_robot_driver_manager.initialize()
                if not userdata.abb_robot_driver_manager.auto_mode:
                    rospy.logerr(f"Controller not in auto mode, aborting..")
                    return "exit"
            except ROSException as exception:
                rospy.logerr(f"Initialization failed! Error: {exception}")
                return "exit"

            # Turn on motors to transition to MotorsOn state
            attempts = 2 # Attmept twice for ABBServiceFailed, as motors sometimes fail to turn on after inactivity.
            for i in range(attempts):
                try:
                    userdata.abb_robot_driver_manager.set_motors_on()
                    break
                except ABBServiceTimeout as error:
                    rospy.logerr(f"Unable to start motors. Error: {error}")
                    return "exit"
                except ABBServiceFailed as error:
                    rospy.logwarn(f"Unable to start motors. Attempt ({i+1}/{attempts}). Error: {error}")
            else:
                rospy.logerr(f"Unable to start motors after {attempts} attempts.")
                return "exit"
            return "motors_on"

    class MotorsOnState(smach.State):
        """
        The motors of the manipulator are on

        This state will request to start the main RAPID program and transition to the IdleState
        """

        outcomes = ["rapid_motion_task_started", "exit"]

        def __init__(self) -> None:
            super().__init__(outcomes=self.outcomes, io_keys=["abb_robot_driver_manager"])

        def execute(self, userdata: Any) -> str:
            # Transition to Idle state
            try:
                userdata.abb_robot_driver_manager.start_rapid_motion_task()
            except (ABBServiceFailed, ABBServiceTimeout):
                rospy.logerr("Unable to start RAPID motion task (main program)")
                return "exit"
            return "rapid_motion_task_started"

    class MonitorMotorsOnState(smach_ros.MonitorState):
        """
        Monitor if the motors are still reported as on.

        This is to ensure the state machine and robot controller states are in sync.
        """

        outcomes = ["motors_off"]

        def __init__(self, topic) -> None:
            super().__init__(topic, SystemState, self._condition)

        def execute(self, userdata: Any) -> str:
            outcome = super().execute(userdata)
            if outcome not in ["invalid", "preempted"]:
                raise RuntimeError(f"MonitorRapidTaskState returned an unexpected outcome: {outcome}")
            return "aborted"

        @staticmethod
        def _condition(userdata: Any, msg: SystemState):
            return msg.motors_on

    class IdleState(smach.State):
        """
        The state machine is ready to start a EGM session or RAPID routine

        Automatically tries to transitions to the EGMActice state and load the position controller.
        TODO: Add an external trigger to advance to the next state
        """

        outcomes = ["egm_motion_started", "rapid_routine_started", "exit"]

        def __init__(self) -> None:
            super().__init__(outcomes=self.outcomes, io_keys=["abb_robot_driver_manager"])

        def execute(self, userdata: Any) -> str:
            # Transition to RunEgmMotion state
            try:
                userdata.abb_robot_driver_manager.write_settings()
                userdata.abb_robot_driver_manager.activate_egm_session()
                userdata.abb_robot_driver_manager.start_position_controller()
            except (RuntimeError, ABBServiceFailed, ABBServiceTimeout) as error:
                rospy.logerr(f"{error}")
                return "exit"
            return "egm_motion_started"

    class MonitorRapidTaskState(smach_ros.MonitorState):
        """
        Monitor if the main RAPID programm is still reported as running.

        This is to ensure the state machine and robot controller states are in sync.
        """

        outcomes = ["rapid_routine_stopped"]

        def __init__(self, topic) -> None:
            super().__init__(topic, SystemState, self._condition)

        def execute(self, userdata: Any) -> str:
            outcome = super().execute(userdata)
            if outcome not in ["invalid", "preempted"]:
                raise RuntimeError(f"MonitorRapidTaskState returned an unexpected value: {outcome}")
            return "aborted"

        @staticmethod
        def _condition(userdata: Any, msg: SystemState) -> bool:
            return msg.rapid_running

    class EgmSessionActiveState(smach_ros.MonitorState):
        """
        The EGM session is active and the robot is able to accept motion requests
        """

        def __init__(self, topic: str) -> None:
            super().__init__(topic, EGMState, self._condition)

        @staticmethod
        def _condition(userdata: Any, msg: EGMState) -> bool:
            """Check if EGM session is active"""
            active = msg.egm_channels[0].active
            egm_running = msg.egm_channels[0].egm_client_state == 4  # EGM_RUNNING = 4
            motors_on = msg.egm_channels[0].motor_state == 2  # MOTORS_ON = 2
            rapid_running = msg.egm_channels[0].rapid_execution_state == 3  # RAPID_RUNNING = 3

            if not active:
                rospy.logdebug(f"EGM Active: {active}")
            if not egm_running:
                rospy.logdebug(f"EGM Running: {egm_running}")
            if not motors_on:
                rospy.logdebug(f"Motors On: {motors_on}")
            if not rapid_running:
                rospy.logdebug(f"RAPID Running: {rapid_running}")
            return active  # and egm_running and motors_on and rapid_running

    class RapidRoutineActiveState(smach.State):
        """A RAPID routine is active"""

        outcomes = ["rapid_routine_finished"]

        def __init__(self) -> None:
            super().__init__(outcomes=self.outcomes, io_keys=["abb_robot_driver_manager"])

        def execute(self, userdata: Any) -> str:
            rospy.logerr("NotImplementedError: Returning to Idle")
            return "rapid_routine_finished"

    class ExitState(smach.State):
        """
        The state machine is shutting down

        This state will try to stop the RAPID program and turn off the motors.
        """

        outcomes = ["shutdown"]

        def __init__(self) -> None:
            super().__init__(outcomes=self.outcomes, io_keys=["abb_robot_driver_manager"])

        def execute(self, userdata: Any) -> str:
            try:
                userdata.abb_robot_driver_manager.stop_rapid_motion_task()
            except ABBServiceFailed as error:
                # Only a warning as we are already shutting down, this can be triggered by an emergency stop
                rospy.logwarn(f"{error}")
            userdata.abb_robot_driver_manager.set_motors_off()
            rospy.signal_shutdown("State machine exited")
            return "shutdown"

    def __init__(self) -> None:
        super().__init__()

        # Initialize node and state machine
        rospy.init_node("abb_robot_driver_state_machine", disable_signals=True, log_level=rospy.DEBUG)
        self._create_state_machine()

        # Register our custom Ctrl-C (SIGINT) behaviour to cleanly shutdown the state machine
        signal.signal(signal.SIGINT, self._preempt)

    def execute(self) -> None:
        """Run the main state machine"""
        self.state_machine.execute()

    def _create_state_machine(self) -> None:
        """Define the complete state machine and its transitions"""

        # Create state machine interface
        self.state_machine = smach.StateMachine(outcomes=["shutdown"])
        self.state_machine.userdata.abb_robot_driver_manager = self
        with self.state_machine:
            smach.StateMachine.add(
                label="Initialize",
                state=self.InitializeState(),
                transitions={
                    "motors_on": "MotorsOnCC",
                    "exit": "Exit",
                },
            )

            # Concurrence containter (cc_)
            # Run the inner state machine (MotorsOn) and MonitorMotorsOnState in parallel
            self.cc_motors_on = smach.Concurrence(
                outcomes=["exit", "aborted"],
                default_outcome="aborted",
                child_termination_cb=lambda _: True,
            )
            self.cc_motors_on.userdata.abb_robot_driver_manager = self
            with self.cc_motors_on:
                self.sub_sm_motors_on = smach.StateMachine(outcomes=["exit", "aborted"])
                self.sub_sm_motors_on.userdata.abb_robot_driver_manager = self
                with self.sub_sm_motors_on:
                    smach.StateMachine.add(
                        label="MotorsOn",
                        state=self.MotorsOnState(),
                        transitions={
                            "rapid_motion_task_started": "RapidTaskCC",
                            "exit": "exit",
                        },
                    )

                    # Concurrence containter (cc_)
                    # Run the inner state machine (Idle) and MonitorRapidTaskState in parallel
                    self.cc_rapid_task = smach.Concurrence(
                        outcomes=["exit", "aborted"],
                        default_outcome="aborted",
                        child_termination_cb=lambda _: True,
                    )
                    self.cc_rapid_task.userdata.abb_robot_driver_manager = self
                    with self.cc_rapid_task:
                        self.sub_sm_rapid_task = smach.StateMachine(outcomes=["exit"])
                        self.sub_sm_rapid_task.userdata.abb_robot_driver_manager = self
                        with self.sub_sm_rapid_task:
                            smach.StateMachine.add(
                                label="Idle",
                                state=self.IdleState(),
                                transitions={
                                    "egm_motion_started": "EgmSessionActive",
                                    "rapid_routine_started": "RapidRoutineActive",
                                    "exit": "exit",
                                },
                            )

                            smach.StateMachine.add(
                                label="EgmSessionActive",
                                state=self.EgmSessionActiveState(self.egm_namespace + "/egm_states"),
                                transitions={
                                    "invalid": "Idle",
                                    "valid": "EgmSessionActive",
                                    "preempted": "exit",
                                },
                            )

                            smach.StateMachine.add(
                                label="RapidRoutineActive",
                                state=self.RapidRoutineActiveState(),
                                transitions={
                                    "rapid_routine_finished": "Idle",
                                },
                            )

                        monitor_rapid_task_state = self.MonitorRapidTaskState(self.rws_namespace + "/system_states")
                        smach.Concurrence.add("MonitorRapidTask", monitor_rapid_task_state)
                        smach.Concurrence.add("RapidTaskSM", self.sub_sm_rapid_task)

                    smach.StateMachine.add(
                        "RapidTaskCC",
                        self.cc_rapid_task,
                        transitions={
                            "exit": "exit",
                            "aborted": "aborted",
                        },
                    )

                monitor_motors_on_state = self.MonitorMotorsOnState(self.rws_namespace + "/system_states")
                smach.Concurrence.add("MonitorMotorsOn", monitor_motors_on_state)
                smach.Concurrence.add("MotorsOnSM", self.sub_sm_motors_on)

            smach.StateMachine.add(
                label="MotorsOnCC",
                state=self.cc_motors_on,
                transitions={
                    "exit": "Exit",
                    "aborted": "Exit",
                },
            )

            smach.StateMachine.add(
                label="Exit",
                state=self.ExitState(),
                transitions={
                    "shutdown": "shutdown",
                },
            )

    def _preempt(self, *_):
        """Preempt the state machine"""
        self.cc_motors_on.request_preempt()
