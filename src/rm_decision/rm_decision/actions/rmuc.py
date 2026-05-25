import math
from typing import Dict, List, Optional

import py_trees
from py_trees.common import Status
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from ..registry import register


@register("RegionPatrolAction")
class RegionPatrolAction(py_trees.behaviour.Behaviour):
	def __init__(
		self,
		name: str,
		node: Node,
		server_name: str = "navigate_to_pose",
		frame_id: str = "map",
		region_param: str = "patrol_region",
		regions: Optional[Dict] = None,
		default_region: Optional[int] = None,
		dwell_s: float = 2.0,
		timeout_s: Optional[float] = None,
		cancel_on_terminate: bool = True,
		publish_goal_topic: str = "/goal_pose",
	):
		super().__init__(name)
		self.node = node
		self.client = ActionClient(node, NavigateToPose, server_name)
		self.frame_id = frame_id
		self.region_param = region_param.lstrip("/")
		self.regions = self._normalise_regions(regions or {})
		self.default_region = default_region
		self.dwell_s = float(dwell_s)
		self.timeout_s = timeout_s
		self.cancel_on_terminate = bool(cancel_on_terminate)
		self.goal_pub = (
			self.node.create_publisher(PoseStamped, publish_goal_topic, 10)
			if publish_goal_topic
			else None
		)
		self._goal_handle = None
		self._result_future = None
		self._goal_rejected = False
		self._sent = False
		self._active_region = None
		self._waypoint_index = 0
		self._start_time = None
		self._dwell_start = None

	def setup(self, **kwargs) -> None:
		timeout_sec = float(kwargs.get("timeout", 3.0)) if "timeout" in kwargs else 3.0
		if not self.node.has_parameter(self.region_param):
			self.node.declare_parameter(self.region_param, 0)
		if not self.client.wait_for_server(timeout_sec=timeout_sec):
			raise RuntimeError("Nav2 NavigateToPose action server not available")

	def initialise(self) -> None:
		self._reset_goal_state()

	def update(self) -> Status:
		region_id = self._read_region_id()
		waypoints = self._waypoints_for_region(region_id)
		if not waypoints:
			self.node.get_logger().warn(f"RegionPatrolAction: no waypoints for patrol_region={region_id}")
			return Status.FAILURE

		if self._active_region != region_id:
			self._cancel_goal()
			self._active_region = region_id
			self._waypoint_index = 0
			self._reset_goal_state()

		if self._dwell_start is not None:
			elapsed = (self.node.get_clock().now() - self._dwell_start).nanoseconds / 1e9
			if elapsed < self.dwell_s:
				return Status.RUNNING
			self._dwell_start = None
			self._waypoint_index = (self._waypoint_index + 1) % len(waypoints)
			self._reset_goal_state()

		if not self._sent:
			self._send_goal(waypoints[self._waypoint_index])
			return Status.RUNNING

		if self._goal_rejected:
			self.node.get_logger().warn("RegionPatrolAction: goal rejected, trying next waypoint")
			self._waypoint_index = (self._waypoint_index + 1) % len(waypoints)
			self._reset_goal_state()
			return Status.RUNNING

		if self.timeout_s is not None:
			elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
			if elapsed > self.timeout_s:
				self.node.get_logger().warn("RegionPatrolAction: goal timeout, trying next waypoint")
				self._cancel_goal()
				self._waypoint_index = (self._waypoint_index + 1) % len(waypoints)
				self._reset_goal_state()
				return Status.RUNNING

		if self._result_future is not None and self._result_future.done():
			result_stub = self._result_future.result()
			status_code = getattr(result_stub, "status", None)
			if status_code is None and self._goal_handle is not None:
				status_code = getattr(self._goal_handle, "status", None)
			if status_code == 4:
				self._dwell_start = self.node.get_clock().now()
			else:
				self.node.get_logger().warn(
					f"RegionPatrolAction: goal status={status_code}, trying next waypoint"
				)
				self._waypoint_index = (self._waypoint_index + 1) % len(waypoints)
			self._reset_goal_state(keep_dwell=True)

		return Status.RUNNING

	def terminate(self, new_status: Status) -> None:
		if new_status == Status.INVALID and self.cancel_on_terminate:
			self._cancel_goal()
			self._reset_goal_state()

	def _read_region_id(self) -> int:
		try:
			return int(self.node.get_parameter(self.region_param).value)
		except Exception:
			return int(self.default_region or 0)

	def _waypoints_for_region(self, region_id: int) -> List[dict]:
		if region_id in self.regions:
			return self.regions[region_id]
		if self.default_region is not None and self.default_region in self.regions:
			return self.regions[self.default_region]
		return []

	def _send_goal(self, waypoint: dict) -> None:
		pose = self._build_pose(waypoint)
		goal = NavigateToPose.Goal()
		goal.pose = pose
		if self.goal_pub is not None:
			self.goal_pub.publish(pose)
		send_future = self.client.send_goal_async(goal)
		send_future.add_done_callback(self._on_goal_response)
		self._sent = True
		self._start_time = self.node.get_clock().now()
		self.node.get_logger().info(
			f"RegionPatrolAction: region={self._active_region}, waypoint={self._waypoint_index}, "
			f"goal=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
		)

	def _build_pose(self, waypoint: dict) -> PoseStamped:
		pose = PoseStamped()
		pose.header.frame_id = self.frame_id
		pose.header.stamp = self.node.get_clock().now().to_msg()
		pose.pose.position.x = float(waypoint.get("x", 0.0))
		pose.pose.position.y = float(waypoint.get("y", 0.0))
		yaw = float(waypoint.get("yaw", 0.0))
		pose.pose.orientation.z = math.sin(yaw * 0.5)
		pose.pose.orientation.w = math.cos(yaw * 0.5)
		return pose

	def _cancel_goal(self) -> None:
		if self._goal_handle is not None:
			try:
				self._goal_handle.cancel_goal_async()
			except Exception as exc:
				self.node.get_logger().warn(f"RegionPatrolAction: cancel goal failed: {exc}")

	def _reset_goal_state(self, keep_dwell: bool = False) -> None:
		self._goal_handle = None
		self._result_future = None
		self._goal_rejected = False
		self._sent = False
		self._start_time = None
		if not keep_dwell:
			self._dwell_start = None

	def _on_goal_response(self, future):
		self._goal_handle = future.result()
		if not getattr(self._goal_handle, "accepted", False):
			self._result_future = None
			self._goal_rejected = True
			self.node.get_logger().warn("RegionPatrolAction: goal rejected")
		else:
			self._result_future = self._goal_handle.get_result_async()

	@staticmethod
	def _normalise_regions(raw: Dict) -> Dict[int, List[dict]]:
		regions: Dict[int, List[dict]] = {}
		for key, waypoints in raw.items():
			regions[int(key)] = list(waypoints or [])
		return regions
