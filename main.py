from ETS2LA.Plugin import ETS2LAPlugin, PluginDescription, Author
from ETS2LA.Utils.translator import _

from Plugins.AR.classes import Coordinate, Polygon, Fade, Color, Circle, Text
from Modules.Semaphores.classes import Gate

from typing import List, Tuple, Optional
import math
import time


class Plugin(ETS2LAPlugin):
    description = PluginDescription(
        name=_("Toll Booth Assistant"),
        version="1.0.0",
        description=_("Tries detect toll booths on route and automatically pass them."),
        dependencies=["AdaptiveCruiseControl", "Map", "AR"],
        modules=["TruckSimAPI", "SDKController", "Semaphores"],
        tags=["Base", "Automation"],
        fps_cap=15,
    )

    author = Author(
        name="ReqT1m3out",
        url="https://github.com/reqt1m3out",
        icon="https://avatars.githubusercontent.com/u/63476357?v=4",
    )

    STOP_OFFSET_METERS = 1.2
    MAX_DETECTION_DISTANCE = 150.0
    MAX_LATERAL_DISTANCE = 11.0
    MAX_ROUTE_DEVIATION = 6.0
    AR_RENDER_DISTANCE = 130.0

    ACTIVATE_HOLD_TIME = 0.2
    ACTIVATE_RETRY_DELAY = 1.5
    STOPPED_SPEED_THRESHOLD = 0.8

    GATE_CLOSING = 0
    GATE_CLOSED = 1
    GATE_OPENING = 2
    GATE_OPEN = 3

    def imports(self):
        self.controller = self.modules.SDKController.SCSController()

    def init(self):
        self._selected_gate: Optional[Gate] = None
        self._selected_gate_uid: Optional[str] = None
        self._last_gate_state: Optional[int] = None

        self._activate_release_time: float = 0.0
        self._last_activate_attempt: float = 0.0
        self._payment_attempted: bool = False

        self._current_phase: str = "idle"

        self.STOP_OFFSET_METERS = max(1.0, min(1.5, float(self.STOP_OFFSET_METERS)))

    def _get_truck_forward_vector(self, api: dict) -> Tuple[float, float]:
        rotation = api["truckPlacement"]["rotationX"] * 360.0
        if rotation < 0:
            rotation += 360.0

        radians = math.radians(rotation)
        return -math.sin(radians), -math.cos(radians)

    def _get_gate_world_position(self, gate: Gate) -> Tuple[float, float, float]:
        return (
            gate.position.x + 512.0 * gate.cx,
            gate.position.y,
            gate.position.z + 512.0 * gate.cy,
        )

    def _calculate_distance_2d(self, point_a: Tuple[float, float],
                               point_b: Tuple[float, float]) -> float:
        dx = point_a[0] - point_b[0]
        dz = point_a[1] - point_b[1]
        return math.sqrt(dx * dx + dz * dz)

    def _normalize_vector_2d(self, vector: Tuple[float, float]) -> Tuple[float, float]:
        length = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
        if length < 1e-6:
            return (0.0, 0.0)
        return (vector[0] / length, vector[1] / length)

    def _project_point_to_route(self, point_x: float, point_z: float,
                                route: List[Tuple[float, float, float]]
                                ) -> Tuple[float, Tuple[float, float, float], Tuple[float, float]]:
        best_distance = float("inf")
        best_projection = route[0] if route else (0, 0, 0)
        best_direction = (0.0, 0.0)

        for i in range(1, len(route)):
            x1, y1, z1 = route[i - 1]
            x2, y2, z2 = route[i]

            segment_dx = x2 - x1
            segment_dz = z2 - z1
            segment_length_sq = segment_dx ** 2 + segment_dz ** 2

            if segment_length_sq < 1e-4:
                continue

            t = ((point_x - x1) * segment_dx + (point_z - z1) * segment_dz) / segment_length_sq
            t = max(0.0, min(1.0, t))

            proj_x = x1 + t * segment_dx
            proj_z = z1 + t * segment_dz
            proj_y = y1 + t * (y2 - y1)

            distance = math.sqrt((point_x - proj_x) ** 2 + (point_z - proj_z) ** 2)

            if distance < best_distance:
                best_distance = distance
                best_projection = (proj_x, proj_y, proj_z)
                best_direction = self._normalize_vector_2d((segment_dx, segment_dz))

        return best_distance, best_projection, best_direction

    def _find_gate_on_route(self, api: dict, route: List[Tuple[float, float, float]],
                            gates: List[Gate]) -> Tuple[Optional[Gate], Optional[float],
    Optional[Tuple[float, float, float]],
    Optional[Tuple[float, float]]]:

        truck_x = api["truckPlacement"]["coordinateX"]
        truck_z = api["truckPlacement"]["coordinateZ"]
        truck_pos_2d = (truck_x, truck_z)
        forward_vector = self._get_truck_forward_vector(api)

        best_gate: Optional[Gate] = None
        best_forward_distance = float("inf")
        best_projection = None
        best_direction = None

        for gate in gates:
            gate_x, gate_y, gate_z = self._get_gate_world_position(gate)
            gate_pos_2d = (gate_x, gate_z)

            total_distance = self._calculate_distance_2d(gate_pos_2d, truck_pos_2d)
            if total_distance > self.MAX_DETECTION_DISTANCE:
                continue

            to_gate_vector = (gate_x - truck_x, gate_z - truck_z)
            forward_distance = (
                    to_gate_vector[0] * forward_vector[0] +
                    to_gate_vector[1] * forward_vector[1]
            )

            if forward_distance <= 0:
                continue

            lateral_distance_sq = total_distance ** 2 - forward_distance ** 2
            if lateral_distance_sq < 0:
                lateral_distance_sq = 0
            lateral_distance = math.sqrt(lateral_distance_sq)

            if lateral_distance > self.MAX_LATERAL_DISTANCE:
                continue

            route_distance, projection, direction = self._project_point_to_route(
                gate_x, gate_z, route
            )

            if route_distance > self.MAX_ROUTE_DEVIATION:
                continue

            current_score = route_distance * 3.0 + forward_distance
            best_score = (best_projection and
                          self._calculate_distance_2d(
                              (best_gate.position.x + 512.0 * best_gate.cx,
                               best_gate.position.z + 512.0 * best_gate.cy),
                              (best_projection[0], best_projection[2])
                          ) * 3.0 + best_forward_distance) if best_gate else float("inf")

            if current_score < best_score:
                best_gate = gate
                best_forward_distance = forward_distance
                best_projection = projection
                best_direction = direction

        return best_gate, best_forward_distance, best_projection, best_direction

    def _create_ar_visualization(self, gate: Gate, forward_distance: float,
                                 projection: Tuple[float, float, float],
                                 route_direction: Tuple[float, float],
                                 api: dict) -> List[object]:
        ar_elements = []

        is_opening_or_open = gate.state in (self.GATE_OPENING, self.GATE_OPEN)
        gate_color = Color(100, 255, 150, 220) if is_opening_or_open else Color(255, 120, 120, 220)
        gate_fill = Color(100, 255, 150, 40) if is_opening_or_open else Color(255, 120, 120, 40)

        gate_x, gate_y, gate_z = self._get_gate_world_position(gate)

        ar_elements.append(
            Circle(
                center=Coordinate(gate_x, gate_y + 1.5, gate_z),
                radius=1.4,
                color=gate_color,
                fill=gate_fill,
                thickness=3,
                fade=Fade(
                    prox_fade_end=0,
                    prox_fade_start=0,
                    dist_fade_start=20,
                    dist_fade_end=self.AR_RENDER_DISTANCE
                ),
            )
        )

        ar_elements.append(
            Text(
                point=Coordinate(gate_x, gate_y + 2.8, gate_z),
                text=f"{forward_distance:.1f} m",
                size=20,
                color=gate_color,
                fade=Fade(
                    prox_fade_end=0,
                    prox_fade_start=0,
                    dist_fade_start=20,
                    dist_fade_end=self.AR_RENDER_DISTANCE
                ),
            )
        )

        if projection and route_direction:
            stop_x = projection[0]
            stop_y = projection[1]
            stop_z = projection[2]

            perpendicular = (-route_direction[1], route_direction[0])
            barrier_half_width = 2.6
            barrier_height = 1.0

            left_bottom = [
                stop_x - perpendicular[0] * barrier_half_width,
                stop_y,
                stop_z - perpendicular[1] * barrier_half_width
            ]
            right_bottom = [
                stop_x + perpendicular[0] * barrier_half_width,
                stop_y,
                stop_z + perpendicular[1] * barrier_half_width
            ]
            right_top = [
                right_bottom[0],
                stop_y + barrier_height,
                right_bottom[2]
            ]
            left_top = [
                left_bottom[0],
                stop_y + barrier_height,
                left_bottom[2]
            ]

            ar_elements.append(
                Polygon(
                    points=[
                        Coordinate(*left_bottom),
                        Coordinate(*right_bottom),
                        Coordinate(*right_top),
                        Coordinate(*left_top),
                    ],
                    closed=True,
                    color=Color(gate_color.r, gate_color.g, gate_color.b, 100),
                    fill=Color(gate_color.r, gate_color.g, gate_color.b, 45),
                    fade=Fade(0, 0, 999, 999),
                )
            )

        return ar_elements

    def _should_attempt_activation(self, current_time: float, gate_state: int) -> bool:
        if gate_state in (self.GATE_OPENING, self.GATE_OPEN):
            return False

        time_since_last_attempt = current_time - self._last_activate_attempt
        if time_since_last_attempt < self.ACTIVATE_RETRY_DELAY:
            return False

        return True

    def _trigger_activation(self, current_time: float) -> None:
        self._last_activate_attempt = current_time
        self._activate_release_time = current_time + self.ACTIVATE_HOLD_TIME
        self.controller.activate = True
        self._payment_attempted = True

    def _update_activation_state(self, current_time: float) -> None:
        if self._activate_release_time > 0 and current_time >= self._activate_release_time:
            self.controller.activate = False
            self._activate_release_time = 0.0

    def run(self):
        api = self.modules.TruckSimAPI.run()
        if api == "not connected" or api == "error checking API status":
            self._reset_state()
            return

        if api.get("pause", False):
            self._reset_state()
            return

        route_points = self.tags.steering_points
        route_points = self.tags.merge(route_points)

        if not route_points or len(route_points) < 2:
            self._reset_state()
            return

        semaphores = self.modules.Semaphores.run()
        if not semaphores:
            self._reset_state()
            return

        gates = [s for s in semaphores if isinstance(s, Gate)]
        if not gates:
            self._reset_state()
            return

        gate, forward_distance, projection, route_direction = self._find_gate_on_route(
            api, route_points, gates
        )

        current_time = time.perf_counter()
        self._update_activation_state(current_time)

        if not gate or gate.state not in (self.GATE_CLOSING, self.GATE_CLOSED):
            self._reset_state()
            return

        if self._selected_gate_uid != gate.uid:
            self._selected_gate_uid = gate.uid
            self._selected_gate = gate
            self._payment_attempted = False

        self._last_gate_state = gate.state

        try:
            self.tags.AR = self._create_ar_visualization(
                gate, forward_distance, projection, route_direction, api
            )
        except Exception:
            self.tags.AR = []

        current_speed = api["truckFloat"]["speed"]
        is_stopped = current_speed < self.STOPPED_SPEED_THRESHOLD
        is_close_to_gate = forward_distance < 8.0
        gate_ready = (
                gate.state in (self.GATE_OPENING, self.GATE_OPEN) or
                api.get("specialBool", {}).get("tollgate", False)
        )

        if self._current_phase == "passing":
            self.tags.stop_in = -1
            self.tags.override_acceleration = 0
            if forward_distance > 15.0:
                self._reset_state()

        elif is_stopped and is_close_to_gate:
            self._current_phase = "stopped"

            if self._should_attempt_activation(current_time, gate.state):
                self._trigger_activation(current_time)

            if gate_ready:
                self._current_phase = "passing"
                self.tags.stop_in = -1
                self.tags.override_acceleration = 0
            else:
                self._current_phase = "waiting"
                self.tags.stop_in = forward_distance
                self.tags.override_acceleration = -1

        else:
            self._current_phase = "approach"
            self.tags.stop_in = forward_distance

    def _reset_state(self):
        self.tags.stop_in = -1
        self.tags.override_acceleration = 0.0
        self.tags.AR = []
        self._current_phase = "idle"
        self._selected_gate = None
        self._selected_gate_uid = None
        self._payment_attempted = False
        self.controller.activate = False
        self._activate_release_time = 0.0