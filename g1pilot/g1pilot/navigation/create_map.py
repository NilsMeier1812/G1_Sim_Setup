#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
create_map — Belegungskarte (/map) aus den echten Umgebungs-Objekten.

Fruehere Version: ein leerer Dummy (Hindernisse fest im Code auskommentiert,
siehe Git-Historie) -- der Planer plante faktisch geradeaus. Jetzt rastert
dieser Node die Objekte, die scene_bridge aus der laufenden MuJoCo-Sim als
/scene_markers veroeffentlicht (siehe g1pilot/docs/51_navigation_technik.md), in eine
2D-Belegungskarte. Das sind GENAU die Objekte, die im Scene-Editor gebaut und
beim Sim-Start als G1_ENV geladen wurden.

Sowohl Hindernisse ALS AUCH Greif-Objekte zaehlen hier als belegt: ein
greifbares Objekt auf dem Boden ist BIS ZUM Greifen trotzdem ein Hindernis
fuer die laufende Basis. Die Hindernis/Greif-Unterscheidung (Kollision OK vs.
ausweichen) betrifft nur den ARM -- siehe ik_solver.py.

Nur was in den Hoehenbereich des Roboters ragt (obstacle_min_z ..
obstacle_max_z) wird gestempelt: eine Bodenplatte (CAD-Zelle), auf der der G1
laeuft, oder eine Kabelpritsche ueber Kopf wuerden sonst die ganze Flaeche
darunter sperren.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray

from g1pilot.navigation import scene_markers as sm


class SceneMapPublisher(Node):
    def __init__(self):
        super().__init__('scene_map_publisher')
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('markers_topic', '/scene_markers')
        self.declare_parameter('publish_rate_hz', 1.0)
        # Hoehenband, in dem Objekte die Basis blockieren (m ueber Boden):
        # darunter = begehbar (Bodenplatten, Kabelkanaele), darueber = Kopffreiheit.
        self.declare_parameter('obstacle_min_z', 0.05)
        self.declare_parameter('obstacle_max_z', 1.8)

        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.res = float(self.get_parameter('resolution').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.min_z = float(self.get_parameter('obstacle_min_z').value)
        self.max_z = float(self.get_parameter('obstacle_max_z').value)
        self.ox = -(self.w * self.res) / 2.0
        self.oy = -(self.h * self.res) / 2.0

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub_markers = self.create_subscription(
            MarkerArray, self.get_parameter('markers_topic').value, self._on_markers, qos)
        self.pub = self.create_publisher(OccupancyGrid, '/map', 1)

        self._latest_markers = None
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate if rate > 0 else 1.0, self.publish_map)

        self.get_logger().info(
            f"create_map aktiv: rastert '{self.get_parameter('markers_topic').value}' "
            f"({self.w}x{self.h} @ {self.res} m/Zelle, Frame '{self.frame_id}')."
        )

    def _on_markers(self, msg: MarkerArray):
        self._latest_markers = msg

    def world_to_grid(self, x, y):
        ix = int((x - self.ox) / self.res)
        iy = int((y - self.oy) / self.res)
        return ix, iy

    def _stamp_footprint(self, map_data, cx, cy, hx, hy):
        """Achsenparalleles Rechteck (Welt-Halbextents hx,hy um cx,cy) als
        belegt (100) in die Rasterkarte stempeln."""
        ix0, iy0 = self.world_to_grid(cx - hx, cy - hy)
        ix1, iy1 = self.world_to_grid(cx + hx, cy + hy)
        x0, x1 = max(0, min(ix0, ix1)), min(self.w - 1, max(ix0, ix1))
        y0, y1 = max(0, min(iy0, iy1)), min(self.h - 1, max(iy0, iy1))
        for yy in range(y0, y1 + 1):
            base = yy * self.w
            for xx in range(x0, x1 + 1):
                map_data[base + xx] = 100

    def _build_map_data(self):
        data = [0] * (self.w * self.h)
        if self._latest_markers is None:
            return data
        for marker in self._latest_markers.markers:
            if marker.action != 0:   # 0 == Marker.ADD; DELETE/andere ueberspringen
                continue
            if not sm.is_collision_marker(marker):   # reine Anzeige (z.B. Optik-Mesh)
                continue
            try:
                z0, z1 = sm.vertical_range(marker)
                if z1 < self.min_z or z0 > self.max_z:
                    continue   # begehbar (flach) oder ueber Kopfhoehe
                x, y, hx, hy = sm.footprint_xy(marker)
            except (AttributeError, ZeroDivisionError):
                continue
            self._stamp_footprint(data, x, y, hx, hy)
        return data

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.width = self.w
        grid.info.height = self.h
        grid.info.resolution = self.res
        origin = Pose()
        origin.position.x = self.ox
        origin.position.y = self.oy
        origin.orientation.w = 1.0
        grid.info.origin = origin
        grid.data = self._build_map_data()
        self.pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = SceneMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
