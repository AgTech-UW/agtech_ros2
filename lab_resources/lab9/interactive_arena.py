#!/usr/bin/env python3
"""
=============================================================================
  INTERACTIVE ARENA — Visual God Node Simulator
  AgTech ROS Course | PHYS-4000 / AG-3000 | Lab 9
=============================================================================

  A matplotlib-based ROS 2 node that replaces the "fake_god_node" with a
  full visual simulation of the arena.

  What it does:
    - Shows a top-down arena grid (like a ceiling camera view)
    - Simulates robot physics (unicycle model)
    - Publishes the robot's simulated position on /tractor/gps
    - Subscribes to /cmd_vel from the student's controller
    - Moves the robot based on received velocity commands
    - Click anywhere in the arena to set/move the target
    - Publishes clicked target position on /tractor/target

  The student writes student_tractor.py, which subscribes to /tractor/gps,
  computes P-control error, and publishes /cmd_vel. The SAME student code
  works here AND with the real ceiling camera. Zero code changes.

  Usage:
    Terminal 1:  ros2 run lab9 interactive_arena
    Terminal 2:  ros2 run lab9 student_tractor

  Parameters (override with --ros-args -p key:=value):
    arena_size:    Field size in meters (default: 4.0)
    start_x:       Robot starting X (default: 3.0)
    start_y:       Robot starting Y (default: 3.0)
    start_theta:   Robot starting heading in radians (default: 0.0)
    publish_rate:  GPS publish rate in Hz (default: 20.0)

  For small office testing:
    ros2 run lab9 interactive_arena --ros-args -p arena_size:=2.0
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
import threading
import time

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import FancyArrow, Circle
    import numpy as np
except ImportError:
    print("\n" + "=" * 60)
    print("  ERROR: matplotlib not found!")
    print("  Install it with:")
    print("    pip3 install matplotlib --break-system-packages")
    print("=" * 60 + "\n")
    exit(1)


class InteractiveArena(Node):
    def __init__(self):
        super().__init__('interactive_arena')

        # =================================================================
        # PARAMETERS
        # =================================================================
        self.declare_parameter('arena_size', 4.0)
        self.declare_parameter('start_x', 3.0)
        self.declare_parameter('start_y', 3.0)
        self.declare_parameter('start_theta', 0.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('goal_tolerance', 0.15)

        self.arena = self.get_parameter('arena_size').value
        self.robot_x = self.get_parameter('start_x').value
        self.robot_y = self.get_parameter('start_y').value
        self.robot_theta = self.get_parameter('start_theta').value
        self.rate = self.get_parameter('publish_rate').value
        self.goal_tol = self.get_parameter('goal_tolerance').value

        # =================================================================
        # TARGET (set by mouse click)
        # =================================================================
        self.target_x = None
        self.target_y = None
        self.target_reached = False

        # =================================================================
        # VELOCITY STATE (from student's /cmd_vel)
        # =================================================================
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_time = None
        self.cmd_watchdog = 0.5  # seconds

        # =================================================================
        # PATH TRAIL
        # =================================================================
        self.trail_x = [self.robot_x]
        self.trail_y = [self.robot_y]
        self.max_trail = 5000

        # =================================================================
        # STATISTICS
        # =================================================================
        self.step_count = 0
        self.total_distance = 0.0
        self.prev_x = self.robot_x
        self.prev_y = self.robot_y

        # =================================================================
        # ROS PUBLISHERS & SUBSCRIBERS
        # =================================================================
        self.gps_pub = self.create_publisher(Pose, '/tractor/gps', 10)
        self.target_pub = self.create_publisher(Pose, '/tractor/target', 10)
        self.tracking_pub = self.create_publisher(Bool, '/tractor/tracking', 10)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        # =================================================================
        # PHYSICS TIMER (runs at publish_rate Hz)
        # =================================================================
        self.dt = 1.0 / self.rate
        self.physics_timer = self.create_timer(self.dt, self.physics_step)

        self.get_logger().info(
            f'\n'
            f'╔══════════════════════════════════════════════════╗\n'
            f'║         INTERACTIVE ARENA ONLINE                ║\n'
            f'║  Arena: {self.arena}m × {self.arena}m                          ║\n'
            f'║  Robot: ({self.robot_x:.1f}, {self.robot_y:.1f})  θ={self.robot_theta:.2f}            ║\n'
            f'║                                                  ║\n'
            f'║  Click in the window to set a target.            ║\n'
            f'║  Run student_tractor in another terminal.        ║\n'
            f'╚══════════════════════════════════════════════════╝'
        )

    # =====================================================================
    # CALLBACKS
    # =====================================================================
    def cmd_callback(self, msg):
        """Receive velocity command from the student's controller."""
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.last_cmd_time = time.monotonic()

    # =====================================================================
    # PHYSICS SIMULATION
    # =====================================================================
    def physics_step(self):
        """Advance the simulated robot one timestep."""

        # If no cmd_vel received recently, assume stopped
        if self.last_cmd_time is not None:
            if time.monotonic() - self.last_cmd_time > self.cmd_watchdog:
                self.cmd_linear = 0.0
                self.cmd_angular = 0.0

            # --- Unicycle kinematic model ---
            self.robot_x += self.cmd_linear * math.cos(self.robot_theta) * self.dt
            self.robot_y += self.cmd_linear * math.sin(self.robot_theta) * self.dt
            self.robot_theta += self.cmd_angular * self.dt

            # Normalize theta to [-pi, pi]
            while self.robot_theta > math.pi:
                self.robot_theta -= 2.0 * math.pi
            while self.robot_theta < -math.pi:
                self.robot_theta += 2.0 * math.pi

            # Track distance traveled
            dx = self.robot_x - self.prev_x
            dy = self.robot_y - self.prev_y
            self.total_distance += math.sqrt(dx * dx + dy * dy)
            self.prev_x = self.robot_x
            self.prev_y = self.robot_y
            self.step_count += 1

        # --- Trail ---
        self.trail_x.append(self.robot_x)
        self.trail_y.append(self.robot_y)
        if len(self.trail_x) > self.max_trail:
            self.trail_x.pop(0)
            self.trail_y.pop(0)

        # --- Check target reached ---
        if self.target_x is not None:
            tdx = self.target_x - self.robot_x
            tdy = self.target_y - self.robot_y
            if math.sqrt(tdx * tdx + tdy * tdy) < self.goal_tol:
                if not self.target_reached:
                    elapsed = self.step_count * self.dt
                    self.get_logger().info(
                        f'TARGET REACHED in {elapsed:.1f}s | '
                        f'Distance traveled: {self.total_distance:.2f}m | '
                        f'Click to set a new target.'
                    )
                    self.target_reached = True

        # --- Publish GPS ---
        gps = Pose()
        gps.x = self.robot_x
        gps.y = self.robot_y
        gps.theta = self.robot_theta
        self.gps_pub.publish(gps)

        # --- Publish tracking status ---
        tracking = Bool()
        tracking.data = True
        self.tracking_pub.publish(tracking)

        # --- Re-publish target (so late-joining student nodes get it) ---
        if self.target_x is not None:
            tgt = Pose()
            tgt.x = self.target_x
            tgt.y = self.target_y
            self.target_pub.publish(tgt)

    # =====================================================================
    # TARGET SETTER (called from matplotlib click)
    # =====================================================================
    def set_target(self, x, y):
        """Set a new navigation target (from mouse click)."""
        self.target_x = x
        self.target_y = y
        self.target_reached = False

        # Reset stats for new run
        self.step_count = 0
        self.total_distance = 0.0
        self.prev_x = self.robot_x
        self.prev_y = self.robot_y

        # Clear trail
        self.trail_x = [self.robot_x]
        self.trail_y = [self.robot_y]

        self.get_logger().info(f'New target: ({x:.2f}, {y:.2f})')

    def reset_robot(self, x, y, theta=0.0):
        """Reset robot to a new position."""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
        self.trail_x = [x]
        self.trail_y = [y]
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.step_count = 0
        self.total_distance = 0.0
        self.prev_x = x
        self.prev_y = y
        self.get_logger().info(f'Robot reset to ({x:.2f}, {y:.2f}, θ={theta:.2f})')


# =========================================================================
# MATPLOTLIB GUI (runs in main thread)
# =========================================================================
def run_gui(node):
    """Set up and run the matplotlib interactive window."""

    arena = node.arena

    # --- Figure Setup ---
    fig, ax = plt.subplots(figsize=(8, 8.5))
    fig.patch.set_facecolor('#0f172a')
    ax.set_facecolor('#0c1222')

    ax.set_xlim(-0.3, arena + 0.3)
    ax.set_ylim(-0.3, arena + 0.3)
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)', color='#64748b', fontsize=10, fontfamily='monospace')
    ax.set_ylabel('Y (meters)', color='#64748b', fontsize=10, fontfamily='monospace')

    title = ax.set_title(
        'Lab 9: Interactive Arena — Click to Set Target',
        color='#38bdf8', fontsize=13, fontweight='bold', fontfamily='monospace',
        pad=12
    )

    # --- Grid ---
    for i in range(int(arena) + 1):
        ax.axhline(y=i, color='#1e293b', linewidth=0.8)
        ax.axvline(x=i, color='#1e293b', linewidth=0.8)
    for i in np.arange(0, arena + 0.5, 0.5):
        ax.axhline(y=i, color='#1e293b', linewidth=0.3, alpha=0.5)
        ax.axvline(x=i, color='#1e293b', linewidth=0.3, alpha=0.5)

    ax.tick_params(colors='#475569', labelsize=9)
    for spine in ax.spines.values():
        spine.set_color('#334155')

    # --- Arena Boundary ---
    boundary = plt.Rectangle((0, 0), arena, arena, fill=False,
                              edgecolor='#475569', linewidth=2, linestyle='--')
    ax.add_patch(boundary)

    # --- Plot Elements ---
    trail_line, = ax.plot([], [], '-', color='#38bdf8', linewidth=1.8,
                          alpha=0.6, solid_capstyle='round')
    robot_dot, = ax.plot([], [], 'o', color='#4ade80', markersize=10, zorder=10)
    heading_line, = ax.plot([], [], '-', color='#4ade80', linewidth=2.5,
                            alpha=0.9, solid_capstyle='round', zorder=11)
    target_h, = ax.plot([], [], '-', color='#f87171', linewidth=2, alpha=0.8)
    target_v, = ax.plot([], [], '-', color='#f87171', linewidth=2, alpha=0.8)
    target_circ = Circle((0, 0), 0, fill=False, edgecolor='#f87171',
                          linewidth=1, linestyle=':', alpha=0.3)
    ax.add_patch(target_circ)
    error_line, = ax.plot([], [], '--', color='#f87171', linewidth=0.8, alpha=0.35)
    goal_heading_line, = ax.plot([], [], '--', color='#fbbf24', linewidth=0.8, alpha=0.4)

    # --- HUD Text ---
    hud = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                  fontsize=10, fontfamily='monospace', color='#94a3b8',
                  verticalalignment='top',
                  bbox=dict(boxstyle='round,pad=0.5', facecolor='#1e293b',
                            edgecolor='#334155', alpha=0.92))

    status = ax.text(0.5, -0.04, 'Click anywhere in the arena to set a target',
                     transform=ax.transAxes, fontsize=11, color='#fbbf24',
                     ha='center', fontweight='bold', fontfamily='monospace')

    # --- Key labels ---
    ax.text(0.98, 0.02, '/tractor/gps → /cmd_vel',
            transform=ax.transAxes, fontsize=8, fontfamily='monospace',
            color='#334155', ha='right', va='bottom')

    # =====================================================================
    # CLICK HANDLER
    # =====================================================================
    def on_click(event):
        if event.inaxes != ax:
            return
        x, y = event.xdata, event.ydata
        if 0 <= x <= arena and 0 <= y <= arena:
            node.set_target(x, y)

    fig.canvas.mpl_connect('button_press_event', on_click)

    # =====================================================================
    # KEY HANDLER (R = reset robot position)
    # =====================================================================
    def on_key(event):
        if event.key == 'r':
            node.reset_robot(
                node.get_parameter('start_x').value,
                node.get_parameter('start_y').value,
                node.get_parameter('start_theta').value
            )

    fig.canvas.mpl_connect('key_press_event', on_key)

    # =====================================================================
    # ANIMATION LOOP (50ms = 20fps visual update)
    # =====================================================================
    def update(frame):
        rx, ry, rth = node.robot_x, node.robot_y, node.robot_theta

        # Robot position
        robot_dot.set_data([rx], [ry])

        # Heading arrow
        hl = 0.3
        hx = rx + hl * math.cos(rth)
        hy = ry + hl * math.sin(rth)
        heading_line.set_data([rx, hx], [ry, hy])

        # Trail
        trail_line.set_data(node.trail_x, node.trail_y)

        if node.target_x is not None:
            tx, ty = node.target_x, node.target_y

            # Target crosshair
            cs = 0.18
            target_h.set_data([tx - cs, tx + cs], [ty, ty])
            target_v.set_data([tx, tx], [ty - cs, ty + cs])

            # Tolerance circle
            target_circ.set_center((tx, ty))
            target_circ.set_radius(node.goal_tol)

            # Error line (robot → target)
            error_line.set_data([rx, tx], [ry, ty])

            # Compute error values for display
            edx = tx - rx
            edy = ty - ry
            dist = math.sqrt(edx * edx + edy * edy)
            atg = math.atan2(edy, edx)
            ae = atg - rth
            while ae > math.pi: ae -= 2 * math.pi
            while ae < -math.pi: ae += 2 * math.pi

            # Goal heading line
            ghl = min(0.3, dist * 0.5)
            ghx = rx + ghl * math.cos(atg)
            ghy = ry + ghl * math.sin(atg)
            goal_heading_line.set_data([rx, ghx], [ry, ghy])

            # HUD
            elapsed = node.step_count * node.dt
            hud.set_text(
                f' /tractor/gps\n'
                f'   x = {rx:+7.3f}    y = {ry:+7.3f}\n'
                f'   θ = {rth:+7.3f} rad  ({math.degrees(rth):+6.1f}°)\n'
                f'\n'
                f' Error\n'
                f'   dist  = {dist:.3f} m\n'
                f'   angle = {ae:+.3f} rad  ({math.degrees(ae):+.1f}°)\n'
                f'\n'
                f' /cmd_vel\n'
                f'   linear.x  = {node.cmd_linear:+.3f}\n'
                f'   angular.z = {node.cmd_angular:+.3f}\n'
                f'\n'
                f' t = {elapsed:.1f}s  d = {node.total_distance:.2f}m'
            )

            if node.target_reached:
                status.set_text(
                    f'TARGET REACHED in {elapsed:.1f}s — '
                    f'click to set a new target  |  R = reset robot'
                )
                status.set_color('#4ade80')
            else:
                status.set_text(
                    f'Driving to ({tx:.1f}, {ty:.1f}) — '
                    f'{dist:.2f}m remaining  |  R = reset robot'
                )
                status.set_color('#fbbf24')
        else:
            hud.set_text(
                f' /tractor/gps\n'
                f'   x = {rx:+7.3f}    y = {ry:+7.3f}\n'
                f'   θ = {rth:+7.3f} rad\n'
                f'\n'
                f' /cmd_vel\n'
                f'   linear.x  = {node.cmd_linear:+.3f}\n'
                f'   angular.z = {node.cmd_angular:+.3f}\n'
                f'\n'
                f' Waiting for target...'
            )
            goal_heading_line.set_data([], [])

        return (robot_dot, heading_line, trail_line, target_h, target_v,
                error_line, goal_heading_line, hud, status)

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout(pad=2.0)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass


# =========================================================================
# MAIN
# =========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = InteractiveArena()

    # Spin ROS in a background thread so matplotlib can own the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run the GUI (blocks until window is closed)
    run_gui(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
