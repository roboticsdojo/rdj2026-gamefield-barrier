"""gamefield_barrier: ROS 2 bridge between the Gamefield Software and the barrier Arduino."""
import json
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .serial_link import SerialLink

STATES = ('UNKNOWN', 'INITIALIZING', 'OPENING', 'OPEN', 'CLOSING', 'CLOSED', 'FAULT')


class BarrierNode(Node):
    def __init__(self):
        super().__init__('gamefield_barrier')

        # ---------------- parameters ----------------
        self.declare_parameter('port', '/dev/gamefield_barrier')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('poll_period', 0.5)       # 'p' poll = status + heartbeat
        self.declare_parameter('comms_timeout', 3.0)     # silence before reconnecting
        self.declare_parameter('ack_timeout', 1.0)       # wait for ACK/NAK
        self.declare_parameter('move_timeout', 6.0)      # wait for barrier to arrive
        self.declare_parameter('reconnect_period', 2.0)
        self.declare_parameter('connect_settle', 2.0)    # Uno boot time after port open

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        poll_period = self.get_parameter('poll_period').value
        self._comms_timeout = self.get_parameter('comms_timeout').value
        self._ack_timeout = self.get_parameter('ack_timeout').value
        self._move_timeout = self.get_parameter('move_timeout').value

        # ---------------- shared state ----------------
        self._cond = threading.Condition()
        self._cmd_lock = threading.Lock()      # one command at a time
        self._state = 'UNKNOWN'
        self._state_version = 0                # increments on every state change
        self._info = {}
        self._connected = False
        self._ack_line = None
        self._last_rx = time.monotonic()

        # ---------------- ROS interfaces ----------------
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, '/gamefield/barrier/state', latched)
        self._status_pub = self.create_publisher(String, '/gamefield/barrier/status', 10)
        self._event_pub = self.create_publisher(String, '/gamefield/barrier/event', 10)

        cb = ReentrantCallbackGroup()   # services block while waiting; keep others free
        self.create_service(Trigger, '/gamefield/barrier/open', self._srv_open,
                            callback_group=cb)
        self.create_service(Trigger, '/gamefield/barrier/close', self._srv_close,
                            callback_group=cb)
        self.create_service(Trigger, '/gamefield/barrier/reset', self._srv_reset,
                            callback_group=cb)
        self.create_timer(poll_period, self._poll, callback_group=cb)

        self._publish_state('UNKNOWN')

        # ---------------- serial link ----------------
        self._link = SerialLink(
            port, baud, self._on_line, self._on_connection, self.get_logger(),
            reconnect_period=self.get_parameter('reconnect_period').value,
            settle_time=self.get_parameter('connect_settle').value)
        self._link.start()
        self.get_logger().info(f'gamefield_barrier started (port {port})')

    # ======================================================================
    # Serial callbacks (run on the serial thread)
    # ======================================================================
    @staticmethod
    def _parse_state(line):
        fields = {}
        for token in line.split():
            if ':' in token:
                key, value = token.split(':', 1)
                fields[key] = value
        state = fields.get('STATE')
        if state not in STATES:
            return None

        def to_int(key, default=-1):
            try:
                return int(fields.get(key, default))
            except ValueError:
                return default

        return {
            'state': state,
            'angle': to_int('ANGLE'),
            'pot': to_int('POT'),
            'distance_cm': to_int('DIST'),
            'sequence': fields.get('SEQ', 'IDLE'),
            'fault': fields.get('FAULT', 'NONE'),
        }

    def _on_line(self, line):
        now = time.monotonic()

        if line.startswith('STATE:'):
            info = self._parse_state(line)
            if info is None:
                return
            with self._cond:
                self._last_rx = now
                self._info = info
                changed = info['state'] != self._state
                if changed:
                    self._state = info['state']
                    self._state_version += 1
                self._cond.notify_all()
            if changed:
                self.get_logger().info(f"Barrier state: {info['state']}")
                self._publish_state(info['state'])
            self._publish_status()
            return

        with self._cond:
            self._last_rx = now
            if line.startswith(('ACK:', 'NAK:')):
                self._ack_line = line
                self._cond.notify_all()

        if line.startswith(('MANUAL:', 'SEQ:')):
            self.get_logger().info(f'Barrier event: {line}')
            msg = String()
            msg.data = line
            self._event_pub.publish(msg)

    def _on_connection(self, connected):
        with self._cond:
            self._connected = connected
            self._last_rx = time.monotonic()
            if not connected:
                self._state = 'UNKNOWN'
                self._state_version += 1
                self._info = {}
            self._cond.notify_all()

        if connected:
            self.get_logger().info('Barrier controller connected')
        else:
            self.get_logger().warn('Barrier controller disconnected - state UNKNOWN')
            self._publish_state('UNKNOWN')
        self._publish_status()

    # ======================================================================
    # Publishing
    # ======================================================================
    def _publish_state(self, state):
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    def _publish_status(self):
        with self._cond:
            status = {'connected': self._connected, 'state': self._state}
            status.update({k: v for k, v in self._info.items() if k != 'state'})
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ======================================================================
    # Poll / heartbeat / comms watchdog
    # ======================================================================
    def _poll(self):
        if not self._link.connected:
            return
        self._link.write('p')   # reply gives status; any byte also counts as heartbeat
        with self._cond:
            silent_for = time.monotonic() - self._last_rx
        if silent_for > self._comms_timeout:
            self.get_logger().warn(
                f'No data from Arduino for {silent_for:.1f}s - reconnecting')
            self._link.request_reconnect()

    # ======================================================================
    # Commands
    # ======================================================================
    def _run_command(self, char, name, target, wait_transition=False):
        """Send one command; return (success, message) once the barrier arrives."""
        if not self._link.connected:
            return False, 'Barrier controller is not connected'

        with self._cmd_lock:
            with self._cond:
                self._ack_line = None
                version0 = self._state_version

            if not self._link.write(char):
                return False, 'Serial write failed'

            # 1) ACK / NAK from the Arduino
            def acked():
                return (self._ack_line is not None and
                        self._ack_line[4:].startswith(name))

            with self._cond:
                if not self._cond.wait_for(acked, timeout=self._ack_timeout):
                    return False, f'No acknowledgement for {name} from Arduino'
                ack = self._ack_line
            if ack.startswith('NAK:'):
                return False, ack

            # 2) Wait for the barrier to actually reach the commanded position
            def done():
                if not self._connected:
                    return True
                if wait_transition and self._state_version <= version0:
                    return False
                return self._state in (target, 'FAULT')

            with self._cond:
                finished = self._cond.wait_for(done, timeout=self._move_timeout)
                state = self._state
                connected = self._connected

            if not connected:
                return False, 'Connection lost while waiting'
            if not finished:
                return False, f'Timed out waiting for {target} (state={state})'
            if state == target:
                return True, f'Barrier {target}'
            return False, f'Barrier entered {state}'

    def _srv_open(self, request, response):
        response.success, response.message = self._run_command('s', 'OPEN', 'OPEN')
        self.get_logger().info(f'open -> {response.success}: {response.message}')
        return response

    def _srv_close(self, request, response):
        response.success, response.message = self._run_command('c', 'CLOSE', 'CLOSED')
        self.get_logger().info(f'close -> {response.success}: {response.message}')
        return response

    def _srv_reset(self, request, response):
        response.success, response.message = self._run_command(
            'r', 'RESET', 'CLOSED', wait_transition=True)
        self.get_logger().info(f'reset -> {response.success}: {response.message}')
        return response

    # ======================================================================
    def shutdown(self):
        self._link.stop()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BarrierNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()