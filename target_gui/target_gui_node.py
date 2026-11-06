import math
import queue
import threading
import time
import tkinter as tk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32, UInt8
from shoot_belt.srv import ManualCurrent


class TargetGui(Node):
    """Touch GUI for configuring the current used by the next shot."""

    def __init__(self):
        super().__init__('target_gui')

        self.declare_parameter('service_name', '/shoot_belt/manual_current')
        self.declare_parameter(
            'state_topic', '/shoot_belt/manual_current_state')
        self.declare_parameter(
            'operation_state_topic', '/shoot_belt/operation_state')
        self.declare_parameter('operation_state_timeout_sec', 1.0)
        self.declare_parameter('full_screen', True)

        self.service_name = str(self.get_parameter('service_name').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.operation_state_topic = str(
            self.get_parameter('operation_state_topic').value)
        self.operation_state_timeout_sec = float(
            self.get_parameter('operation_state_timeout_sec').value)
        self.full_screen = bool(self.get_parameter('full_screen').value)

        if not math.isfinite(self.operation_state_timeout_sec) or \
                self.operation_state_timeout_sec <= 0.0:
            raise ValueError('operation_state_timeout_sec must be finite and positive')

        self.event_queue = queue.Queue()
        self.request_in_flight = False
        self.last_operation_state_time = None

        self.client = self.create_client(ManualCurrent, self.service_name)
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_sub = self.create_subscription(
            Float32, self.state_topic, self._on_current_state, state_qos)
        self.operation_state_sub = self.create_subscription(
            UInt8, self.operation_state_topic, self._on_operation_state, state_qos)

        self.root = tk.Tk()
        self.root.title('Target GUI - 射出電流設定')
        self.root.configure(bg='#202124')
        self.root.geometry('1280x800')
        self.root.minsize(800, 480)
        self.root.protocol('WM_DELETE_WINDOW', self.close_window)
        self.root.bind('<Escape>', lambda _event: self.close_window())
        self.root.bind('<F11>', lambda _event: self.toggle_full_screen())

        # Upper control area: three columns with aligned bottom edges.
        self.main_frame = tk.Frame(self.root, bg='#202124')
        self.main_frame.grid(
            row=0, column=0, sticky='nsew', padx=10, pady=(10, 5))
        self.main_frame.grid_rowconfigure(0, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=2, uniform='main')
        self.main_frame.grid_columnconfigure(1, weight=1, uniform='main')
        self.main_frame.grid_columnconfigure(2, weight=1, uniform='main')

        self.fixed_frame = tk.Frame(self.main_frame, bg='#202124')
        self.fixed_frame.grid(
            row=0, column=0, sticky='nsew', padx=(0, 6), pady=0)
        for row in range(4):
            self.fixed_frame.grid_rowconfigure(row, weight=1, uniform='fixed_bucket')
        self.fixed_frame.grid_columnconfigure(0, weight=1)
        self.fixed_buttons = []
        fixed_bucket_options = (
            ('高', ManualCurrent.Request.SET_FIXED_BUCKET_1),
            ('中', ManualCurrent.Request.SET_FIXED_BUCKET_2),
            ('低', ManualCurrent.Request.SET_FIXED_BUCKET_3),
            ('移動', ManualCurrent.Request.SET_MOVE),
        )
        for row, (label, operation) in enumerate(fixed_bucket_options):
            button = tk.Button(
                self.fixed_frame,
                text=label,
                command=lambda op=operation: self._send_operation(op),
                font=('Sans', 24, 'bold'),
                bg='#3569a8', fg='white',
                activebackground='#467fc4', activeforeground='white',
                relief=tk.FLAT, bd=0, takefocus=False,
            )
            button.grid(
                row=row, column=0, sticky='nsew',
                pady=(0, 5) if row < 3 else (0, 0))
            self.fixed_buttons.append(button)

        # Center column: brake and reset occupy the same total height
        # as the four preset buttons.
        self.middle_frame = tk.Frame(self.main_frame, bg='#202124')
        self.middle_frame.grid(
            row=0, column=1, sticky='nsew', padx=6, pady=0)
        self.middle_frame.grid_rowconfigure(0, weight=1, uniform='middle')
        self.middle_frame.grid_rowconfigure(1, weight=1, uniform='middle')
        self.middle_frame.grid_columnconfigure(0, weight=1)

        self.brake_button = tk.Button(
            self.middle_frame,
            text='ブレーキ',
            command=lambda: self._log_gui_only_action('ブレーキ'),
            font=('Sans', 26, 'bold'),
            bg='#b3261e', fg='white',
            activebackground='#d13b32', activeforeground='white',
            relief=tk.FLAT, bd=0, takefocus=False,
        )
        self.brake_button.grid(
            row=0, column=0, sticky='nsew', pady=(0, 5))

        self.reset_button = tk.Button(
            self.middle_frame,
            text='リセット',
            command=lambda: self._log_gui_only_action('リセット'),
            font=('Sans', 26, 'bold'),
            bg='#5f6368', fg='white',
            activebackground='#777b80', activeforeground='white',
            relief=tk.FLAT, bd=0, takefocus=False,
        )
        self.reset_button.grid(
            row=1, column=0, sticky='nsew', pady=(0, 0))

        self.right_frame = tk.Frame(self.main_frame, bg='#202124')
        self.right_frame.grid(
            row=0, column=2, sticky='nsew', padx=(6, 0), pady=0)
        self.right_frame.grid_rowconfigure(0, weight=1, uniform='right')
        self.right_frame.grid_rowconfigure(1, weight=1, uniform='right')
        self.right_frame.grid_columnconfigure(0, weight=1)

        self.up_button = tk.Button(
            self.right_frame,
            text='UP',
            command=lambda: self._send_operation(
                ManualCurrent.Request.INCREASE),
            font=('Sans', 28, 'bold'),
            bg='#36865a', fg='white',
            activebackground='#49a873', activeforeground='white',
            relief=tk.FLAT, bd=0, takefocus=False,
        )
        self.up_button.grid(
            row=0, column=0, sticky='nsew', pady=(0, 5))

        self.down_button = tk.Button(
            self.right_frame,
            text='DOWN',
            command=lambda: self._send_operation(
                ManualCurrent.Request.DECREASE),
            font=('Sans', 28, 'bold'),
            bg='#a64d4d', fg='white',
            activebackground='#c76363', activeforeground='white',
            relief=tk.FLAT, bd=0, takefocus=False,
        )
        self.down_button.grid(
            row=1, column=0, sticky='nsew', pady=(0, 0))

        # Full-width information area at the bottom of the GUI.
        self.footer = tk.Frame(self.root, bg='#303134')
        self.footer.grid(
            row=1, column=0, sticky='nsew', padx=10, pady=(5, 10))
        self.footer.grid_rowconfigure(0, weight=1)
        self.footer.grid_columnconfigure(0, weight=1, uniform='footer')
        self.footer.grid_columnconfigure(1, weight=1, uniform='footer')

        self.current_var = tk.StringVar(value='--.-- A')
        self.current_label = tk.Label(
            self.footer,
            textvariable=self.current_var,
            font=('Sans', 52, 'bold'),
            fg='white', bg='#303134',
            anchor='center',
        )
        self.current_label.grid(
            row=0, column=0, sticky='nsew', padx=(12, 6), pady=8)

        self.operation_state_var = tk.StringVar(value='ERROR')
        self.operation_state_label = tk.Label(
            self.footer,
            textvariable=self.operation_state_var,
            font=('Sans', 56, 'bold'),
            fg='#e45b5b', bg='#303134',
            anchor='center',
        )
        self.operation_state_label.grid(
            row=0, column=1, sticky='nsew', padx=(6, 12), pady=8)

        self.root.grid_rowconfigure(0, weight=8)
        self.root.grid_rowconfigure(1, weight=2)
        self.root.grid_columnconfigure(0, weight=1)

        self.root.update_idletasks()
        if self.full_screen:
            self.root.attributes('-fullscreen', True)

        self.root.after(50, self._process_ros_events)
        self.get_logger().info(
            f'Target GUI ready: service={self.service_name}, '
            f'state_topic={self.state_topic}, '
            f'operation_state_topic={self.operation_state_topic}')

    def _on_current_state(self, message):
        if math.isfinite(message.data):
            self.event_queue.put(('current_state', float(message.data)))

    def _on_operation_state(self, message):
        if message.data in (0, 1, 2):
            self.event_queue.put((
                'operation_state', (int(message.data), time.monotonic())))

    def _set_operation_state(self, state):
        state_display = {
            0: ('ERROR', '#e45b5b'),
            1: ('WAIT', '#e0b45b'),
            2: ('OK', '#55b878'),
        }
        text, color = state_display.get(state, state_display[0])
        self.operation_state_var.set(text)
        self.operation_state_label.configure(fg=color)

    def _log_gui_only_action(self, action_name):
        """Record a GUI-only button press without publishing ROS commands."""
        self.get_logger().info(
            f'GUI-only button pressed: {action_name}; no ROS/Action/CAN command sent')

    def _send_operation(self, operation):
        if self.request_in_flight:
            return
        if not self.client.service_is_ready():
            self.get_logger().warning(
                f'Control service is unavailable: {self.service_name}')
            return

        request = ManualCurrent.Request()
        request.operation = int(operation)
        self.request_in_flight = True
        self._set_buttons_enabled(False)

        future = self.client.call_async(request)
        future.add_done_callback(self._on_service_done)

    def _on_service_done(self, future):
        try:
            self.event_queue.put(('service_result', future.result()))
        except Exception as exc:
            self.event_queue.put(('service_error', str(exc)))

    def _process_ros_events(self):
        try:
            while True:
                event, payload = self.event_queue.get_nowait()
                if event == 'current_state':
                    self.current_var.set(f'{payload:.2f} A')
                elif event == 'operation_state':
                    state, received_at = payload
                    self.last_operation_state_time = received_at
                    self._set_operation_state(state)
                elif event == 'service_result':
                    self.request_in_flight = False
                    self._set_buttons_enabled(True)
                    if math.isfinite(payload.current_a):
                        self.current_var.set(f'{payload.current_a:.2f} A')
                    if payload.success:
                        self.get_logger().info(
                            f'Current setpoint updated: {payload.current_a:.2f} A')
                    else:
                        self.get_logger().warning(
                            f'Current setting rejected: {payload.message}')
                elif event == 'service_error':
                    self.request_in_flight = False
                    self._set_buttons_enabled(True)
                    self.get_logger().error(
                        f'Manual-current service call failed: {payload}')
        except queue.Empty:
            pass

        if self.last_operation_state_time is None or \
                time.monotonic() - self.last_operation_state_time > \
                self.operation_state_timeout_sec:
            self._set_operation_state(0)

        self.root.after(50, self._process_ros_events)

    def _set_buttons_enabled(self, enabled):
        state = tk.NORMAL if enabled else tk.DISABLED
        for button in (*self.fixed_buttons, self.up_button, self.down_button):
            button.configure(state=state)

    def toggle_full_screen(self):
        current = bool(self.root.attributes('-fullscreen'))
        self.root.attributes('-fullscreen', not current)

    def close_window(self):
        try:
            self.root.quit()
            self.root.destroy()
        except tk.TclError:
            pass

    def run_gui(self):
        self.root.mainloop()


def _spin_ros(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    spin_thread = None
    try:
        node = TargetGui()
        spin_thread = threading.Thread(
            target=_spin_ros, args=(node,), daemon=True)
        spin_thread.start()
        node.run_gui()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
