import math
import queue
import threading
import tkinter as tk

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from shoot_belt.srv import ManualCurrent
from sequence_interfaces.action import Sequence

from std_msgs.msg import Bool, Empty, Float32, UInt8

from std_srvs.srv import Trigger


class TargetGui(Node):
    """Touch GUI for configuring the current used by the next shot."""

    def __init__(self):
        super().__init__('target_gui')

        self.declare_parameter('service_name', '/shoot_belt/manual_current')
        self.declare_parameter('stop_topic', '/shoot_belt/stop')
        self.declare_parameter(
            'reset_origin_service_name', '/shoot_belt/reset_origin')
        self.declare_parameter(
            'state_topic', '/shoot_belt/manual_current_state')
        self.declare_parameter(
            'operation_state_topic', '/shoot_belt/operation_state')
        self.declare_parameter('reload_execute_action_name', '/reload/execute')
        self.declare_parameter(
            'reload_reset_origin_action_name', '/reload/reset_origin')
        self.declare_parameter('reload_ready_topic', '/reload/ready')
        self.declare_parameter('full_screen', True)

        self.service_name = str(self.get_parameter('service_name').value)
        self.stop_topic = str(self.get_parameter('stop_topic').value)
        self.reset_origin_service_name = str(
            self.get_parameter('reset_origin_service_name').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.operation_state_topic = str(
            self.get_parameter('operation_state_topic').value)
        self.reload_execute_action_name = str(
            self.get_parameter('reload_execute_action_name').value)
        self.reload_reset_origin_action_name = str(
            self.get_parameter('reload_reset_origin_action_name').value)
        self.reload_ready_topic = str(
            self.get_parameter('reload_ready_topic').value)
        self.full_screen = bool(self.get_parameter('full_screen').value)

        self.event_queue = queue.Queue()
        self.request_in_flight = False
        self.reset_request_in_flight = False
        self.resetting_state_seen = False
        self.reset_request_id = 0
        self.reset_status_after_id = None
        self.operation_request_timeout_after_id = None
        self.operation_state = None
        self.reload_ready = False
        self.reload_request_in_flight = False
        self.reload_request_id = 0
        self.reset_shoot_done = False
        self.reset_shoot_success = False
        self.reset_shoot_message = ''
        self.reset_reload_done = False
        self.reset_reload_success = False
        self.reset_reload_message = ''
        self.reload_reset_retry_needed = False

        self.client = self.create_client(ManualCurrent, self.service_name)
        self.stop_publisher = self.create_publisher(Empty, self.stop_topic, 10)
        self.reset_origin_client = self.create_client(
            Trigger, self.reset_origin_service_name)
        self.reload_execute_client = ActionClient(
            self, Sequence, self.reload_execute_action_name)
        self.reload_reset_origin_client = ActionClient(
            self, Sequence, self.reload_reset_origin_action_name)
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_sub = self.create_subscription(
            Float32, self.state_topic, self._on_current_state, state_qos)
        self.operation_state_sub = self.create_subscription(
            UInt8, self.operation_state_topic, self._on_operation_state, state_qos)
        self.reload_ready_sub = self.create_subscription(
            Bool, self.reload_ready_topic, self._on_reload_ready, state_qos)

        self.root = tk.Tk()
        self.root.title('Target GUI - 射出電流設定')
        self.root.configure(bg='#202124')
        self.root.geometry('1280x800')
        self.root.minsize(800, 480)
        self.root.protocol('WM_DELETE_WINDOW', self.close_window)
        self.root.bind('<Escape>', lambda _event: self.close_window())
        self.root.bind('<F11>', lambda _event: self.toggle_full_screen())

        # Split the screen into left and right halves. The fixed buttons
        # span the full height of the left half.
        self.root.grid_rowconfigure(0, weight=7)
        self.root.grid_rowconfigure(1, weight=3)
        self.root.grid_columnconfigure(0, weight=1, uniform='screen')
        self.root.grid_columnconfigure(1, weight=1, uniform='screen')

        self.fixed_frame = tk.Frame(self.root, bg='#202124')
        self.fixed_frame.grid(
            row=0, column=0, rowspan=2, sticky='nsew',
            padx=(10, 5), pady=10)
        fixed_row_weights = (7, 7, 7, 7, 12)
        for row, weight in enumerate(fixed_row_weights):
            self.fixed_frame.grid_rowconfigure(
                row, weight=weight, uniform='fixed_bucket')
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
                pady=(0, 5) if row < 3 else (0, 1))
            self.fixed_buttons.append(button)

        self.reload_button = tk.Button(
            self.fixed_frame,
            text='RELOAD',
            command=self._send_reload,
            font=('Sans', 22, 'bold'),
            bg='#75549a', fg='white',
            activebackground='#9069b8', activeforeground='white',
            disabledforeground='#9aa0a6',
            relief=tk.FLAT, bd=0, takefocus=False,
        )
        self.reload_button.grid(
            row=4, column=0, sticky='nsew', pady=(0, 0))
        self.reload_button.configure(state=tk.DISABLED)

        # The right half keeps the existing brake/reset and up/down controls
        # above the current and operation-state indicators.
        self.main_frame = tk.Frame(self.root, bg='#202124')
        self.main_frame.grid(
            row=0, column=1, sticky='nsew', padx=(5, 10), pady=(10, 5))
        # A shared 3:1 row split keeps the lower edge of the UP/DOWN
        # group exactly aligned with the upper edge of the RESET frame.
        self.main_frame.grid_rowconfigure(0, weight=3, uniform='control_area')
        self.main_frame.grid_rowconfigure(1, weight=1, uniform='control_area')
        self.main_frame.grid_columnconfigure(0, weight=5, uniform='main')
        self.main_frame.grid_columnconfigure(1, weight=10, uniform='main')

        # CANCEL occupies the upper 3/4 of the shared control area.
        # RESET occupies the lower 1/4; this is the same 3:1 ratio as before.
        self.brake_button = tk.Button(
            self.main_frame,
            text='CANCEL',
            command=self._send_brake,
            font=('Sans', 26, 'bold'),
            bg='#5f6368', fg='white',
            activebackground='#777b80', activeforeground='white',
            relief=tk.FLAT, bd=0, takefocus=False,
        )
        self.brake_button.grid(
            row=0, column=0, sticky='nsew', padx=6, pady=(0, 5))

        # Draw one persistent outer frame around both the RESET button and
        # its lower result-display area. The highlight border reserves its own
        # space, so the child widgets cannot cover the bottom edge.
        self.reset_container = tk.Frame(
            self.main_frame,
            bg='#5f6368',
            bd=0,
            relief=tk.FLAT,
            highlightthickness=2,
            highlightbackground='#9aa0a6',
            highlightcolor='#9aa0a6',
        )
        self.reset_container.grid(
            row=1, column=0, sticky='nsew', padx=6, pady=(0, 0))

        self.reset_button = tk.Button(
            self.reset_container,
            text='RESET',
            command=self._send_reset_origin,
            font=('Sans', 26, 'bold'),
            bg='#5f6368', fg='white',
            activebackground='#777b80', activeforeground='white',
            relief=tk.FLAT, bd=0, highlightthickness=0, takefocus=False,
        )
        # Keep the button inside the outer frame's 2 px border. Disabling its
        # own focus highlight prevents a second line along the upper edge.
        self.reset_button.place(
            x=2, y=2,
            relwidth=1.0, width=-4,
            relheight=1.0, height=-4)

        # This label overlays only the lower edge of the button. The RESET
        # button text itself never changes or moves while showing the result.
        self.reset_result_label = tk.Label(
            self.reset_container,
            text='',
            font=('Sans', 11, 'bold'),
            fg='#d9dce0', bg='#5f6368',
            anchor='center',
            takefocus=False,
        )
        self.reset_result_label.place(
            relx=0.5, rely=1.0, y=-2, anchor='s',
            relwidth=1.0, width=-4, height=22)
        self.reset_result_label.bind(
            '<Button-1>', lambda _event: self._send_reset_origin())
        self.reset_button.configure(state=tk.DISABLED)

        self.right_frame = tk.Frame(self.main_frame, bg='#202124')
        self.right_frame.grid(
            row=0, column=1, sticky='nsew', padx=(6, 0), pady=0)
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

        # Current and operation-state indicators share the bottom area
        # of the right half.
        self.footer = tk.Frame(self.root, bg='#303134')
        self.footer.grid(
            row=1, column=1, sticky='nsew', padx=(5, 10), pady=(5, 10))
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

        self.operation_state_var = tk.StringVar(value='--')
        self.operation_state_label = tk.Label(
            self.footer,
            textvariable=self.operation_state_var,
            font=('Sans', 56, 'bold'),
            fg='#9aa0a6', bg='#303134',
            anchor='center',
        )
        self.operation_state_label.grid(
            row=0, column=1, sticky='nsew', padx=(6, 12), pady=8)

        self._update_reload_button_state()
        self.root.update_idletasks()
        if self.full_screen:
            self.root.attributes('-fullscreen', True)

        self.root.after(50, self._process_ros_events)
        self.get_logger().info(
            f'Target GUI ready: current_service={self.service_name}, '
            f'stop_topic={self.stop_topic}, '
            f'reset_origin_service={self.reset_origin_service_name}, '
            f'state_topic={self.state_topic}, '
            f'operation_state_topic={self.operation_state_topic}, '
            f'reload_execute_action={self.reload_execute_action_name}, '
            f'reload_reset_action={self.reload_reset_origin_action_name}, '
            f'reload_ready_topic={self.reload_ready_topic}')

    def _on_current_state(self, message):
        if math.isfinite(message.data):
            self.event_queue.put(('current_state', float(message.data)))

    def _on_operation_state(self, message):
        if message.data in (0, 1, 2, 3, 4, 5):
            self.event_queue.put(('operation_state', int(message.data)))

    def _on_reload_ready(self, message):
        self.event_queue.put(('reload_ready', bool(message.data)))

    def _set_operation_state(self, state):
        state_display = {
            0: ('ERROR', '#e45b5b', 56),
            1: ('WAIT', '#e0b45b', 56),
            2: ('OK', '#55b878', 56),
            3: ('CANCELLED', '#e0b45b', 36),
            4: ('RESETTING', '#e0b45b', 36),
            5: ('RESET FAILED', '#e45b5b', 32),
        }
        if state not in state_display:
            return

        text, color, font_size = state_display[state]
        self.operation_state = state
        self.operation_state_var.set(text)
        self.operation_state_label.configure(
            fg=color, font=('Sans', font_size, 'bold'))
        if state != 2:
            self.reload_ready = False

        # Only the shoot_belt reset transition is a fallback for its own
        # service result. RESET is complete only after both mechanisms finish.
        if self.reset_request_in_flight:
            if state == 4:
                self.resetting_state_seen = True
            elif self.resetting_state_seen and state == 2:
                self._mark_shoot_reset_result(
                    self.reset_request_id, True,
                    'RESETTING -> READY state transition received')
            elif self.resetting_state_seen and state == 5:
                self._mark_shoot_reset_result(
                    self.reset_request_id, False,
                    'RESETTING -> RESET_FAILED state transition received')

        self._update_reset_button_state()
        self._update_operation_buttons_state()
        self._update_reload_button_state()

    def _update_reset_button_state(self):
        retry_reload_only = (
            self.reload_reset_retry_needed and self.operation_state == 2)
        reset_allowed = self.operation_state in (0, 3, 5) or retry_reload_only
        enabled = (reset_allowed and not self.reset_request_in_flight and
                   not self.reload_request_in_flight)
        self.reset_button.configure(
            state=tk.NORMAL if enabled else tk.DISABLED)

    def _update_reload_button_state(self):
        enabled = (
            self.reload_ready and
            self.operation_state == 2 and
            not self.reload_request_in_flight and
            not self.reset_request_in_flight
        )
        self.reload_button.configure(
            state=tk.NORMAL if enabled else tk.DISABLED)

    def _set_reset_result(self, text, color='#d9dce0'):
        self.reset_result_label.configure(text=text, fg=color)

    def _cancel_reset_status_timer(self):
        if self.reset_status_after_id is not None:
            try:
                self.root.after_cancel(self.reset_status_after_id)
            except tk.TclError:
                pass
            self.reset_status_after_id = None

    def _clear_reset_status(self):
        self.reset_status_after_id = None
        if not self.reset_request_in_flight:
            self.reset_result_label.configure(text='')

    def _display_reset_result(self, success, message, service_error=False):
        """Display reset result temporarily without moving the RESET caption."""
        self.reset_request_in_flight = False
        self.resetting_state_seen = False
        if success:
            status_text = 'COMPLETE'
            status_color = '#55b878'
            display_duration_ms = 2500
            self.get_logger().info(
                f'Origin initialization succeeded: {message}')
        else:
            status_text = 'SERVICE ERROR' if service_error else 'FAILED'
            status_color = '#e45b5b'
            display_duration_ms = 5000
            self.get_logger().error(
                f'Origin initialization failed: {message}')

        self.reset_result_label.configure(
            text=status_text, fg=status_color)
        self._cancel_reset_status_timer()
        self.reset_status_after_id = self.root.after(
            display_duration_ms, self._clear_reset_status)
        self._update_reset_button_state()
        self._update_operation_buttons_state()

    def _send_brake(self):
        """Request brake mode through shoot_belt's existing stop topic."""
        message = Empty()
        self.stop_publisher.publish(message)
        self.get_logger().warning(
            f'Brake/stop request published to {self.stop_topic}')

    def _send_reset_origin(self):
        """Initialize both origins, retrying only reload if shoot reset already succeeded."""
        retry_reload_only = (
            self.reload_reset_retry_needed and self.operation_state == 2)
        if (self.reset_request_in_flight or self.reload_request_in_flight or
                (self.operation_state not in (0, 3, 5) and not retry_reload_only)):
            return

        self._cancel_reset_status_timer()
        self.reset_request_id += 1
        request_id = self.reset_request_id
        self.resetting_state_seen = False
        self.reset_request_in_flight = True
        self.reset_shoot_done = retry_reload_only
        self.reset_shoot_success = retry_reload_only
        self.reset_shoot_message = (
            'shoot_belt origin already initialized; retrying reload only'
            if retry_reload_only else '')
        self.reset_reload_done = False
        self.reset_reload_success = False
        self.reset_reload_message = ''
        self.reload_reset_retry_needed = False
        self.reload_ready = False
        self.reset_button.configure(state=tk.DISABLED)
        self._update_operation_buttons_state()
        self._update_reload_button_state()
        self._set_reset_result(
            'RESETTING RELOAD...' if retry_reload_only else 'RESETTING BOTH...',
            '#e0b45b')

        # Existing shoot_belt initialization. Retry only the failed reload
        # half when the shooting origin already succeeded and remains READY.
        if retry_reload_only:
            pass
        elif self.reset_origin_client.service_is_ready():
            try:
                future = self.reset_origin_client.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda completed_future, request_id=request_id:
                        self._on_reset_origin_done(completed_future, request_id))
            except Exception as exc:
                self._mark_shoot_reset_result(
                    request_id, False, f'shoot_belt service call failed: {exc}')
        else:
            self._mark_shoot_reset_result(
                request_id, False,
                f'shoot_belt reset service unavailable: {self.reset_origin_service_name}')

        # Reload origin initialization is an independent Sequence Action.
        if self.reload_reset_origin_client.server_is_ready():
            goal = Sequence.Goal()
            goal.operation = Sequence.Goal.OPERATION_HOME
            try:
                future = self.reload_reset_origin_client.send_goal_async(goal)
                future.add_done_callback(
                    lambda completed_future, request_id=request_id:
                        self._on_reload_reset_goal_response(completed_future, request_id))
            except Exception as exc:
                self._mark_reload_reset_result(
                    request_id, False, f'reload reset Action call failed: {exc}')
        else:
            self._mark_reload_reset_result(
                request_id, False,
                f'reload reset Action server unavailable: {self.reload_reset_origin_action_name}')

    def _on_reset_origin_done(self, future, request_id):
        try:
            response = future.result()
            self.event_queue.put((
                'shoot_reset_result',
                (request_id, bool(response.success), str(response.message))))
        except Exception as exc:
            self.event_queue.put(('shoot_reset_error', (request_id, str(exc))))

    def _on_reload_reset_goal_response(self, future, request_id):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.event_queue.put((
                    'reload_reset_result',
                    (request_id, False, 'reload reset Action goal was rejected')))
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed_future, request_id=request_id:
                    self._on_reload_reset_result(completed_future, request_id))
        except Exception as exc:
            self.event_queue.put((
                'reload_reset_result',
                (request_id, False, f'reload reset Action failed: {exc}')))

    def _on_reload_reset_result(self, future, request_id):
        try:
            wrapped = future.result()
            result = wrapped.result
            success = (wrapped.status == GoalStatus.STATUS_SUCCEEDED and
                       bool(result.success))
            message = str(result.reason)
            self.event_queue.put((
                'reload_reset_result', (request_id, success, message)))
        except Exception as exc:
            self.event_queue.put((
                'reload_reset_result',
                (request_id, False, f'reload reset result error: {exc}')))

    def _mark_shoot_reset_result(self, request_id, success, message):
        if request_id != self.reset_request_id or not self.reset_request_in_flight:
            return
        if self.reset_shoot_done:
            return
        self.reset_shoot_done = True
        self.reset_shoot_success = bool(success)
        self.reset_shoot_message = str(message)
        self._try_finish_combined_reset()

    def _mark_reload_reset_result(self, request_id, success, message):
        if request_id != self.reset_request_id or not self.reset_request_in_flight:
            return
        if self.reset_reload_done:
            return
        self.reset_reload_done = True
        self.reset_reload_success = bool(success)
        self.reset_reload_message = str(message)
        self._try_finish_combined_reset()

    def _try_finish_combined_reset(self):
        if not (self.reset_shoot_done and self.reset_reload_done):
            return
        success = self.reset_shoot_success and self.reset_reload_success
        self.reload_reset_retry_needed = (
            self.reset_shoot_success and not self.reset_reload_success)
        message = (
            f'shoot_belt: {self.reset_shoot_message}; '
            f'reload: {self.reset_reload_message}')
        self._display_reset_result(success, message)

    def _send_operation(self, operation):
        # Do not allow setpoint changes while origin initialization is active.
        if (self.request_in_flight or self.reset_request_in_flight or
                self.operation_state == 4):
            return
        if not self.client.service_is_ready():
            self.get_logger().warning(
                f'Control service is unavailable: {self.service_name}')
            return

        request = ManualCurrent.Request()
        request.operation = int(operation)
        self.request_in_flight = True
        self._update_operation_buttons_state()

        future = self.client.call_async(request)
        future.add_done_callback(self._on_service_done)
        # Report a stalled service response, but do not release the lock:
        # an INCREASE/DECREASE request may still have been applied server-side.
        self.operation_request_timeout_after_id = self.root.after(
            5000, self._on_operation_request_timeout)

    def _on_service_done(self, future):
        try:
            self.event_queue.put(('service_result', future.result()))
        except Exception as exc:
            self.event_queue.put(('service_error', str(exc)))

    def _send_reload(self):
        if (not self.reload_ready or self.operation_state != 2 or
                self.reload_request_in_flight or self.reset_request_in_flight):
            return
        if not self.reload_execute_client.server_is_ready():
            self.get_logger().warning(
                f'RELOAD Action server is unavailable: {self.reload_execute_action_name}')
            return

        self.reload_request_id += 1
        request_id = self.reload_request_id
        goal = Sequence.Goal()
        goal.operation = Sequence.Goal.OPERATION_PROCEED
        self.reload_request_in_flight = True
        self._update_reload_button_state()
        self._update_reset_button_state()
        try:
            future = self.reload_execute_client.send_goal_async(goal)
            future.add_done_callback(
                lambda completed_future, request_id=request_id:
                    self._on_reload_execute_goal_response(completed_future, request_id))
        except Exception as exc:
            self.event_queue.put(('reload_execute_error', (request_id, str(exc))))

    def _on_reload_execute_goal_response(self, future, request_id):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.event_queue.put((
                    'reload_execute_rejected',
                    (request_id, 'RELOAD Action goal was rejected by belt_reload')))
                return
            self.event_queue.put(('reload_execute_accepted', request_id))
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda completed_future, request_id=request_id:
                    self._on_reload_execute_result(completed_future, request_id))
        except Exception as exc:
            self.event_queue.put(('reload_execute_error', (request_id, str(exc))))

    def _on_reload_execute_result(self, future, request_id):
        try:
            wrapped = future.result()
            result = wrapped.result
            success = (wrapped.status == GoalStatus.STATUS_SUCCEEDED and
                       bool(result.success))
            self.event_queue.put((
                'reload_execute_result',
                (request_id, success, str(result.reason))))
        except Exception as exc:
            self.event_queue.put(('reload_execute_error', (request_id, str(exc))))

    def _process_ros_events(self):
        try:
            while True:
                event, payload = self.event_queue.get_nowait()
                if event == 'current_state':
                    self.current_var.set(f'{payload:.2f} A')
                elif event == 'operation_state':
                    self._set_operation_state(payload)
                elif event == 'service_result':
                    self.request_in_flight = False
                    self._cancel_operation_request_timeout()
                    self._update_operation_buttons_state()
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
                    self._cancel_operation_request_timeout()
                    self._update_operation_buttons_state()
                    self.get_logger().error(
                        f'Manual-current service call failed: {payload}')
                elif event == 'shoot_reset_result':
                    request_id, success, message = payload
                    self._mark_shoot_reset_result(request_id, success, message)
                elif event == 'shoot_reset_error':
                    request_id, error = payload
                    self._mark_shoot_reset_result(
                        request_id, False, f'shoot_belt reset service error: {error}')
                elif event == 'reload_reset_result':
                    request_id, success, message = payload
                    self._mark_reload_reset_result(request_id, success, message)
                elif event == 'reload_ready':
                    self.reload_ready = bool(payload)
                    self._update_reload_button_state()
                elif event == 'reload_execute_accepted':
                    request_id = payload
                    if request_id == self.reload_request_id:
                        self.reload_ready = False
                        self._update_reload_button_state()
                elif event == 'reload_execute_rejected':
                    request_id, message = payload
                    if request_id == self.reload_request_id:
                        self.reload_request_in_flight = False
                        # A rejected Goal was not accepted; its one-shot
                        # permission remains available for a retry.
                        self.get_logger().warning(message)
                        self._update_reload_button_state()
                        self._update_reset_button_state()
                elif event == 'reload_execute_result':
                    request_id, success, message = payload
                    if request_id == self.reload_request_id:
                        self.reload_request_in_flight = False
                        self.reload_ready = False
                        if success:
                            self.get_logger().info(f'RELOAD completed: {message}')
                        else:
                            self.get_logger().error(f'RELOAD failed: {message}')
                        self._update_reload_button_state()
                        self._update_reset_button_state()
                elif event == 'reload_execute_error':
                    request_id, error = payload
                    if request_id == self.reload_request_id:
                        self.reload_request_in_flight = False
                        self.reload_ready = False
                        self.get_logger().error(f'RELOAD Action error: {error}')
                        self._update_reload_button_state()
                        self._update_reset_button_state()
        except queue.Empty:
            pass

        self.root.after(50, self._process_ros_events)

    def _update_operation_buttons_state(self):
        # RESETTING has priority over a manual-service completion event.
        enabled = (
            not self.request_in_flight and
            not self.reset_request_in_flight and
            self.operation_state != 4
        )
        state = tk.NORMAL if enabled else tk.DISABLED
        for button in (*self.fixed_buttons, self.up_button, self.down_button):
            button.configure(state=state)
        self._update_reload_button_state()

    def _cancel_operation_request_timeout(self):
        if self.operation_request_timeout_after_id is not None:
            try:
                self.root.after_cancel(self.operation_request_timeout_after_id)
            except tk.TclError:
                pass
            self.operation_request_timeout_after_id = None

    def _on_operation_request_timeout(self):
        self.operation_request_timeout_after_id = None
        if self.request_in_flight:
            self.get_logger().error(
                'Manual-current service response has not arrived after 5 seconds; '
                'operation buttons remain disabled because the request outcome is unknown')

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
