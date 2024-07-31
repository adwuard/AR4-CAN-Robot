
class DualSense:
    def __init__(self, event_cb) -> None:
        from dualsense_controller import DualSenseController

        self.device_infos = DualSenseController.enumerate_devices()
        if len(self.device_infos) < 1:
            raise Exception("No DualSense Controller available.")

        self.is_running = True
        self.controller = DualSenseController()

        self.layer = 1

        self.cb = event_cb
        self.joystick_thresh = 0.6
        self.trigger_thresh = 0.2
        self.latch_hold_repeat_time = 0.1

        self.left_trigger_latch_time = time.time()
        self.right_trigger_latch_time = time.time()

        self.left_joystick_latch_time = time.time()
        self.right_joystick_latch_time = time.time()

        self.left_joystick_latch = False
        self.right_joystick_latch = False
        self.left_trigger_latch = False
        self.right_trigger_latch = False

        ctl = self.controller
        """ Right D Pad buttons"""
        ctl.btn_cross.on_change(self.on_cross_btn_on_change)
        ctl.btn_square.on_change(self.on_square_btn_on_change)
        ctl.btn_circle.on_change(self.on_circle_btn_on_change)
        ctl.btn_triangle.on_change(self.on_triangle_btn_on_change)

        """ Left D Pad buttons"""
        ctl.btn_left.on_change(self.on_left_btn_on_change)
        ctl.btn_right.on_change(self.on_right_btn_on_change)
        ctl.btn_up.on_change(self.on_up_btn_on_change)
        ctl.btn_down.on_change(self.on_down_btn_on_change)

        """ Functional buttons"""
        ctl.btn_ps.on_down(self.on_ps_btn_pressed)

        ctl.left_trigger.on_change(self.on_left_trigger)
        ctl.right_trigger.on_change(self.on_right_trigger)

        ctl.left_stick.on_change(self.on_left_stick_changed)
        ctl.right_stick.on_change(self.on_right_stick_changed)

        ctl.on_error(self.on_error)

        # ctl.lightbar.set_color(88, 10, 200)
        self.controller.lightbar.set_color_red()

        ctl.activate()

    def __del__(self):
        self.controller.deactivate()
        pass

    def stop(self):
        global is_running
        is_running = False

    def on_error(self, error):
        print(f"Opps! an error occured: {error}")
        self.stop()

    def toggle_layer(self):
        if self.layer == 1:
            self.layer = 2
            self.controller.lightbar.set_color_blue()
        else:
            self.layer = 1
            self.controller.lightbar.set_color_red()

    def unified_event_proxy(self, event):
        try:
            self.cb([self.layer, event])
        except Exception as e:
            print(e)

    def on_cross_btn_on_change(self, status):
        self.unified_event_proxy({"cross-btn": status})

    def on_square_btn_on_change(self, status):
        self.unified_event_proxy({"square-btn": status})

    def on_circle_btn_on_change(self, status):
        self.unified_event_proxy({"circle-btn": status})

    def on_triangle_btn_on_change(self, status):
        self.unified_event_proxy({"triangle-btn": status})

    def on_left_btn_on_change(self, status):
        self.unified_event_proxy({"left-btn": status})

    def on_right_btn_on_change(self, status):
        self.unified_event_proxy({"right-btn": status})

    def on_up_btn_on_change(self, status):
        self.unified_event_proxy({"up-btn": status})

    def on_down_btn_on_change(self, status):
        self.unified_event_proxy({"down-btn": status})

    def on_cross_btn_released(self):
        print("cross button released")
        # self.controller.left_rumble.set(0)
        # self.controller.right_rumble.set(0)

    def on_left_trigger(self, value):
        if time.time() - self.left_trigger_latch_time > self.latch_hold_repeat_time:
            self.left_trigger_latch = False
            self.left_trigger_latch_time = time.time()

        if value > self.trigger_thresh and not self.left_trigger_latch:
            self.unified_event_proxy({"left-trigger": True})
            self.left_trigger_latch = True
        else:
            self.left_trigger_latch = False

        return

    def on_right_trigger(self, value):
        if time.time() - self.right_trigger_latch_time > self.latch_hold_repeat_time:
            self.right_trigger_latch = False
            self.right_trigger_latch_time = time.time()

        if value > self.trigger_thresh and not self.left_trigger_latch:
            self.unified_event_proxy({"right-trigger": True})
            self.right_trigger_latch = True
        else:
            self.right_trigger_latch = False

        return

    def on_left_stick_changed(self, left_stick):
        # timestamp compare, if time diff over 500ms, then set latch to false
        if time.time() - self.left_joystick_latch_time > self.latch_hold_repeat_time:
            self.left_joystick_latch = False
            self.left_joystick_latch_time = time.time()

        if abs(left_stick.x) > self.joystick_thresh and not self.left_joystick_latch:
            self.left_joystick_latch = True
            if left_stick.x < 0:
                self.unified_event_proxy({"left-joystick-x": -1})
            else:
                self.unified_event_proxy({"left-joystick-x": 1})
        elif abs(left_stick.x) < self.joystick_thresh and self.left_joystick_latch:
            self.left_joystick_latch = False

        if abs(left_stick.y) > self.joystick_thresh and not self.left_joystick_latch:
            self.left_joystick_latch = True
            if left_stick.y < 0:
                self.unified_event_proxy({"left-joystick-y": -1})
            else:
                self.unified_event_proxy({"left-joystick-y": 1})
        elif abs(left_stick.x) < self.joystick_thresh and self.left_joystick_latch:
            self.left_joystick_latch = False

    def on_right_stick_changed(self, right_stick):
        # timestamp compare, if time diff over 500ms, then set latch to false
        if time.time() - self.right_joystick_latch_time > self.latch_hold_repeat_time:
            self.right_joystick_latch = False
            self.right_joystick_latch_time = time.time()

        if abs(right_stick.x) > self.joystick_thresh and not self.right_trigger_latch:
            self.right_trigger_latch = True
            if right_stick.x < 0:
                self.unified_event_proxy({"right-joystick-x": -1})
            else:
                self.unified_event_proxy({"right-joystick-x": 1})
        elif abs(right_stick.x) < self.joystick_thresh and self.right_trigger_latch:
            self.right_trigger_latch = False

        if abs(right_stick.y) > self.joystick_thresh and not self.right_trigger_latch:
            self.right_trigger_latch = True
            if right_stick.y < 0:
                self.unified_event_proxy({"right-joystick-y": -1})
            else:
                self.unified_event_proxy({"right-joystick-y": 1})
        elif abs(right_stick.x) < self.joystick_thresh and self.right_trigger_latch:
            self.right_trigger_latch = False

    # stop program
    def on_ps_btn_pressed(self):
        # print('PS button released -> stop')
        # self.stop()
        self.toggle_layer()
