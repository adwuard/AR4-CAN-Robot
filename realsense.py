from time import sleep

from dualsense_controller import DualSenseController

# list availabe devices and throw exception when tzhere is no device detected
device_infos = DualSenseController.enumerate_devices()
if len(device_infos) < 1:
    raise Exception('No DualSense Controller available.')

# flag, which keeps program alive
is_running = True

# create an instance, use fiŕst available device
controller = DualSenseController()


# switches the keep alive flag, which stops the below loop
def stop():
    global is_running
    is_running = False


# callback, when cross button is pressed, which enables rumble
def on_cross_btn_pressed():
    print('cross button pressed')
    controller.left_rumble.set(255)
    controller.right_rumble.set(255)


# callback, when cross button is released, which disables rumble
def on_cross_btn_released():
    print('cross button released')
    controller.left_rumble.set(0)
    controller.right_rumble.set(0)


# callback, when PlayStation button is pressed
# stop program
def on_ps_btn_pressed():
    print('PS button released -> stop')
    stop()


# callback, when unintended error occurs,
# i.e. physically disconnecting the controller during operation
# stop program
def on_error(error):
    print(f'Opps! an error occured: {error}')
    stop()


# register the button callbacks
controller.btn_cross.on_down(on_cross_btn_pressed)
controller.btn_cross.on_up(on_cross_btn_released)
controller.btn_ps.on_down(on_ps_btn_pressed)


controller.lightbar.set_color(88, 10, 200)

# controller.player_leds.set_all()

def on_left_trigger(value):
    print(f'left trigger changed: {value}')


def on_left_stick_x_changed(left_stick_x):
    print(f'on_left_stick_x_changed: {left_stick_x}')


def on_left_stick_y_changed(left_stick_y):
    print(f'on_left_stick_y_changed: {left_stick_y}')


def on_left_stick_changed(left_stick):
    print(f'on_left_stick_changed: {left_stick}')

# controller.left_trigger.effect.continuous_resistance(start_position=100, force=255)  # full resistance
# controller.left_trigger.effect.feedback(start_position=0, strength=8)  # full resistance

# controller.left_trigger.effect.slope_feedback(start_position=0, end_position=9, start_strength=1, end_strength=4) 
# controller.left_trigger.effect.section_resistance(start_position=70, end_position=100, force=255)  # full
# controller.left_trigger.effect.weapon(start_position=2, end_position=5, strength=8) 
# controller.left_trigger.effect.bow(start_position=1, end_position=4, strength=1, snap_force=8) 
# controller.left_trigger.effect.bow(start_position=0, end_position=9, first_foot=4, second_foot=7, frequency=2) 
# controller.left_trigger.effect.machine(
#     start_position=1,
#     end_position=9,
#     amplitude_a=2,
#     amplitude_b=7,
#     frequency=5,
#     period=3
# )



controller.left_trigger.on_change(on_left_trigger)
controller.left_stick_x.on_change(on_left_stick_x_changed)
controller.left_stick_y.on_change(on_left_stick_y_changed)
controller.left_stick.on_change(on_left_stick_changed)


# register the error callback
controller.on_error(on_error)

# enable/connect the device
controller.activate()

# start keep alive loop, controller inputs and callbacks are handled in a second thread
# while is_running:
    # sleep(0.001)



import PySimpleGUI as psg
layout = [
#    [psg.Text('Hello World', enable_events=True,
#    key='-TEXT-', font=('Arial Bold', 20),
#    size=(50, 2), relief="raised", border_width=5,
#    expand_x=True, justification='center')],

    [psg.Text("Joint 1: ",font=('Arial Bold', 15)), psg.Slider(range=(10, 30), default_value=12,
    expand_x=True, enable_events=True,
    orientation='horizontal', key='-SL-')],
    [psg.Slider(range=(10, 30), default_value=12,
    expand_x=True, enable_events=True,
    orientation='horizontal', key='-SL-')],
    [psg.Slider(range=(10, 30), default_value=12,
    expand_x=True, enable_events=True,
    orientation='horizontal', key='-SL-')],
    [psg.Slider(range=(10, 30), default_value=12,
    expand_x=True, enable_events=True,
    orientation='horizontal', key='-SL-')],
    [psg.Slider(range=(10, 30), default_value=12,
    expand_x=True, enable_events=True,
    orientation='horizontal', key='-SL-')],
    [psg.Slider(range=(10, 30), default_value=12,
    expand_x=True, enable_events=True,
    orientation='horizontal', key='-SL-')],
]
window = psg.Window('Hello', layout)
while is_running:
    event, values = window.read(timeout=100)
    print(event, values)
    if event == psg.WIN_CLOSED or event == 'Exit':
        break
    
    # window['-SL-'].update(20)
    
#    if event == '-SL-':
    #   window['-TEXT-'].update(font=('Arial Bold', int(values['-SL-'])))
window.close()


# disable/disconnect controller device
controller.deactivate()