import time
import threading
from dash import html, Input, Output, State
from components import RVizSection
from page_draw_mode.function_draw_mode import *
from dash import callback_context
import numpy as np
from make_marker_with_json.process_with_json import *
from make_marker_with_json.generate_image_from_json import *
from components import button_default_manual_style, button_active_manual_style
from geometry_msgs.msg import Twist
import math
import rospy

# Initialize publisher without node initialization
try:
    rospy.init_node('mir_joystick_interface', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
except Exception as e:
    print(f"ROS initialization failed: {e}")
    pub = None

stop_requested = False
mission_running = False

PAUSE_BUTTON_1_STYLE = {
    "width": "100%",
    "height": "100px",
    "display": "flex",
    "justifyContent": "center",
    "alignItems": "center",
    "fontSize": "4em",
    "borderRadius": "10px",
    "marginBottom": "10px",
    "marginTop": "10px",
    "backgroundColor": "#003049"
}

def run_mission_in_thread(mission_list, rviz_section):
    global stop_requested, mission_running
    mission_running = True
    
    for mission in mission_list:
        if stop_requested:
            break
        rviz_section.publish_goal(mission["x"], mission["y"], np.arctan2(mission["z"], mission["w"]))
        start_time = time.time()
        while not check_goal_status():
            if time.time() - start_time > 30:
                print("Timeout: Goal không hoàn thành!")
                break
            if stop_requested:
                break
            time.sleep(1)
        
        if not stop_requested:
            print(f"Đã hoàn thành nhiệm vụ: {mission['id']}")
            remove_completed_mission(mission['id'])
    
    mission_running = False

@callback(
    [Output("pause-button", "children", allow_duplicate=True),
     Output("pause-button", "className", allow_duplicate=True),
     Output("pause-button-1", "children", allow_duplicate=True),
     Output("pause-button-1", "style", allow_duplicate=True)],
    [Input("pause-button", "n_clicks"),
     Input("pause-button-1", "n_clicks")],
    [State("pause-button", "children"),
     State("pause-button-1", "children")],
    prevent_initial_call=True
)
def toggle_pause(pause_n_clicks, pause_1_n_clicks, current_icon, current_icon_1):
    global stop_requested, mission_running
    ctx = callback_context
    if not ctx.triggered:
        return no_update, no_update, no_update, no_update
    
    mission_list = load_mission_data()
    if not mission_list and 'fa-pause' in current_icon['props']['className']:
        pause_button_1_style = PAUSE_BUTTON_1_STYLE.copy()
        pause_button_1_style.update({"color": "#08702B"})  
        return (html.I(className="fas fa-play"), "btn btn-success btn-sm me-2",
                html.I(className="fas fa-play"), pause_button_1_style)
    
    if 'fa-play' in current_icon['props']['className']:
        if not mission_list:
            print("Không có nhiệm vụ nào trong marker_mission.json!")
            pause_button_1_style = PAUSE_BUTTON_1_STYLE.copy()
            pause_button_1_style.update({"color": "#08702B"})  
            return (html.I(className="fas fa-play"), "btn btn-success btn-sm me-2",
                    html.I(className="fas fa-play"), pause_button_1_style)
        
        stop_requested = False
        rviz_section = RVizSection()
        mission_thread = threading.Thread(target=run_mission_in_thread, args=(mission_list, rviz_section))
        mission_thread.daemon = True  
        mission_thread.start()
        pause_button_1_style = PAUSE_BUTTON_1_STYLE.copy()
        pause_button_1_style.update({"color": "#FFC107"}) 
        return (html.I(className="fas fa-pause"), "btn btn-warning btn-sm me-2",
                html.I(className="fas fa-pause"), pause_button_1_style)
    else:
        stop_requested = True
        if mission_running and pub is not None:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
            print("Đã xuất bản cmd_vel = (0, 0) để dừng robot.")
        pause_button_1_style = PAUSE_BUTTON_1_STYLE.copy()
        pause_button_1_style.update({"color": "#08702B"}) 
        return (html.I(className="fas fa-play"), "btn btn-success btn-sm me-2",
                html.I(className="fas fa-play"), pause_button_1_style)

@callback(
    Output("joystick-modal", "is_open"),
    Input("close-joystick-btn", "n_clicks"),
    State("joystick-modal", "is_open"),
    prevent_initial_call=True
)
def close_joystick(n_clicks, is_open):
    return not is_open
    
@callback(
    [Output("joystick-container", "style"),
     Output("manual-control", "style"),
     Output("interval-joystick", "disabled")],
    Input("manual-control", "n_clicks"),
    State("joystick-container", "style"),
    prevent_initial_call=True
)
def toggle_joystick(n_clicks, current_style):
    if n_clicks and current_style:
        if current_style.get("display") == "none":
            return {"display": "block"}, button_active_manual_style, False
        else:
            if pub is not None:
                twist = Twist()
                pub.publish(twist)
            return {"display": "none"}, button_default_manual_style, True
    return {"display": "none"}, button_default_manual_style, True

@callback(
    Output("joystick-data", "data"),
    Input("joystick", "angle"),
    Input("joystick", "force"),
    prevent_initial_call=True
)
def update_joystick_data(angle, force):
    MAX_FORCE_THRESHOLD = 0.5
    if force is not None and force > MAX_FORCE_THRESHOLD:
        return {"angle": angle or 0, "force": force or 0}
    return {"angle": 0, "force": 0}  

@callback(
    Output("joystick-output", "children", allow_duplicate=True),
    Input("interval-joystick", "n_intervals"),
    State("joystick-data", "data"),
    State("speed-scale", "value"),
    State("emergency-stop", "on"),
    prevent_initial_call=True
)
def send_twist(n, data, speed_scale, emergency_stop):
    if pub is None:
        return "⚠️ ROS not initialized!"

    if emergency_stop:
        twist = Twist()
        pub.publish(twist)
        return "🛑 Emergency Stop Activated!"

    angle = data["angle"]
    force = data["force"]

    if force == 0:
        twist = Twist()
        pub.publish(twist)
        return "⏹ Robot Stopped"
    angle = angle % 360
    linear, angular = 0.0, 0.0

    linear = math.sin(math.radians(angle)) * force * speed_scale
    angular = math.cos(math.radians(angle)) * force * speed_scale * 2.0

    linear = max(min(linear, 1.0), -1.0)
    angular = max(min(angular, 2.0), -2.0)

    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    pub.publish(twist)

    return f"🚀 Moving: Linear = {linear:.2f} m/s, Angular = {angular:.2f} rad/s"