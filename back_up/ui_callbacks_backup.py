import dash, rospy, tf, requests
from dash import dcc, html, Input, Output, State
from dash.exceptions import PreventUpdate
import dash_bootstrap_components as dbc
from components import LoginPage, ChangePasswordPage, Sidebar, StatusBar, MapSection
from utils.data import authenticate, user_credentials, update_password
from components.draw_mode import create_draw_mode_layout
from page_draw_mode.function_draw_mode import *
from components.rviz_section import create_rviz_section
from make_marker_with_json import *
from page_map.map_api import MapAPI
from page_mission.missions.layout import mission_queue_layout
from page_path_guides import PathGuideLayout
from page_options.callbacks import *

login_page = LoginPage()
change_password_page = ChangePasswordPage()
sidebar = Sidebar()
status_bar = StatusBar()
map_section = MapSection()
map_api = MapAPI()
path_guides = PathGuideLayout()

ip = '192.168.0.172'
host = 'http://' + ip + '/api/v2.0.0/'
headers = {
    'Content-Type': 'application/json',
    'Authorization': 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
}

@callback(
    Output("app-container", "children"),
    Input("login-button", "n_clicks"),
    State("username", "value"),
    State("password", "value"),
    prevent_initial_call=True
)
def login(n_clicks, username, password):
    if authenticate(username, password):
        return html.Div(
            [
                dcc.Location(id='url', refresh=False),
                status_bar.create_status_bar(),
                sidebar.create_sidebar(),
                html.Div(id="page-content", style={"marginLeft": "250px"}),
            ],
            style={"background": "#BDC3C7", "height": "100vh", "overflow": "hidden"},
        )
    else:
        return html.Div([login_page.layout, html.Div("Login Failed", style={"color": "red"})])
    
@callback(
    [
        Output("position-modal", "is_open"),
        Output("name-position", "value"),  
        Output("x-input", "value"),
        Output("y-input", "value"),
        Output("z-input", "value"),
        Output("w-input", "value"),
        Output("content-area", "children"),
    ],
    [
        Input("add-positions-btn", "n_clicks"),
        Input("use-robot-btn", "n_clicks"),
        Input("add-position-btn", "n_clicks"),
        Input("cancel-btn", "n_clicks"),
    ],
    [
        State("name-position", "value"), 
        State("x-input", "value"),
        State("y-input", "value"),
        State("z-input", "value"),
        State("w-input", "value"),
        State("position-modal", "is_open"),
    ],
    prevent_initial_call=True
)
def manage_position_modal(add_pos_clicks, use_robot_clicks, add_clicks, cancel_clicks,
                         name_position, x_val, y_val, z_val, w_val, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return no_update, no_update, no_update, no_update, no_update, no_update, no_update
    
    button_id = ctx.triggered[0]["prop_id"].split(".")[0]
    
    if button_id == "add-positions-btn":
        return True, "", None, None, None, None, no_update 
    
    elif button_id == "use-robot-btn":
        try:
            listener = tf.TransformListener()
            listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
            robot_x, robot_y, _ = trans
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            robot_z, robot_w = math.sin(yaw), math.cos(yaw)
            return True, no_update, robot_x, robot_y, robot_z, robot_w, no_update 
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return True, no_update, no_update, no_update, no_update, no_update, "Không thể lấy vị trí robot"
    
    elif button_id == "add-position-btn":
        if x_val is not None and y_val is not None and name_position is not None:
            save_position_to_json(name_position, x_val, y_val, z_val, w_val)
            generate_marker_image()
            return False, "", None, None, None, None, f"Đã thêm điểm {name_position} ({x_val}, {y_val})"  
        return True, no_update, x_val, y_val, z_val, w_val, "Vui lòng nhập X hoặc Y hoặc Tên vị trí"
    
    elif button_id == "cancel-btn":
        return False, "", None, None, None, None, no_update  
    
    return no_update, no_update, no_update, no_update, no_update, no_update, no_update

@callback(
    Output("docker-modal", "is_open", allow_duplicate=True),
    Input("add-dockers-btn", "n_clicks"),
    State("docker-modal", "is_open"),
    prevent_initial_call=True
)
def toggle_docker_modal(n_clicks, is_open):
    return not is_open 

@callback(
    [
        Output("docker-modal", "is_open"),
        Output("name-docker", "value"),
        Output("docker-x", "value"),
        Output("docker-y", "value"),
        Output("docker-z", "value"),
        Output("docker-w", "value"),
        Output("content-area", "children", allow_duplicate=True),
    ],
    [
        Input("add-dockers-btn", "n_clicks"),
        Input("use-robot-docker-btn", "n_clicks"),
        Input("add-docker-btn", "n_clicks"),
        Input("cancel-btn", "n_clicks"),
    ],
    [
        State("name-docker", "value"),
        State("docker-x", "value"),
        State("docker-y", "value"),
        State("docker-z", "value"),
        State("docker-w", "value"),
        State("docker-modal", "is_open"),
    ],
    prevent_initial_call=True
)
def manage_docker_modal(add_pos_clicks, use_robot_clicks, add_clicks, cancel_clicks,
                         name_docker, x_val, y_val, z_val, w_val, is_open):
    ctx = dash.callback_context
    if not ctx.triggered:
        return no_update, no_update, no_update, no_update, no_update, no_update, no_update
    button_id = ctx.triggered[0]["prop_id"].split(".")[0]
    if button_id == "add-dockers-btn":
        return True, "", None, None, None, None, no_update
    elif button_id == "use-robot-docker-btn":
        try:
            listener = tf.TransformListener()
            listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
            robot_x, robot_y, _ = trans
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            robot_z, robot_w = math.sin(yaw), math.cos(yaw)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return True, "", None, None, None, None, "Không thể lấy vị trí robot"
        return True, robot_x, robot_y, robot_z, robot_w, no_update
    elif button_id == "add-docker-btn":
        if x_val is not None and y_val is not None and name_docker is not None:
            save_docker_to_json(name_docker, x_val, y_val, z_val, w_val) 
            generate_docker_image() 
            return False, "", None, None, None, None, f"Đã thêm điểm Docker"
        return True, "", x_val, y_val, z_val, w_val, "Vui lòng nhập X hoặc Y hoặc Tên trạm sạc"
    elif button_id == "cancel-btn":
        return False, "", None, None, None, None, no_update
    return no_update, no_update, no_update, no_update, no_update, no_update, no_update

@callback(
    [Output("pause-button", "children"), Output("pause-button", "className")],
    Input("pause-button", "n_clicks"),
    State("pause-button", "children"),
    State("pause-button", "className"),
    prevent_initial_call=True
)
def toggle_pause(n_clicks, current_icon, current_class):
    if 'fa-play' in current_icon['props']['className']:  
        return html.I(className="fas fa-pause"), "btn btn-warning btn-sm me-2" 
    return html.I(className="fas fa-play"), "btn btn-success btn-sm me-2" 

@callback(
    Output('page-content', 'children'),
    Input('url', 'pathname')
)
def display_page(pathname):
    if pathname == '/draw-mode':
        return html.Div(
            [
                status_bar.create_status_bar(),
                html.Iframe(
                    src="/assets/drawing_tool.html",  # Adjust the path based on your setup
                    style={"width": "100%", "height": "1000px", "border": "none"}  # Adjust dimensions as needed
                ),
                dcc.Interval(
                    id='interval-component',
                    interval=1*1000,
                    n_intervals=0
                )
            ]
        )
    elif pathname == '/maps':
        return html.Div(
                [
                    status_bar.create_status_bar(),
                    map_section.create_map_section(),
                    dcc.Interval(
                        id='interval-component',
                        interval=1*1000,
                        n_intervals=0
                    )
                ]
            )
    elif pathname == '/change-password':
        return html.Div(
                [
                    status_bar.create_status_bar(),
                    change_password_page.layout,
                    dcc.Interval(
                        id='interval-component',
                        interval=1*1000,
                        n_intervals=0
                    )
                ]
            )
    elif pathname == '/rviz':
        return html.Div(
                [
                    status_bar.create_status_bar(),
                    create_rviz_section(),
                    dcc.Interval(
                        id='interval-component',
                        interval=1*1000,
                        n_intervals=0
                    )
                ]
            )
    elif pathname == '/missions':
        return mission_queue_layout()
    elif pathname == '/map-api':
        return map_api.create_map_api()
    elif pathname == '/path-guides':
            return html.Div(
            [
                status_bar.create_status_bar(),
                path_guides.create_layout(),
                dcc.Interval(
                    id='interval-component',
                    interval=1*1000,
                    n_intervals=0
                )
            ]
        )
    else:
        return html.Div(
            [
                status_bar.create_status_bar(),
                map_section.create_map_section(),
                dcc.Interval(
                    id='interval-component',
                    interval=1*1000,
                    n_intervals=0
                )
            ]
        )
    
@callback(
    Output("password-status", "children"),
    Input("update-password-button", "n_clicks"),
    State("new-password", "value"),
    State("confirm-password", "value"),
    prevent_initial_call=True
)
def update_password_callback(n_clicks, new_password, confirm_password):
    if new_password == confirm_password:
        global user_credentials
        username = list(user_credentials.keys())[0]
        if update_password(username, new_password):
            return html.Div("Password updated successfully!", style={"color": "green"})
        else:
            return html.Div("Failed to update password.", style={"color": "red"})
    else:
        return html.Div("Passwords do not match.", style={"color": "red"})
    
@callback(
    Output('map_section', 'children'),
    Input('language-dropdown', 'value')
)
def change_language(language):
    translations = {
        'en': {'title': 'Main Floor', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
        'vi': {'title': 'Tầng Chính', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
        'es': {'title': 'Piso Principal', 'map': 'Edit and draw the map', 'ready': 'El mapa está listo para tu trabajo.'},
    }
    translation = translations.get(language, translations['en'])
    return html.Div(
        [
            html.H3(translation['title'], className="mb-3", style={"color": "#2C3E50"}),
            html.P(translation['map'], className="text-muted"),
            html.Img(src="/path/to/save/map_image.png", style={"width": "100%", "border": "2px solid #34495E"}),
            html.P(translation['ready'], className="text-info mt-2"),
            html.Div(id="content-area"),
        ],
        style={
            "padding": "20px",
            "flex": "1",
            "background": "#ECF0F1",
            "marginLeft": "250px",
            "marginTop": "50px",
        },
    )

layout = html.Div(
    [
        dcc.Location(id='url', refresh=False),
        html.Div(id="app-container", children=[login_page.layout]),
        html.Div(id="joystick-output", style={"margin": "20px", "fontSize": "0px"}),
        dcc.Interval(
            id='interval-component',
            interval=1000,
            n_intervals=0
        )
    ]
)

@callback(
    Output("sidebar-nav", "children"),
    Input('url', 'pathname'),
    prevent_initial_call=True
)
def update_active_link(pathname):
    nav_links = [
        {"href": "/", "id": "index-link", "label": "Home"},
        {"href": "/draw-mode", "id": "draw-mode-link", "label": "Draw Mode"},
        {"href": "/rviz", "id": "rviz-link", "label": "RViz"},
        {"href": "/missions", "id": "mission-link", "label": "Missions"},
        {"href": "/map-api", "id": "map-api-link", "label": "Maps"},
        {"href": "/path-guides", "id": "path-guides-link", "label": "Path Guides"},
        {"href": "#", "id": "io-modules-link", "label": "I/O Modules"},
        {"href": "#", "id": "users-link", "label": "Users"},
        {"href": "#", "id": "user-groups-link", "label": "User Groups"},
        {"href": "#", "id": "paths-link", "label": "Paths"},
        {"href": "#", "id": "marker-types-link", "label": "Marker Types"},
        {"href": "#", "id": "footprints-link", "label": "Footprints"},
        {"href": "/change-password", "id": "change-password-link", "label": "Change Password"},
    ]

    updated_links = []
    for link in nav_links:
        active = pathname == link["href"]
        updated_links.append(
            dbc.NavLink(
                link["label"],
                href=link["href"],
                id=link["id"],
                className="text-white",
                active=active
            )
        )
    return updated_links

@callback(
    Output("battery-percen", "children"),
    Input("interval-component", "n_intervals")
)
def update_battery_status(n):

    try:
        response = requests.get(host + '/status', headers=headers)  
        data = response.json()
        battery_level = data.get("battery_percentage", "--") 
        if isinstance(battery_level, float):
            battery_level = round(battery_level)
        return f"{battery_level}%"
    except Exception as e:
        print(f"⚠️ Lỗi khi gọi API: {e}") 
        return "--%"
    
@callback(
    [Output('status-text', 'children'),
    #  Output('status-status-bar', 'children'),
     Output('status-icon', 'className'),
     Output('status-badge', 'children'),
     Output('status-badge', 'color')],
    [Input('interval-component', 'n_intervals')]
)
def update_charging_status(n):
    json_path = os.path.join("database_json", "mir_status.json")
    
    try:
        with open(json_path, "r") as f:
            content = json.load(f)
            latest_status = content.get("data", {})
    except Exception as e:
        # print(f"⚠️ Lỗi đọc file JSON: {e}")
        latest_status = {"message": "Không thể đọc trạng thái", "args": {}}
    message_template = latest_status.get("message", "No status available")
    args = latest_status.get("args", {})
    try:
        message = message_template % args
    except Exception as e:
        # print(f"⚠️ Lỗi khi format message: {e}")
        message = message_template
    icon_class = "fas fa-spinner fa-spin me-2"
    badge_text = "EXECUTING"
    badge_color = "success"
    if "error" in message.lower():
        icon_class = "fas fa-exclamation-triangle me-2"
        badge_color = "danger"
        badge_text = "ERROR"
    elif "complete" in message.lower():
        icon_class = "fas fa-check-circle me-2"
        badge_text = "COMPLETED"
    elif "waiting" in message.lower():
        icon_class = "fas fa-clock me-2"
        badge_text = "WAITING"

    return (
        # message,
        message,
        icon_class,
        badge_text,
        badge_color
    )

# Thêm các import cần thiết vào đầu file ui_callbacks.py
import subprocess
import psutil
import signal
import os
import time
import json

def update_simulation_state():
    """
    Cập nhật file simulattion_state.json khi logout
    """
    try:
        simulation_data = {
            "mode": "null",
            "simulation_running": False
        }
        
        # Đường dẫn file JSON
        json_path = "simulation_state.json"
        
        # Kiểm tra và tạo thư mục nếu cần
        os.makedirs(os.path.dirname(json_path) if os.path.dirname(json_path) else '.', exist_ok=True)
        
        # Ghi file JSON
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(simulation_data, f, indent=2, ensure_ascii=False)
        
        print(f"✅ Đã cập nhật simulation state: {simulation_data}")
        
    except Exception as e:
        print(f"⚠️ Lỗi khi cập nhật simulation state: {e}")
        # Thử tạo file trong thư mục backup
        try:
            backup_path = os.path.join("database_json", "simulate_state.json")
            os.makedirs("database_json", exist_ok=True)
            
            with open(backup_path, 'w', encoding='utf-8') as f:
                json.dump(simulation_data, f, indent=2, ensure_ascii=False)
            
            print(f"✅ Đã cập nhật simulation state tại backup path: {backup_path}")
        except Exception as backup_error:
            print(f"⚠️ Lỗi cả backup path: {backup_error}")

# Thay thế callback admin popover cũ bằng callback mới này
@callback(
    [Output("url", "pathname", allow_duplicate=True),
     Output("url", "refresh", allow_duplicate=True),
     Output("admin-popover", "is_open", allow_duplicate=True)],
    Input("logout-button", "n_clicks"),
    prevent_initial_call=True
)
def handle_logout(n_clicks):
    """
    Xử lý logout: tắt các process ROS, cập nhật simulation state và chuyển về trang options
    """
    if n_clicks:
        try:
            print("🔄 Đang thực hiện logout...")
            
            # Tắt các ROS processes 
            terminate_ros_processes()
            
            # Cập nhật simulation state
            update_simulation_state()
            
            print("✅ Logout thành công!")
            
            # Chuyển về trang options
            return "/", False, True
            
        except Exception as e:
            print(f"⚠️ Lỗi khi logout: {e}")
            # Vẫn logout dù có lỗi và chuyển về options
            return "/", False, True
    
    raise PreventUpdate

def terminate_ros_processes():
    """
    Tắt tất cả các ROS processes (roslaunch, rosrun, rviz, roscore, etc.)
    """
    ros_process_names = [
        'roslaunch',
        'rosrun', 
        'rviz',
        'roscore',
        'rosmaster',
        'rosout',
        'rosnode',
        'rostopic',
        'rosservice',
        'rosparam',
        'rosbag',
        'rostopic'
    ]
    
    terminated_processes = []
    
    print("🔄 Đang tắt các ROS processes...")
    
    try:
        # Method 1: Sử dụng psutil để tìm và terminate processes
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                process_name = proc.info['name']
                cmdline = proc.info['cmdline'] if proc.info['cmdline'] else []
                
                # Kiểm tra tên process
                if process_name in ros_process_names:
                    print(f"🎯 Tìm thấy process: {process_name} (PID: {proc.info['pid']})")
                    proc.terminate()
                    terminated_processes.append(f"{process_name} (PID: {proc.info['pid']})")
                    continue
                    
                # Kiểm tra command line có chứa ROS commands
                cmdline_str = ' '.join(cmdline).lower()
                if any(ros_name in cmdline_str for ros_name in ros_process_names):
                    print(f"🎯 Tìm thấy ROS process: {process_name} (PID: {proc.info['pid']})")
                    proc.terminate()
                    terminated_processes.append(f"{process_name} (PID: {proc.info['pid']})")
                        
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
                
        # Chờ processes terminate gracefully
        print("⏳ Chờ processes tắt...")
        time.sleep(3)
        
        # Force kill processes còn sót lại
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if proc.info['name'] in ros_process_names:
                    print(f"💀 Force killing: {proc.info['name']} (PID: {proc.info['pid']})")
                    proc.kill()
                    terminated_processes.append(f"KILLED: {proc.info['name']} (PID: {proc.info['pid']})")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
                
    except Exception as e:
        print(f"⚠️ Lỗi khi sử dụng psutil: {e}")
    
    # Method 2: Backup method sử dụng system commands
    try:
        print("🔄 Sử dụng backup method...")
        
        # Sử dụng pkill để tắt processes
        ros_commands = ['roscore', 'roslaunch', 'rosrun', 'rviz', 'rosmaster']
        for cmd in ros_commands:
            try:
                result = subprocess.run(['pkill', '-f', cmd], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    print(f"✅ Đã tắt {cmd}")
            except subprocess.TimeoutExpired:
                print(f"⏰ Timeout khi tắt {cmd}")
            except Exception as e:
                print(f"⚠️ Lỗi khi tắt {cmd}: {e}")
        
    except Exception as e:
        print(f"⚠️ Lỗi backup method: {e}")
    
    # Method 3: Force kill với killall
    try:
        print("🔄 Force killing remaining processes...")
        
        force_kill_commands = ['roscore', 'roslaunch', 'rosrun', 'rviz', 'rosmaster']
        for cmd in force_kill_commands:
            try:
                subprocess.run(['killall', '-9', cmd], 
                              capture_output=True, timeout=3)
            except:
                pass  # Ignore errors
                
    except Exception as e:
        print(f"⚠️ Lỗi force kill: {e}")
    
    # Cleanup ROS temporary files
    try:
        print("🧹 Cleaning up ROS temporary files...")
        
        # Xóa ROS log files
        subprocess.run(['find', '/tmp', '-name', 'ros*', '-type', 'd', '-exec', 'rm', '-rf', '{}', '+'], 
                      capture_output=True, timeout=10)
        
        # Xóa ROS runtime files
        subprocess.run(['rm', '-rf', '/tmp/.ros*'], shell=True, capture_output=True, timeout=5)
        
        print("✅ Cleanup hoàn tất")
        
    except Exception as e:
        print(f"⚠️ Lỗi cleanup: {e}")
    
    if terminated_processes:
        print(f"✅ Đã tắt các processes: {terminated_processes}")
    else:
        print("ℹ️ Không tìm thấy ROS processes nào đang chạy")

# Cập nhật callback toggle admin popover (loại bỏ xử lý logout)
@callback(
    Output("admin-popover", "is_open"),
    Input("admin-popover-target", "n_clicks"),
    State("admin-popover", "is_open"),
    prevent_initial_call=True
)
def toggle_admin_popover(admin_btn_clicks, is_open):
    """
    Chỉ toggle popover, không xử lý logout
    """
    if admin_btn_clicks:
        return not is_open
    raise PreventUpdate