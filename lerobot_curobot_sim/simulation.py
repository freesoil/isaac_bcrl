import sys
import os
import time
import argparse
from pathlib import Path

# Basic imports that don't require Isaac Sim initialization
try:
    import carb
    import numpy as np
    from isaacsim import SimulationApp
except ImportError as e:
    print(f"Error importing basic Isaac Sim dependencies: {e}")
    print("Make sure you're running this script in the Isaac Sim Python environment")
    sys.exit(1)

GRAPH_PATH = "/ActionGraph"
ROBOT_STAGE_PATH = "/SOARM100"
REALSENSE_VIEWPORT_NAME = "realsense_viewport"

USER_ROOT_PATH = os.getcwd()  # Make sure you're running the script from the correct directory
ROBOT_ASSET_PATH = os.path.join(USER_ROOT_PATH, "isaac_assets", "so100.usd")
ROOM_CAMERA_ASSET_PATH = os.path.join(USER_ROOT_PATH, "isaac_assets", "room_camera.usd")

BACKGROUND_USD_PATH = "/Isaac/Environments/Grid/default_environment.usd"

CAMERA_PRIM_PATH = f"{ROBOT_STAGE_PATH}/Fixed_Jaw/realsense/realsense/realsense_camera"

def parse_arguments():
    """Parse command line arguments to determine simulation mode."""
    parser = argparse.ArgumentParser(description='Isaac Sim SO-ARM 100 Robot Simulation')
    parser.add_argument('--mode', choices=['gui', 'headless', 'webrtc'], default='gui',
                       help='Simulation mode: gui (local GUI), headless (no GUI), webrtc (headless with streaming)')
    parser.add_argument('--show-target', action='store_true',
                       help='Show target cube for manipulation')
    parser.add_argument('--port', type=int, default=8211,
                       help='WebRTC streaming port (default: 8211)')
    return parser.parse_args()

def get_config(mode):
    """Get configuration based on simulation mode."""
    is_headless = mode in ['headless', 'webrtc']
    
    config = {
        "width": 1280,
        "height": 720,
        "window_width": 1920,
        "window_height": 1080,
        "headless": is_headless,
        "hide_ui": is_headless,
        "renderer": "RaytracedLighting",
        "display_options": 3286,
        # Memory management (from docker-compose limits)
        "memory_limit_mb": 8192,  # 8GB limit from docker-compose
        "cache_size_mb": 1024,    # From OMNIVERSE_CACHE_SIZE
        # Physics settings
        "physics_dt": 1.0 / 60.0,
        "rendering_dt": 1.0 / 60.0,
        "physics_substeps": 1,
        "sync_physics": True,
        "sync_rendering": False if is_headless else True,  # Enable sync rendering for GUI mode
        "physics_engine": "physx",
        "physics_solver": "tgs",
        "physics_solver_position_iteration_count": 4,
        "physics_solver_velocity_iteration_count": 1,
        "physics_solver_stabilization_threshold": 0.001,
        "physics_solver_default_buffer_size_fraction": 0.02,
        "physics_solver_contact_offset": 0.02,
        "physics_solver_rest_offset": 0.001,
        "physics_solver_bounce_threshold_velocity": 0.2,
        "physics_solver_friction_threshold_velocity": 0.04,
        "physics_solver_max_velocity": 1000.0,
        "physics_solver_max_angular_velocity": 1000.0,
    }
    
    # Mode-specific settings
    if mode == 'webrtc':
        config.update({
            "enable_livestream": True,
            "livestream_port": args.port,
            "livestream_fps": 30,
            "livestream_resolution_x": 1280,
            "livestream_resolution_y": 720,
            "livestream_bitrate": 8000000,  # 8 Mbps for good quality
            "livestream_encoder_threads": 4,
        })
    elif mode == 'gui':
        config.update({
            "window_title": "Isaac Sim - SO-ARM 100 Robot",
            "anti_aliasing": 2,  # MSAA level
            "enable_vsync": False,  # Disable vsync for better performance
            "enable_debug_rendering": True,
            "enable_hdr": True,
        })
    
    return config

# Parse command line arguments
args = parse_arguments()
mode = args.mode
show_target_cube = args.show_target

# Get configuration for the selected mode
CONFIG = get_config(mode)

print(f"Starting Isaac Sim in {mode} mode...")
simulation_app = SimulationApp(launch_config=CONFIG)

# Now that SimulationApp is initialized, we can import Omniverse modules
try:
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.utils import extensions, nucleus, prims, rotations, stage, viewports
    from omni.isaac.core.utils.prims import is_prim_path_valid, set_targets
    from omni.isaac.core_nodes.scripts.utils import set_target_prims
    from omni.isaac.core.objects import cuboid
    import omni.graph.core as og
    from pxr import Gf, UsdGeom
except ImportError as e:
    print(f"Error importing Omniverse modules: {e}")
    print("Make sure you're running this script in the Isaac Sim Python environment")
    simulation_app.close()
    sys.exit(1)

def setup_viewport(simulation_app, mode):
    """Set up viewport based on mode."""
    try:
        # Set camera view
        viewports.set_camera_view(eye=np.array([1.0, 1.0, 0.6]), target=np.array([0, 0, 0]))
        print("Camera view set successfully")
        
        if mode == 'gui':
            # Enable mouse controls and optimize for GUI mode
            simulation_app.set_setting("/app/window/drawMouse", True)
            simulation_app.set_setting("/app/window/resizable", True)
            simulation_app.set_setting("/app/window/fullscreen", False)
            simulation_app.set_setting("/app/window/x", 100)  # Window position
            simulation_app.set_setting("/app/window/y", 100)
            print("GUI mode settings configured")
            
        elif mode == 'webrtc':
            # Import viewport utility only when needed for WebRTC
            try:
                import omni.kit.viewport.utility as viewport_utility
                viewport_window = viewport_utility.get_active_viewport_window()
                if viewport_window:
                    print("Using existing viewport for WebRTC streaming")
                else:
                    print("Creating viewport for WebRTC streaming...")
                    viewport_window = viewport_utility.create_viewport_window(
                        "WebRTCViewport",
                        1280,  # Width
                        720,   # Height
                        (0, 0),  # Position
                        False   # Do not show window decorations
                    )
                    print("Created viewport window for WebRTC streaming")
                
                # Configure WebRTC viewport settings
                if viewport_window:
                    viewport_window.set_texture_resolution((1280, 720))
                    viewport_window.set_camera_position(np.array([1.0, 1.0, 0.6]), np.array([0, 0, 0]))
                    print("WebRTC viewport configured successfully")
                
                print(f"WebRTC stream will be available at: http://localhost:{args.port}")
                print("Note: WebRTC streaming may take a few minutes to initialize")
            except ImportError as e:
                print(f"Warning: Could not import viewport utility for WebRTC: {e}")
                print("WebRTC streaming may not work correctly")
            
        elif mode == 'headless':
            # Minimal viewport setup for headless mode
            print("Configuring headless viewport")
            simulation_app.set_setting("/app/window/drawMouse", False)
            simulation_app.set_setting("/app/window/resizable", False)
            
    except Exception as e:
        print(f"Warning: Could not set up viewport: {e}")
        print("Attempting to continue with default viewport settings...")
        if mode == 'webrtc':
            print("WebRTC streaming may not work correctly without proper viewport setup")

def setup_extensions(mode):
    """Set up extensions based on mode."""
    # Always enable ROS2 bridge
    extensions.enable_extension("isaacsim.ros2.bridge")
    
    # Enable WebRTC for webrtc mode
    if mode == 'webrtc':
        print("Enabling WebRTC streaming...")
        extensions.enable_extension("omni.kit.livestream.webrtc")
        print("WebRTC streaming extension enabled")
        print(f"Stream should be available at: http://localhost:{args.port}")
        print("Note: WebRTC streaming may take a few minutes to initialize")

def print_controls(mode):
    """Print appropriate controls for the mode."""
    if mode == 'gui':
        print("Starting simulation with GUI controls...")
        print("Controls:")
        print("  - Mouse: Look around")
        print("  - WASD: Move camera")
        print("  - Q/E: Roll camera")
        print("  - R/F: Zoom in/out")
        print("  - Space: Pause/Resume simulation")
    elif mode == 'webrtc':
        print("Starting simulation loop with WebRTC streaming...")
        print(f"WebRTC stream should be available at: http://localhost:{args.port}")
        print("Note: WebRTC streaming may take a few minutes to initialize")
    else:
        print("Starting headless simulation...")

def check_ros2_topics():
    """Check if ROS2 topics are being published."""
    print("ROS2 topics will be published through Isaac Sim ROS2 bridge extension")
    print("To check topics from outside the container, run:")
    print("  docker exec -it isaacsim450-headless-webrtc ros2 topic list")
    print("  docker exec -it isaacsim450-headless-webrtc ros2 topic echo /isaac_joint_states")

# Set up extensions based on mode
setup_extensions(mode)

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Check user assets exist
if not os.path.isfile(ROBOT_ASSET_PATH):
    carb.log_error(f"current working directory: {USER_ROOT_PATH}")
    carb.log_error("Could not find 'so100.usd' user assets")
    simulation_app.close()
    sys.exit()

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

print("Loading environment and robot...")

# Set up viewport based on mode
setup_viewport(simulation_app, mode)

# Loading the simple_room environment
stage.add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, "/background"
)

# Loading the robot USD
prims.create_prim(
    ROBOT_STAGE_PATH,
    "Xform",
    position=np.array([0, 0, 0]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path= ROBOT_ASSET_PATH,
)

# Loading Nvblox Camera (if available)
if os.path.isfile(ROOM_CAMERA_ASSET_PATH):
    prims.create_prim(
        "/nvblox_camera",
        "Xform",
        position=np.array([0, 0, 0]),
        usd_path= ROOM_CAMERA_ASSET_PATH,
    )
    print("Nvblox camera loaded successfully")
else:
    print("Note: room_camera.usd not found, skipping Nvblox camera")

# add some objects, spread evenly along the X axis
# with a fixed offset from the robot in the Y and Z
if show_target_cube:
    target = cuboid.VisualCuboid(
        "/World/target",
        position=np.array([0.3, 0, 0.3]),
        orientation=np.array([0, 1, 0, 0]),
        color=np.array([1.0, 0, 0]),
        size=0.03,
    )

prims.create_prim(
    "/cracker_box",
    "Xform",
    position=np.array([0.2, 0.1, 0.06]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)),
    scale=np.array([0.5,0.5,0.5]),
    usd_path=assets_root_path
    + "/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd",
)
prims.create_prim(
    "/sugar_box",
    "Xform",
    position=np.array([0.39, 0.0232, 0.015]),
    scale=np.array([0.35,0.35,0.35]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90)),
    usd_path=assets_root_path
    + "/Isaac/Props/YCB/Axis_Aligned_Physics/004_sugar_box.usd",
)
prims.create_prim(
    "/tomato_can",
    "Xform",
    position=np.array([0.40985, -0.0752, 0.02]),
    scale=np.array([0.5,0.5,0.5]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90)),
    usd_path=assets_root_path
    + "/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
)

print("Initial simulation update...")
simulation_app.update()

try:
    ros_domain_id = int(os.environ["ROS_DOMAIN_ID"])
    print("Using ROS_DOMAIN_ID: ", ros_domain_id)
except ValueError:
    print("Invalid ROS_DOMAIN_ID integer value. Setting value to 0")
    ros_domain_id = 0
except KeyError:
    print("ROS_DOMAIN_ID environment variable is not set. Setting value to 0")
    ros_domain_id = 0

simulation_app.update()

# Tf Action Graph
TF_GRAPH_PATH = "/World/TfActionGraph"
tf_target_prims = [
    ROBOT_STAGE_PATH,  # Always include the robot
    "/background",      # Include the environment
    CAMERA_PRIM_PATH,  # Include the camera
    "/cracker_box",    # Include objects
    "/sugar_box",
    "/tomato_can",
]
if show_target_cube:
    tf_target_prims.append("/World/target")
    
try:
    # If a graph is not found, create a new one.
    if not is_prim_path_valid(TF_GRAPH_PATH):
        (ros_camera_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": TF_GRAPH_PATH,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ]
            }
        )

    og.Controller.edit(
        TF_GRAPH_PATH,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Publish_Tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Publish_Tf"+".inputs:topicName", "/tf"),
            ],
            og.Controller.Keys.CONNECT: [
                (TF_GRAPH_PATH+"/OnTick.outputs:tick",
                    "Publish_Tf"+".inputs:execIn"),
                (TF_GRAPH_PATH+"/IsaacClock.outputs:simulationTime",
                    "Publish_Tf"+".inputs:timeStamp"),
            ],
        },
    )
    
    # Set the target prims for the TF graph
    if tf_target_prims:
        set_target_prims(
            primPath=TF_GRAPH_PATH + "/Publish_Tf",
            inputName="inputs:targetPrims",
            targetPrimPaths=tf_target_prims,
        )
        print(f"TF action graph setup completed successfully with {len(tf_target_prims)} target prims")
    else:
        print("Warning: No target prims specified for TF graph")
        
except Exception as e:
    print(f"Error setting up TF graph: {e}")

# Main Action Graph for Joint States and Commands
MAIN_GRAPH_PATH = "/World/MainActionGraph"
try:
    # If a graph is not found, create a new one.
    if not is_prim_path_valid(MAIN_GRAPH_PATH):
        (main_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": MAIN_GRAPH_PATH,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("JointStatePublisher", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("JointCommandSubscriber", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("JointStatePublisher.inputs:topicName", "/isaac_joint_states"),
                    ("JointStatePublisher.inputs:targetPrim", ROBOT_STAGE_PATH),
                    ("JointCommandSubscriber.inputs:topicName", "/isaac_joint_commands"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "JointStatePublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "JointStatePublisher.inputs:timeStamp"),
                    ("OnTick.outputs:tick", "JointCommandSubscriber.inputs:execIn"),
                ]
            }
        )
        print("Main action graph created successfully")
    else:
        print("Main action graph already exists")
except Exception as e:
    print(f"Error setting up main action graph: {e}")

# Print mode-specific information
print_controls(mode)

# Start simulation
simulation_app.update()
simulation_context.play()

print("Simulation started. Action graphs should now be executing...")

# Check ROS2 topics after a few frames
check_ros2_topics()

# Main simulation loop
frame_count = 0
status_interval = 100 if mode == 'webrtc' else 300  # More frequent updates for streaming
last_status_time = time.time()

print(f"\nStarting {mode} simulation loop...")
print("Press Ctrl+C to exit\n")

while simulation_app.is_running():
    try:
        # Step the simulation with rendering
        simulation_context.step(render=True)
        
        # Execute action graphs - tick the OnTick nodes to trigger execution
        try:
            # Trigger TF graph execution
            og.Controller.set(
                og.Controller.attribute("/World/TfActionGraph/OnTick.outputs:tick"), True
            )
            
            # Trigger main graph execution
            og.Controller.set(
                og.Controller.attribute("/World/MainActionGraph/OnTick.outputs:tick"), True
            )
        except Exception as e:
            print(f"Warning: Could not execute action graphs: {e}")
        
        frame_count += 1
        current_time = time.time()
        
        # Print status at different intervals based on mode
        if frame_count % status_interval == 0:
            elapsed_time = current_time - last_status_time
            fps = status_interval / elapsed_time if elapsed_time > 0 else 0
            
            print(f"\n=== Simulation Status (Frame {frame_count}) ===")
            print(f"Mode: {mode}")
            print(f"FPS: {fps:.1f}")
            print(f"Action graphs executing: TF and Main")
            
            if mode == 'webrtc':
                print(f"WebRTC stream available at: http://localhost:{args.port}")
            elif mode == 'gui':
                print("GUI controls active")
            
            print("=" * 40 + "\n")
            last_status_time = current_time
        
        # Add a small delay to prevent excessive CPU usage
        # Adjust delay based on mode
        if mode == 'gui':
            time.sleep(0.001)  # Minimal delay for GUI
        else:
            time.sleep(0.01)  # Larger delay for headless/webrtc
            
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
        break
    except Exception as e:
        print(f"\nError in simulation loop: {e}")
        print("Attempting to continue...")

print("\nSimulation finished.")
print(f"Total frames: {frame_count}")
simulation_context.stop()
simulation_app.close()
