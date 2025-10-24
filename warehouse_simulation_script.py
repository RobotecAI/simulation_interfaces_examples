import time
import rclpy
import argparse
import os
import logging
import numpy as np

from rclpy.node import Node

from simulation_interfaces.msg import Result, SimulatorFeatures, SimulationState
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist
from ur10_helpers import move_ur10_joints

# ************************************
# Isaac Sim and Gazebo specific addons
# ************************************
DEMO_ASSET_PATH = os.getenv('DEMO_ASSET_PATH')


def configure_simulation_backend(sim_backend: str):
    """Configure asset URIs, service names, and entity naming based on simulation backend.
    
    Sets global variables directly based on the chosen backend.
    
    Args:
        sim_backend: Simulation backend ("isaacsim", "o3de", or "gazebo")
    """
    global TABLE_URI, BLUE_CUBE_URI, RED_CUBE_URI, DINGO_URI, UR10_URI, CARDBOARD_URI, WORLD_URI
    global GET_FEATURES_SERVICE, LOAD_WORLD_SERVICE, SPAWN_ENTITY_SERVICE, DELETE_ENTITY_SERVICE, GET_ENTITY_STATE_SERVICE
    global SET_ENTITY_STATE_SERVICE, SET_SIMULATION_STATE_SERVICE, STEP_SIMULATION_SERVICE, RESET_SIMULATION_SERVICE, UNLOAD_WORLD_SERVICE
    global RENAME_ENTITY
    
    # Configure asset URIs based on backend
    if sim_backend == "isaacsim":
        if not DEMO_ASSET_PATH:
            raise RuntimeError("DEMO_ASSET_PATH must be set to use IsaacSim asset backend")
        WORLD_URI = os.path.join(DEMO_ASSET_PATH, "Collected_warehouse_with_forklifts/warehouse_with_forklifts.usd")
        TABLE_URI = os.path.join(DEMO_ASSET_PATH, "thor_table/thor_table.usd")
        BLUE_CUBE_URI = os.path.join(DEMO_ASSET_PATH, "Collected_blue_block/blue_block.usd")
        RED_CUBE_URI = os.path.join(DEMO_ASSET_PATH, "Collected_red_block/red_block.usd")
        DINGO_URI = os.path.join(DEMO_ASSET_PATH, "dingo/dingo_ROS.usd")
        UR10_URI = os.path.join(DEMO_ASSET_PATH, "Collected_ur10e_robotiq2f-140_ROS/ur10e_robotiq2f-140_ROS.usd")
        CARDBOARD_URI = os.path.join(DEMO_ASSET_PATH, "Collected_warehouse_with_forklifts/Props/SM_CardBoxA_02.usd")
    elif sim_backend == "o3de":
        WORLD_URI = "levels/warehouse/warehouse.spawnable"
        TABLE_URI = "product_asset:///assets/props/thortable/thortable.spawnable"
        BLUE_CUBE_URI = "product_asset:///assets/props/collectedblocks/basicblock_blue.spawnable"
        RED_CUBE_URI = "product_asset:///assets/props/collectedblocks/basicblock_red.spawnable"
        DINGO_URI = "product_asset:///assets/dingo/dingo-d.spawnable"
        UR10_URI = "product_asset:///prefabs/ur10-with-fingers.spawnable"
        CARDBOARD_URI = "product_asset:///assets/props/sm_cardboxa_02.spawnable"
    elif sim_backend == "gazebo":
        if not DEMO_ASSET_PATH:
            raise RuntimeError("DEMO_ASSET_PATH must be set to use Gazebo asset backend")
        TABLE_URI = os.path.join(DEMO_ASSET_PATH, "thor_table")
        BLUE_CUBE_URI = os.path.join(DEMO_ASSET_PATH, "blue_block")
        RED_CUBE_URI = os.path.join(DEMO_ASSET_PATH, "red_block")
        DINGO_URI = os.path.join(DEMO_ASSET_PATH, "dingo_d")
        UR10_URI = os.path.join(DEMO_ASSET_PATH, "ur10")
        CARDBOARD_URI = os.path.join(DEMO_ASSET_PATH, "cardboard_box")
    else:
        raise RuntimeError(f"Unknown simulation backend: {sim_backend}")
    
    # Configure service names based on backend
    service_prefix_str = '/gz_server' if sim_backend == "gazebo" else ''
    GET_FEATURES_SERVICE = service_prefix_str + "/get_simulator_features"
    LOAD_WORLD_SERVICE = service_prefix_str + "/load_world"
    SPAWN_ENTITY_SERVICE = service_prefix_str + "/spawn_entity"
    DELETE_ENTITY_SERVICE = service_prefix_str + "/delete_entity"
    GET_ENTITY_STATE_SERVICE = service_prefix_str + "/get_entity_state"
    SET_ENTITY_STATE_SERVICE = service_prefix_str + "/set_entity_state"
    SET_SIMULATION_STATE_SERVICE = service_prefix_str + "/set_simulation_state"
    STEP_SIMULATION_SERVICE = service_prefix_str + "/step_simulation"
    RESET_SIMULATION_SERVICE = service_prefix_str + "/reset_simulation"
    UNLOAD_WORLD_SERVICE = service_prefix_str + "/unload_world"
    
    # Configure entity naming
    RENAME_ENTITY = (sim_backend == "isaacsim")


def format_entity_name(entity_name: str, rename: bool) -> str:
    """Format entity name according to simulation backend requirements.
    
    For Isaacsim Sim backend, prefixes entity names with forward slash.
    For other backends, returns the name unchanged.
    """
    if rename:
        return f"/{entity_name}" if not entity_name.startswith("/") else entity_name
    return entity_name


# ************************************
# Simulation interfaces helper methods
# ************************************


def get_features(node: Node, service_name: str) -> SimulatorFeatures:
    from simulation_interfaces.srv import GetSimulatorFeatures
    get_features_client = node.create_client(GetSimulatorFeatures, service_name)
    req = GetSimulatorFeatures.Request()
    future = get_features_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        return future.result().features
    else:
        print(f"Failed to call get_simulator_features service: {future.result().result.error_message}")
        return None


def load_world(node: Node, service_name: str, uri: str) -> bool:
    from simulation_interfaces.srv import LoadWorld
    load_world_client = node.create_client(LoadWorld, service_name)
    req = LoadWorld.Request()
    req.uri = uri
    future = load_world_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and future.result().result.result == Result.RESULT_OK:
        return True
    else:
        print(f"Failed to load world {req.uri}: {future.result().result.error_message}")    
        return False


def spawn_entity(node: Node, service_name: str, uri: str, name: str, initial_pose: PoseStamped) -> bool:
    from simulation_interfaces.srv import SpawnEntity
    spawn_entity_client = node.create_client(SpawnEntity, service_name)
    req = SpawnEntity.Request()
    req.name = name
    req.uri = uri
    req.allow_renaming = True
    req.initial_pose = initial_pose
    future = spawn_entity_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and future.result().result.result == Result.RESULT_OK:
        return True
    else:
        print(f"Failed to spawn {name}: {future.result().result.error_message}")
        return False


def delete_entity(node: Node, service_name: str, name: str) -> bool:
    from simulation_interfaces.srv import DeleteEntity
    delete_entity_client = node.create_client(DeleteEntity, service_name)
    req = DeleteEntity.Request()
    req.entity = name
    future = delete_entity_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and future.result().result.result == Result.RESULT_OK:
        return True
    else:
        print(f"Failed to delete entity '{name}': {future.result().result.error_message}")
        return False


def get_entity_state(node: Node, service_name: str, name: str) -> PoseStamped:
    from simulation_interfaces.srv import GetEntityState
    get_entity_state_client = node.create_client(GetEntityState, service_name)
    req = GetEntityState.Request()
    req.entity = name
    future = get_entity_state_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and future.result().result.result == Result.RESULT_OK:
        return future.result().state
    else:
        print(f"Failed to get entity state for '{name}': {future.result().result.error_message}")
        return None


def set_entity_state(node: Node, service_name: str, name: str, pose: PoseStamped) -> bool:
    from simulation_interfaces.msg import EntityState
    from simulation_interfaces.srv import SetEntityState
    set_entity_state_client = node.create_client(SetEntityState, service_name)
    req = SetEntityState.Request()
    req.entity = name
    state = EntityState()
    state.pose = pose.pose  # Extract Pose from PoseStamped
    req.state = state
    future = set_entity_state_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and future.result().result.result == Result.RESULT_OK:
        return True
    else:
        print(f"Failed to move {name} to new position: {future.result().result.error_message}")
        return False


def set_simulation_state(node: Node, service_name: str, state: int) -> bool:
    from simulation_interfaces.srv import SetSimulationState
    set_state_client = node.create_client(SetSimulationState, service_name)
    req = SetSimulationState.Request()
    req.state.state = state
    future = set_state_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and future.result().result.result == Result.RESULT_OK:
        return True
    else:
        print(f"Failed to set simulation state: {future.result().result.error_message}")
        return False


def step_simulation(node: Node, service_name: str, steps: int) -> bool:
    from simulation_interfaces.srv import StepSimulation
    step_simulation_client = node.create_client(StepSimulation, service_name)
    req = StepSimulation.Request()
    req.steps = steps
    future = step_simulation_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and hasattr(future.result(), 'result') and future.result().result.result == Result.RESULT_OK:
        return True
    else:
        print(f"Failed to step simulation: {future.result().result.error_message}")
        return False


def reset_simulation(node: Node, service_name: str) -> bool:
    from simulation_interfaces.srv import ResetSimulation
    reset_client = node.create_client(ResetSimulation, service_name)
    req = ResetSimulation.Request()
    req.scope = ResetSimulation.Request.SCOPE_STATE
    future = reset_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and hasattr(future.result(), 'result') and future.result().result.result == Result.RESULT_OK:
        return True
    else:
        print(f"Failed to reset simulation: {future.result().result.error_message}")
        return False


def unload_world(node: Node, service_name: str) -> bool:
    from simulation_interfaces.srv import UnloadWorld
    unload_world_client = node.create_client(UnloadWorld, service_name)
    req = UnloadWorld.Request()
    future = unload_world_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() and future.result().result.result == Result.RESULT_OK:
        return True
    else:
        print(f"Failed to unload world: {future.result().result.error_message}")
        return False

# ************************************
# Demo methods
# ************************************


def spawn_scene(node: Node, service_name: str) -> bool:
    initial_pose = PoseStamped()
    initial_pose.pose.position = Point(x=-1.0, y=-1.5, z=1.19)
    initial_pose.pose.orientation = yaw_to_quaternion(1.5708)
    if not spawn_entity(node, service_name, TABLE_URI, format_entity_name("Table", RENAME_ENTITY), initial_pose):
        return False
    initial_pose.pose.position = Point(x=-4.0, y=-3.0, z=0.0)
    initial_pose.pose.orientation = yaw_to_quaternion(0.0)
    if not spawn_entity(node, service_name, DINGO_URI, format_entity_name("Dingo", RENAME_ENTITY), initial_pose):
        return False
    initial_pose.pose.position = Point(x=-1.0, y=-2.14, z=1.19)
    initial_pose.pose.orientation = yaw_to_quaternion(1.5708)
    if not spawn_entity(node, service_name, UR10_URI, format_entity_name("UR10", RENAME_ENTITY), initial_pose):
        return False

    table = Point(x=-1.0, y=-1.5, z=1.19)
    cube_configs = [
        ((table.x + 0.2, table.y + 0.2, table.z + 0.15), "blue"),
        ((table.x - 0.2, table.y + 0.2, table.z + 0.15), "red"),
        ((table.x + 0.2, table.y - 0.2, table.z + 0.15), "red"),
        ((table.x - 0.2, table.y - 0.2, table.z + 0.15), "blue"),
        ((table.x, table.y, table.z + 0.15), "blue"),
        ((table.x + 0.1, table.y, table.z + 0.15), "red"),
        ((table.x - 0.1, table.y, table.z + 0.15), "red"),
    ]

    for i, (pos, color) in enumerate(cube_configs):
        pose = PoseStamped()
        pose.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        pose.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        if color == "blue":
            uri = BLUE_CUBE_URI
        else:
            uri = RED_CUBE_URI
        success = spawn_entity(
            node,
            service_name,
            uri,
            format_entity_name(f"{color}_cube_{i}", RENAME_ENTITY),
            pose
        )
        if not success:
            print(f"Failed to spawn {color} cube {i}")
            return False
    return True


def spawn_boxes(node: Node, service_name: str, step: int) -> bool:
    box_positions_by_iteration = [
        [
            (-2.5, -2.5, 0.0, 0.0),
            (-0.5, -3.5, 0.0, 1.5708),
            (-1.8, -3.3, 0.0, 0.7854),
        ],
        [
            (-3.0, -1.0, 0.0, 1.5708),
            (0.5, -2.0, 0.0, 0.0),
            (0.8, -0.3, 0.0, 0.7854),
        ],
        [
            (-1.8,  0.3, 0.0, 0.7854),
            (-0.7,  0.0, 0.0, 1.5708),
            (-2.2, -0.7, 0.0, 0.0),
        ],
    ]
    
    box_positions = box_positions_by_iteration[step]
    for i, (box_x, box_y, box_z, box_yaw) in enumerate(box_positions):
        pose = PoseStamped()
        pose.pose.position = Point(x=float(box_x), y=float(box_y), z=float(box_z))
        pose.pose.orientation = yaw_to_quaternion(box_yaw)
        if not spawn_entity(
            node,
            service_name,
            CARDBOARD_URI,
            format_entity_name(f"obstacle_box_{step}_{i}", RENAME_ENTITY),
            pose
        ):
            print(f"Failed to spawn obstacle box {i + 1}")
            return False

    return True


# Example loop: push 3 boxes to target poses with Dingo
def loop_simulation(node: Node, sim_backend: str):
    logger = logging.getLogger(__name__)
    dingo_positions = [
        (-4.0, -2.5, 0.0, 0.0),
        (0.5, -3.5, 0.0, 1.5708),
        (-4.0, 0.3, 0.0, 0.0)
    ]
    target_positions = [
        Point(x=-2.0, y=-2.5, z=0.0),
        Point(x=0.5, y=-1.0, z=0.0),
        Point(x=-0.8, y=0.3, z=0.0)
    ]
    box_entities = [
        "obstacle_box_0_0",
        "obstacle_box_1_1",
        "obstacle_box_2_0"
    ]
    dingo_cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    cmd_vel = Twist()
    for loop_iteration in range(3):
        logger.info(f"\t* loop {loop_iteration + 1}/3")
        spawn_boxes(node, SPAWN_ENTITY_SERVICE, loop_iteration)
        pose = PoseStamped()
        pose.pose.position = Point(x=dingo_positions[loop_iteration][0],
                                   y=dingo_positions[loop_iteration][1],
                                   z=dingo_positions[loop_iteration][2])
        pose.pose.orientation = yaw_to_quaternion(dingo_positions[loop_iteration][3])
        logger.info("\tMoving Dingo to start position")
        set_entity_state(node, SET_ENTITY_STATE_SERVICE, "Dingo", pose)
        target_reached = False
        target_pos = target_positions[loop_iteration]

        # Push box until it reaches target position
        while not target_reached:
            # Get current box state
            box_state = get_entity_state(node, GET_ENTITY_STATE_SERVICE, format_entity_name(box_entities[loop_iteration], RENAME_ENTITY))
            if box_state is None:
                print("Failed to get box state, stopping loop")
                break
            
            current_pos = box_state.pose.position
            
            # Calculate distance to target
            dx = target_pos.x - current_pos.x
            dy = target_pos.y - current_pos.y
            if np.sqrt(dx**2 + dy**2) < 0.5:
                logger.info("\tBox reached target position")
                target_reached = True
            else:
                cmd_vel.linear.x = 0.5
                dingo_cmd_vel_pub.publish(cmd_vel)

        # Extra: move UR10
        logger.info("\tMoving UR10 joints")
        move_ur10_joints(node, loop_iteration, sim_backend)
        time.sleep(1)


def yaw_to_quaternion(yaw):
    """Convert a yaw angle (in radians) to a geometry_msgs.msg.Quaternion.

    Returns a Quaternion message with normalized components (w, x, y, z).
    """
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
    half_yaw = yaw / 2.0
    w = float(np.cos(half_yaw))
    x = 0.0
    y = 0.0
    z = float(np.sin(half_yaw))
    # Normalize to be safe
    norm = float(np.sqrt(w * w + x * x + y * y + z * z))
    if norm == 0.0:
        norm = 1.0
    q = Quaternion(w=w / norm, x=x / norm, y=y / norm, z=z / norm)
    return q

# ************************************
# Main loop
# ************************************


def main():
    # Map simulation differences
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-backend", choices=["isaacsim", "o3de", "gazebo"], default="isaacsim",
                        help="Choose which asset backend to use (isaacsim, o3de or gazebo).")
    args, unknown = parser.parse_known_args()

    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    logging.info("Starting warehouse simulation script")

    # Configure simulation backend (sets global variables)
    configure_simulation_backend(args.sim_backend)

    # Initialize ROS 2 node and client library
    rclpy.init()
    node = rclpy.create_node("simulation_interfaces_demo")
    
    # 1. Get features
    features = get_features(node, GET_FEATURES_SERVICE)
    if features is None:
        return
    
    # 1.1 Load world
    logging.info("1.1 Loading world (if supported)")
    if SimulatorFeatures.WORLD_LOADING in features.features:
        load_world(node, LOAD_WORLD_SERVICE, WORLD_URI)
        time.sleep(1.5)

    # 1.2 Spawn and despawn object
    logging.info("1.2 Spawning and despawning a table")
    initial_pose = PoseStamped()
    initial_pose.pose.position = Point(x=0.0, y=0.0, z=1.19)
    spawn_entity(node, SPAWN_ENTITY_SERVICE, TABLE_URI, format_entity_name("Table", RENAME_ENTITY), initial_pose)
    time.sleep(1.5)
    delete_entity(node, DELETE_ENTITY_SERVICE, format_entity_name("Table", RENAME_ENTITY))
    time.sleep(1.5)

    # 1.3 Spawn Table, some boxes, Dingo and UR10 with helper method
    logging.info("1.3 Spawning the full scene")
    spawn_scene(node, SPAWN_ENTITY_SERVICE)

    # 2.1 Play and pause simulation
    logging.info("2.1 Playing and pausing the simulation")
    set_simulation_state(node, SET_SIMULATION_STATE_SERVICE, SimulationState.STATE_PLAYING)
    time.sleep(3)
    set_simulation_state(node, SET_SIMULATION_STATE_SERVICE, SimulationState.STATE_PAUSED)

    # 2.2 Move cubes with set_entity_state
    logging.info("2.2 Moving cubes")
    pose = PoseStamped()
    pose.pose.position = Point(x=-0.8, y=-1.3, z=1.59)
    set_entity_state(node, SET_ENTITY_STATE_SERVICE, format_entity_name(f"blue_cube_0", RENAME_ENTITY), pose)
    pose.pose.position = Point(x=-1.2, y=-1.7, z=1.59)
    set_entity_state(node, SET_ENTITY_STATE_SERVICE, format_entity_name(f"blue_cube_3", RENAME_ENTITY), pose)
    pose.pose.position = Point(x=-1.0, y=-1.5, z=1.59)
    set_entity_state(node, SET_ENTITY_STATE_SERVICE, format_entity_name(f"blue_cube_4", RENAME_ENTITY), pose)
    pose.pose.position = Point(x=-1.2, y=-1.3, z=1.59)
    set_entity_state(node, SET_ENTITY_STATE_SERVICE, format_entity_name(f"red_cube_1", RENAME_ENTITY), pose)
    pose.pose.position = Point(x=-0.8, y=-1.7, z=1.59)
    set_entity_state(node, SET_ENTITY_STATE_SERVICE, format_entity_name(f"red_cube_2", RENAME_ENTITY), pose)
    pose.pose.position = Point(x=-0.9, y=-1.5, z=1.59) 
    set_entity_state(node, SET_ENTITY_STATE_SERVICE, format_entity_name(f"red_cube_5", RENAME_ENTITY), pose)
    time.sleep(1.5)

    # 2.3 Step simulation
    logging.info("2.3 Stepping the simulation")
    for _ in range(15):
        step_simulation(node, STEP_SIMULATION_SERVICE, 2)
        # do some work while stepping
        time.sleep(0.05)

    # 3.1 - 3.3: Loop that spawns boxes around the table, moves the robot and continues when robot reaches a certain pose.
    logging.info("3.x Looping the simulation with robot movement and box spawning")
    set_simulation_state(node, SET_SIMULATION_STATE_SERVICE, SimulationState.STATE_PLAYING)
    loop_simulation(node, args.sim_backend)
    time.sleep(3)

    # Unload world
    logging.info("Terminating: unloading world (if supported)")
    if SimulatorFeatures.WORLD_LOADING in features.features:
        unload_world(node, UNLOAD_WORLD_SERVICE)
    logging.info("Demo completed.")


if __name__ == "__main__":
    main()
