import argparse

# the following road id sets define the chosen route
ROADS = set()
STRAIGHT = {8, 0, 1, 2, 3, 15, 5, 6, 7}
CURVE = {11, 13, 20, 14}
JUNCTION = {40, 41, 61, 62, 117, 118, 93, 94, 157, 158}
# JUNCTION_LANE={33,85,141}
# JUNCTION_LANE_MINUS={102,109,150,163,46,67,128}
ROADS.update(STRAIGHT)
ROADS.update(CURVE)
ROADS.update(JUNCTION)


# the flowing arguments set the simulation parameters
carla_args = argparse.ArgumentParser(
    description='CARLA Automatic Control Client')
carla_args.add_argument(
    '-v', '--verbose',
    action='store_true',
    dest='debug',
    default=False,
    help='Print debug information')
carla_args.add_argument(
    '-t',
    action='store_true',
    dest='train',
    default=True,
    help='Training Reinforcement agent')
carla_args.add_argument(
    '--host',
    metavar='H',
    default='127.0.0.1',
    help='IP of the host server (default: 127.0.0.1)')
carla_args.add_argument(
    '-p', '--port',
    metavar='P',
    default=2000,
    type=int,
    help='TCP port to listen to (default: 2000)')
carla_args.add_argument(
    '--res',
    metavar='WIDTHxHEIGHT',
    default='1280x720',
    help='Window resolution (default: 1280x720)')
carla_args.add_argument(
    '--sync',
    action='store_true',
    default=True,
    help='Synchronous mode execution')
carla_args.add_argument(
    '--fps', metavar='FPS',
    default=10, type=int,
    help="The fps of server running speed")
carla_args.add_argument(
    '--filter',
    metavar='PATTERN',
    default='vehicle.tesla.model3',
    help='Actor filter (default: "vehicle.*")')
carla_args.add_argument(
    '-l', '--loop',
    action='store_true',
    dest='loop',
    default='True',
    help='Sets a new random destination upon reaching the previous one (default: False)')
carla_args.add_argument(
    "-a", "--agent", type=str,
    choices=["Behavior", "Basic"],
    help="select which agent to run",
    default="Behavior")
carla_args.add_argument(
    '-b', '--behavior', type=str,
    choices=["cautious", "normal", "aggressive"],
    help='Choose one of the possible agent behaviors (default: normal) ',
    default='normal')
carla_args.add_argument(
    '-s', '--seed',
    help='Set seed for repeating executions (default: None)',
    default=None,
    type=int)
carla_args.add_argument(
    '-m', '--map', type=str,
    choices=['Town01_Opt', 'Town02_Opt'],
    help='Choose one of the possible world maps',
    default='Town01_Opt')
carla_args.add_argument(
    '-n', '--num_of_vehicles', type=list,
    help='Total vehicles number which run in simulation',
    default=[5,10,15,20])
carla_args.add_argument(
    '-sa', '--sampling_resolution', type=float,
    help='Distance between generated two waypoints',
    default=1.0)
carla_args.add_argument(
    '--tm-port',
    metavar='P',
    default=8000,
    type=int,
    help='Port to communicate with traffic manager (default: 8000)')
carla_args.add_argument(
    '--hybrid',
    action='store_true',
    default=True,
    help='Activate hybrid mode for Traffic Manager')
carla_args.add_argument(
    '--no_rendering',
    action='store_true',
    default=False,
    help='Activate no rendering mode')
carla_args.add_argument(
    '--stride', type=int,
    default=10,
    help='The number of upfront waypoints each state should include')
carla_args.add_argument(
    '--buffer-size', type=int,
    default=30,
    help='The number of look-ahead waypoints in each step')
carla_args.add_argument(
    '--TTC_th', type=float,
    default=4.001,
    help='TTC threshold')
carla_args.add_argument(
    '--penalty', type=float,
    default=20,
    help='reward penalty for simulation terminated early on account of collision and lane invasion')
carla_args.add_argument(
    '--speed_limit', type=float,
    default=72.0,
    help='Speed limit for ego vehicle, km/h')
carla_args.add_argument(
    '--speed_threshold', type=float,
    default=10.0,
    help='Speed limit for ego vehicle, km/h')
carla_args.add_argument(
    '--speed_min', type=float,
    default=1,
    help='When ego vehicle speed reaches down to this threshold, we should let basic agent take control \
        and the action of basic need to add into the replay buffer, km/h')
carla_args.add_argument(
    '--steer_bound', type=float,
    default=1.0,
    help='Steer bound for ego vehicle controller')
carla_args.add_argument(
    '--throttle_bound', type=float,
    default=1.0,
    help='Throttle bound for ego vehicle controller')
carla_args.add_argument(
    '--brake_bound', type=float,
    default=1.0,
    help='Brake bound for ego vehicle controller')
carla_args.add_argument(
    '--switch_threshold', type=int,
    default=20,
    help='Let the RL controller and PID controller alternatively take control every 500 steps')
carla_args.add_argument(
    '--pre_train_steps', type=int,
    default=10000,
    help='')
carla_args.add_argument(
    '--vehicle_proximity',type=float,
    default=30.0,
    help='Distance for searching vehicles in front of ego vehicle, unit -- meters')
carla_args.add_argument(
    '--min_distance',type=float,
    default=2.0,
    help='Min distance between two vehicles, unit -- meters')
carla_args.add_argument(
    '--pygame', type=bool,
    default=False,
    help='Render another pygame window for ego vehicle and the window style looks like automatic_control.py')
carla_args.add_argument(
    '--adapt', type=bool,
    default=True,
    help='Indicator for adaptive action duration, True -- action dim is 3, False -- action dim is 2')
