#! /usr/bin/env python

'''
A meta-script that looks at all config files for benchmarking, and based on the names
of each file, runs them and combines the benchmarks for the same scenes but different
planners.

usage: python meta_planner_script.py (runs on all scenes and all steps)
       python meta_planner_script.py --scenes andrewstable raw (one specific scene, one step)
'''
import argparse
import datetime
import os
import random
import re
import subprocess
import logging
from collections import defaultdict
import rospkg

LOG_LOC = "/home/brycew/opt_benchmarks/"
RAW_LOG_LOC = LOG_LOC + "raw/"
RANDOM_FOLDER = '-REPLACE-' + str(random.randint(0, 10000))
rospack = rospkg.RosPack()
PLANNER_PARAMS_PATH = str(rospack.get_path('ur5_robotiq85_moveit_config')) + \
                          "/benchmarks/bench_scene_planner_params/"

def time_str():
    '''
    Returns current time as a string, formated the same way as MoveIt! benchmark
    logs are.
    '''
    return datetime.datetime.now().strftime("%Y-%m-%dCST%H:%M:%S")

def find_all_scenes(planner_params_path=PLANNER_PARAMS_PATH):
    ''' Finds all scenes that could be benchmarked.
    '''
    return [filename.split('-')[0] for filename in os.listdir(planner_params_path)]

def find_all_planners_for_scene(query_scene, planner_params_path=PLANNER_PARAMS_PATH):
    ''' Finds all planners that should be run for a specific scene.
    '''
    planner_list = []
    for filename in os.listdir(planner_params_path):
        scene, planner = filename.split('-')
        if scene == query_scene:
            planner = planner[:-len('.yaml')]
            planner_list.append(planner)
    return planner_list

def make_scene_planner_combinations(planner_params_path=PLANNER_PARAMS_PATH):
    ''' Iterates through all files to find the planners to run for each scene.
    '''
    scene_to_planner_list = defaultdict(list)
    for filename in os.listdir(planner_params_path):
        scene, planner = filename.split('-')
        planner = planner[:-len('.yaml')]
        scene_to_planner_list[scene].append(planner)
    return scene_to_planner_list

def generate_raw_benchmarks(scene, planner_list=None):
    '''
    Generates new data from benchmarking planners, but does not post process it.
    '''
    if not planner_list:
        planner_list = find_all_planners_for_scene(scene, PLANNER_PARAMS_PATH)
    for planner in planner_list:
        subprocess.call(["roslaunch",
                         "ur5_robotiq85_moveit_config",
                         "ur5_bench.launch",
                         "bench_opts:= {0}/{1}-{2}.yaml".format(
                             PLANNER_PARAMS_PATH, scene, planner),
                         "planner:={}".format(planner),
                         "collisions:={}".format("FCL")])

def concat_benchmarks(scene, _=None):
    '''
    Combines existing benchmark logs for scene. Planner_list and planner_params_path aren't used,
    but they keep the same interface as the other functions.
    '''
    log_list = [filename for filename in os.listdir(RAW_LOG_LOC)
                if filename.startswith(scene + '-')]
    planner_data_str = ''
    planners = 0
    planner_pattern = r'(\d+) planners'
    if not log_list:
        logging.error("Can't find any log files for " + scene)
        return ''
    with open(RAW_LOG_LOC + log_list[0], 'r') as first_log:
        new_log = re.split(planner_pattern, first_log.read())[0]
    for log in log_list:
        with open(RAW_LOG_LOC + log, 'r') as log_file:
            _, number, post = re.split(planner_pattern, log_file.read())
            planners += int(number)
            planner_data_str += '\n' + post.strip()
        # Delete the old, to-be concated, log.
        os.remove(RAW_LOG_LOC + log)
    new_log += str(planners) + ' planners' + planner_data_str
    log_name = scene + '-' + time_str() + '.log'
    new_log_file = open(LOG_LOC + log_name, 'w')
    new_log_file.write(new_log)
    new_log_file.close()
    return log_name

def move_benchmarks(scene, log_name):
    '''
    Moves the concatenated log into the 'official' folder.
    '''
    organizing_folder = LOG_LOC + datetime.datetime.now().strftime('%Y-%m-%d') + RANDOM_FOLDER
    if not os.path.isdir(organizing_folder):
        # Make the folder so we can use it.
        os.mkdir(organizing_folder)
    # Move the log into the random folder.
    os.rename(LOG_LOC + log_name, organizing_folder + '/' + log_name)

def convert_log_to_pdf(scene, log_name):
    '''
    Turns the concat log into a database/pdf.
    '''
    organizing_folder = LOG_LOC + datetime.datetime.now().strftime('%Y-%m-%d') + RANDOM_FOLDER
    # Call ompl's benchmark statistics.
    subprocess.call(["ompl_benchmark_statistics.py",
                     organizing_folder + '/' + log_name,
                     "-d",
                     organizing_folder + '/' + scene + '.db'
                    ])
    subprocess.call(["ompl_benchmark_statistics.py",
                     "-d",
                     organizing_folder + '/' + scene + '.db',
                     "-p",
                     organizing_folder + '/' + scene + '.pdf'
                    ])

def postprocess_benchmarks(scene, _=None):
    '''
    Runs all processing on the log files that happens after the actual
    benchmarks are finished.
    '''
    log_name = concat_benchmarks(scene)
    move_benchmarks(scene, log_name)
    convert_log_to_pdf(scene, log_name)

def run_scene_benchmarks(scene, planner_list=None):
    '''
    Runs all steps of the benchmark, get new data, combine it together, for a
    particular scene.
    '''
    if not planner_list:
        planner_list = find_all_planners_for_scene(scene, PLANNER_PARAMS_PATH)
    generate_raw_benchmarks(scene, planner_list)
    postprocess_benchmarks(scene)

def run_benchmarks():
    ''' Finds and runs all planners for each scene
    '''
    scene_to_planner_list = make_scene_planner_combinations(PLANNER_PARAMS_PATH)
    for scene, planner_list in scene_to_planner_list.items():
        run_scene_benchmarks(scene, planner_list)

def main():
    ''' Parses arguments and runs benchmarks based on arguments.
    '''
    parser = argparse.ArgumentParser(description='Run MoveIt! Benchmarks over multiple ' +
                                     'planning plugins and scenes.')
    parser.add_argument('--scene', nargs=1, help="scenes to run benchmarks on")
    subparser = parser.add_subparsers(help='Commands')
    all_parser = subparser.add_parser('all', help=run_scene_benchmarks.__doc__)
    all_parser.set_defaults(func=run_scene_benchmarks)

    raw_parser = subparser.add_parser('raw', help=generate_raw_benchmarks.__doc__)
    raw_parser.set_defaults(func=generate_raw_benchmarks)

    post_parser = subparser.add_parser('postprocess', help=postprocess_benchmarks.__doc__)
    post_parser.set_defaults(func=postprocess_benchmarks)

    args = parser.parse_args()
   
    if not args.func:
        logging.error('ERROR: You need to give a real command!')
        quit()

    # Awfully hardcoded collision checkers.
    for collisions in ['FCL', 'Distance_Field']:
        if not args.scene:
            scene_to_planner_list = make_scene_planner_combinations(PLANNER_PARAMS_PATH)
            for scene, planner_list in scene_to_planner_list.items():
                args.func(scene, planner_list, PLANNER_PARAMS_PATH)
        else:
            args.func(args.scene[0])

if __name__ == '__main__':
    main()

# TODO: consider doing
# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# launch = roslaunch.parent.ROSLaunchParent(uuid, ["/.../test_node.launch"])
# launch.start()
# launch.spin()