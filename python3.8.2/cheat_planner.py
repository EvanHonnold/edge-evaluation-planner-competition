#!/usr/bin/env python3

import time
import pathlib
import json
import copy
from typing import List, Tuple, Dict

THIS_FILE_PATH = pathlib.Path(__file__).resolve()
REQUEST_FILE = THIS_FILE_PATH.parent / \
    (THIS_FILE_PATH.stem + "_edge_request.json")
RESPONSE_FILE = THIS_FILE_PATH.parent / \
    (THIS_FILE_PATH.stem + "_edge_response.json")
SUBMISSION_FILE = THIS_FILE_PATH.parent / \
    (THIS_FILE_PATH.stem + "_path_submission.json")

Edge = Tuple[Tuple[float, float], Tuple[float, float]]

################################################
#                 Utilities                    #
################################################


def write_request(edges: List[Edge]) -> None:
    with open(REQUEST_FILE, 'w') as f:
        json.dump(edges, f)


def write_submission(path: List[Edge]) -> None:
    """ Submit a path. If this path reaches from the start to the goal, 
        and does not collide with obstacles, then this planner will be 
        declared the winner. """
    with open(SUBMISSION_FILE, 'w') as f:
        json.dump(path, f)


def read_response() -> Dict[Edge, bool]:
    """ Returns a dictionary mapping each edge to whether it is obstructed 
        (True -> blocked). """
    count = 0
    while not RESPONSE_FILE.exists():
        count += 1
        time.sleep(0.1)
        if count % 30 == 0:
            print("Waiting for response...")

    with open(RESPONSE_FILE, 'r') as f:
        data = json.load(f)

    # tuples can't be keys in json dictionaries,
    # so we must do some reconstruction:
    response = dict()
    for item in data:
        is_blocked = item[1]
        edge = item[0]
        start, end = edge
        sx, sy = start
        ex, ey = end
        response[((sx, sy), (ex, ey))] = is_blocked

    RESPONSE_FILE.unlink()
    return response

####################################################################
#                      Interactive Component                       #
####################################################################

if __name__ == "__main__":

    # clean up any old files lingering from previous runs
    if REQUEST_FILE.exists():
        REQUEST_FILE.unlink()

    # Hardcoded edges for the problem generated from random.seed(3).
    # They form a path from the start to the goal.
    cheaty_edges = [((0.0, 0.0), (0.1, 0.3)), ((0.1, 0.3), (0.1, 0.6)), ((0.1, 0.6), (0.18, 0.98)),
                    ((0.18, 0.98), (0.3, 0.65)), ((0.3, 0.65), (0.3, 0.6)), ((0.3, 0.6), (0.6, 0.7)), ((0.6, 0.7), (0.9, 0.7)), ((0.9, 0.7), (1.0, 1.0))]
    edges_to_request = []
    for edge in copy.deepcopy(cheaty_edges):
        edges_to_request.append(edge)

    cycle_count = 0
    while True:

        print(f"cycle count: {cycle_count}")

        if len(edges_to_request) > 0:
            request: List = []
            if len(edges_to_request) >= 1:
                request.append(edges_to_request.pop(0))
            print(f"Requesting edges: {request}")
            write_request(request)
        else:
            write_submission(cheaty_edges)
            # we generally wouldn't write a request after writing a submission,
            # but doing so here allows the competition to continue if the environment
            # has changed (of course, this planner won't win...)
            write_request(cheaty_edges[0:1])

        response = read_response()
        print(f"Received response: {response}")

        cycle_count += 1

    # planners must clean up their own requests,
    # since requests are used to start the simulation:
    if REQUEST_FILE.exists():
        REQUEST_FILE.unlink()
