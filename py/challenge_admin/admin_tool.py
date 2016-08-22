#!/usr/bin/env python3

import sys
import getopt
import requests
import json
from utils import utils
from enum import IntEnum, unique

@unique
class Status(IntEnum):
    created = 0
    fixed = 1
    false_positive = 2
    skipped = 3
    deleted = 4


class Challenge:
    def __init__(self, id):
        self.id = id
        self.tasks = {}
        self.types = []

    def add_task(self, task):
        if task['name'] not in self.tasks:
            self.tasks[task['name']] = task

    def contains_task(self, task):
        try:
            ret = self.tasks[task['name']]
        except KeyError:
            return None

        return ret


    def set_types(self, lst):
        self.types = lst


class admin_tool:
    def __init__(self, config, geojson):

        utils.status_begin('Initializing')

        # Read the config file into an object for access
        with open(config) as conf_file:
            conf = json.loads(conf_file.read())

        # Set up header for API interactions
        self.header = {'Content-Type': 'application/json',
                       'apiKey': conf['api_key']}
        # Get server url from config
        self.server_url = conf['server_url']

        # the challenges to hold the tasks
        self.challenges = {}

        # A list to hold the new tasks to be uploaded
        self.new_tasks = []

        # A list of tasks that exist but should have been fixed
        self.flagged_tasks = []

        for challenge in conf['challenges']:
            # Build dictionary of challenges
            self.challenges[challenge] = Challenge(challenge)
            # add the types it handles
            self.challenges[challenge].set_types(conf['challenges'][challenge])

        # Read in the new geojson
        with open(geojson) as geojson_file:
            self.geojson = json.loads(geojson_file.read())

        utils.status_done()


    def init_challenges(self):
        for challenge in self.challenges:
            # Download tasks for each challenge
            utils.status_begin('Downloading tasks from challenge ' + challenge)
            tasks = self.get_tasks_from_api(challenge).json()

            # Build a dictionary of challenge ids (name field from MR)
            for task in tasks:
                self.challenges[challenge].add_task(task)

            utils.status_done()


    def generate_tasks(self, geojson):
        '''generates tasks for the given geojson for the parent id specified'''
        # build a task for each item in the geojson array
        task_num = 1
        tasks = []
        for feature in geojson['features']:
            task = {
                        'name': str(feature['properties']['key']),
                        'parent': int(self.get_task_parent(feature)),
                        'status': 0,
                        'geometries':
                        {
                            'type': 'FeatureCollection',
                            'features':
                            [{
                                'type': 'Feature',
                                'geometry': feature['geometry'],
                                'properties': {}
                            }]
                        },
                        'instruction': feature['instruction']
                    }

            tasks.append(task)
            task_num += 1
        return tasks


    def get_tasks_from_api(self, challenge_id):
        '''http get up to 10000 tasks from maproulette'''
        payload = {'limit': 10000}
        response = requests.get('{}/api/v2/challenge/{}/tasks'.format(self.server_url, challenge_id), headers=self.header, params=payload)
        return response

    def upload_tasks(self, tasks):
        '''http POST a list of tasks to maproulette'''
        utils.status_begin('Uploading {} tasks'.format(len(tasks)))
        response = requests.post('{}/api/v2/tasks'.format(self.server_url), data=json.dumps(tasks), headers=self.header)
        utils.status_done()
        return response

    def update_tasks(self, tasks):
        '''http PUT a list of tasks to maproulette'''
        utils.status_begin('Uploading {} tasks'.format(len(tasks)))
        response = requests.put('{}/api/v2/tasks'.format(self.server_url), data=json.dumps(tasks), headers=self.header)
        utils.status_done()
        return response


    def headless_start(self):
        '''Begin a headless run (Requires that init_challenges has been called)'''
        # Get a list of the tasks from the new geojson
        tasks = self.generate_tasks(self.geojson)

        # If we haven't created any tasks for our challenges yet, do so now
        for challenge in self.challenges:
            if len(self.challenges[challenge].tasks) == 0:
                self.upload_tasks(tasks)
            else:
                self.compare_tasks(tasks)
        if len(self.new_tasks) != 0:
            self.upload_tasks(self.new_tasks)
        #self.compare_log(self.flagged)
        #self.update_log(self.flagged)


    def get_task_parent(self, task):
        '''Given a task, check it's type and match it against a challenge ID'''
        task_type = task['properties']['type']
        for challenge in self.challenges:
            if task_type in self.challenges[challenge].types:
                return challenge
            else:
                return None


    def compare_tasks(self, new_tasks):
        '''Compare the existing tasks downloaded from the api to the new tasks that have been generated'''
        for task in new_tasks:
            existing = self.challenges[str(task['parent'])].contains_task(task)
            if not existing:
                self.new_tasks.append(task)
                continue
            elif existing['status'] == Status.fixed:
                #flag for logging into file
                pass


    def read_recurring_tasks(self):
        pass


    def process_recurring_tasks(self, flagged_tasks):
        pass


    def flag_task(self, task):
        pass

if __name__ == '__main__':

    # headless is off by default
    config = ''

    # Set expected options and parse
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hc:i:',['help','config=','geojson='])
    except getopt.GetoptError:
        print('admin_tool.py --config conf/maproulette.json')
        sys.exit(2)

    # Use arguments and flags to set program parameters
    for opt, arg in opts:
        if opt in ('-h', '--help'):
            print('admin_tool.py --config conf/maproulette.json')
            sys.exit()
        elif opt in ('-c', '--config'):
            config = arg
        elif opt in ('-i', '--geojson'):
            geojson = arg

    admin = admin_tool(config, geojson)

    # Initialize the challenges from the geojson and MR API
    admin.init_challenges()

    # start the main part
    admin.headless_start()
