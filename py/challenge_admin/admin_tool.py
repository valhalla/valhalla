#!/usr/bin/env python3

import sys
import getopt
import requests
import json
from utils import utils


class admin_tool:
    def __init__(self, config):

        utils.status_begin('Initializing')

        # Read the config file into an object for access
        with open(config) as conf_file:
            conf = json.loads(conf_file.read())

        # Set up header for API interactions
        self.header = {'Content-Type': 'application/json',
                       'apiKey': conf['api_key']}
        # Get server url from config
        self.server_url = conf['server_url']

        self.challenges = {}
        challenge_ids = conf['challenges']
        for challenge in challenge_ids:
            # Build dictionary of challenge types
            self.challenges[challenge] = conf['challenges'][challenge]

        utils.status_done()


    def init_challenges(self):
        self.challenge_tasks = {}
        for challenge in self.challenges:
            # Download tasks for each challenge
            utils.status_begin('Downloading tasks from challenge ' + challenge)
            tasks = self.get_tasks_from_api(challenge).json()
            self.challenge_tasks[challenge] = tasks
            utils.status_done()


    def create_upload_tasks(self):
        '''Reads a geojson file specified in config, then generates and uploads the tasks to maproulete'''

        # open the file with our task geojson
        with open(config.geojson_file) as task_file:
            geojson = json.loads(task_file.read())

        # we'll need the parent id for the api call
        parent = int(input('Enter parent id: '))

        tasks = self.generate_tasks(geojson, parent)

        while len(tasks) > 10000:
            batch = tasks[:10000]
            tasks = tasks[10000:]
            self.upload_tasks(batch)
        self.upload_tasks(tasks)


    def generate_tasks(self, geojson, parent):
        '''generates tasks for the given geojson for the parent id specified'''
        # build a task for each item in the geojson array
        task_num = 1
        tasks = []
        for feature in geojson['features']:
            task = {
                        'name': 'task-' + str(task_num),
                        'identifier': 'test-' + str(task_num),
                        'parent': parent,
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
                        }
                    }

            # change some values depending on what type of task we built
            task_type = feature['properties']['type']
            task['instruction'] = geojson['properties']['instructions'][task_type]
            # add the task to the list for later
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
        print('Uploading {} tasks'.format(len(tasks)))
        response = requests.post('{}/api/v2/tasks'.format(config.url), data=json.dumps(tasks), headers=config.header)
        return response

    def update_tasks(self, tasks):
        '''http PUT a list of tasks to maproulette'''
        print('Uploading {} tasks'.format(len(tasks)))
        response = requests.put('{}/api/v2/tasks'.format(config.url), data=json.dumps(tasks), headers=config.header)
        return response


    def init_headless_mode(self):
        self.init_challenges()


if __name__ == '__main__':

    # headless is off by default
    config = ''
    headless = False

    # Set expected options and parse
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hc:H',['help','config=','headless'])
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
        elif opt in ('-H', '--headless'):
            headless = True

    admin = admin_tool(config)

    # start in the correct mode
    if headless:
        admin.init_headless_mode()
        admin.headless_start()
    else:
        pass
