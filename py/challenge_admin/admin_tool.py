#!/usr/bin/env python3

import requests
import json
import config
from utils import utils
from db_tool import db_tool
from challenge_manager import challenge_interface


class admin_tool:

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
            if feature['properties']['type'] == 'Loop':
                task['instruction'] = 'This one way road loops back on itself. Edit it so that the road is properly accessible'
            else:
                task['instruction'] = 'This node is either unreachable or unleaveable. '
                task['instruction'] += 'Edit the surrounding roads so that the node can be accessed properly'

            # add the task to the list for later
            tasks.append(task)
            task_num += 1
        return tasks


    def get_tasks_from_api(self, challenge_id):
        '''http get up to 10000 tasks from maproulette'''
        payload = {'limit': 10000}
        response = requests.get('{}/api/v2/challenge/{}/tasks'.format(config.url, challenge_id), headers=config.header, params=payload)
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






if __name__ == '__main__':
    selection = None
    admin = admin_tool()
    while selection != 'q':
        utils.clear_console()
        selection = input(
'''Select One:
    [1] Manage Challenges
    [2] Upload tasks to a challenge
    [3] Manage Projects
    [q] Quit

==> ''')

        if (selection == '1'):
            challenge_interface()




'''
        if (selection == '1'):
            admin.manage('project', True)
        elif (selection == '2'):
            admin.manage('project')
        elif (selection == '3'):
            admin.manage('challenge', True)
        elif (selection == '4'):
            admin.manage('challenge')
        elif (selection == '5'):
            admin.create_upload_tasks()
        elif (selection == 'q'):
            break
        else:
            print("Please enter an option 1-6")
'''

