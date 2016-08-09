import requests
import json
from utils import utils
from db_tool import db_tool
from models import Challenge
import config

def challenge_interface():

    # database tool to use with challenges
    dbtool = db_tool()

    action = None
    while action != 'q':
        list_challenges(dbtool)
        action = input(
'''Available actions:
    [a] Change active challenge
    [c] Create challenge
    [d] Delete challenge
    [e] Edit a challenge
    [q] Finished updating challenges

==> ''')
        if action == 'a':
            set_active_challenge(dbtool)
        elif action == 'c':
            challenge_builder(dbtool, create=True)
        elif action == 'd':
            delete_challenge(dbtool)
        elif action == 'e':
            challenge_builder(dbtool)


def set_active_challenge(dbtool):
    '''presents a list of challenges to the user and allows them to choose the new challenge'''

    # get challenge from selector
    selected_challenge = challenge_selector(dbtool)

    # update database with new active challenge
    dbtool.set_active_challenge(selected_challenge)


def challenge_builder(dbtool, create=False):
    '''manages the creation and update of challenges'''

    print('Enter each field or leave blank to skip')
    payload = {}

    challenge = None

    # if it's being updated we need the challenge from the database
    if not create:
        challenge = challenge_selector(dbtool)

    # fill in all the fields for the api call
    for field in config.fields:

        # if it's a required field, make sure it's filled in
        data = ''
        if field in config.required_fields:
            while not data:
                print(field, 'is required')
                data = input(field + ': ')
        else:
            data = input(field + ': ')
        if data:
            payload[field] = data

    # Send off the payload to the appropriate endpoint
    response = None
    if create:
        # build the challenge and insert into the database
        response = requests.post('{}/api/v2/challenge'.format(config.url),
                                 data=json.dumps(payload), headers=config.header)

        # get the maproulette id from the response and build it into the challenge
        challenge = Challenge(name=payload['name'], mr_id = response.json()['id'], instruction=payload['instruction'], active=False)
        dbtool.insert_challenge(challenge)

    else:
        # update the fields in the challenge and update in the database and on Map Roulette
        challenge.instruction = payload['instruction']
        challenge.name = payload['name']
        dbtool.insert_challenge(challenge)

        response = requests.put('{}/api/v2/challenge/{}'.format(config.url, challenge.mr_id),
                                data=json.dumps(payload), headers=config.header)


def delete_challenge(dbtool):
    '''selects then deletes a challenge'''
    #get challenge from selector
    selected_challenge = challenge_selector(dbtool)

    # delete from db
    dbtool.delete_challenge(selected_challenge)

    requests.delete('{}/api/v2/challenge/{}'.format(config.url, selected_challenge.mr_id), headers=config.header)


#### These are some utility functions used by several of the other modifier functions


def list_challenges(dbtool):
    'clear out the console and print the challenges'
    utils.clear_console()

    # get a list of challenges using the db_tool
    challenges = dbtool.get_challenges()
    print('Challenges:')
    for challenge in challenges:
        if challenge.active == 0:
            print('    {}'.format(challenge))
        else:
            utils.print_active(challenge)
    print()


def challenge_selector(dbtool):
    '''presents a list of challenges and asks for input to choose one'''

    selected_challenge = None

    # until we have a valid selection
    while selected_challenge == None:
        # list challenges
        list_challenges(dbtool)
        print('The active challenge is highlighted in blue,')
        print('Enter the number of the challenge you wish to make the active challenge.\n')

        # get input and retrieve from the database
        challenge_num = input('==> ')
        selected_challenge = dbtool.get_challenge(challenge_num)

    return selected_challenge

