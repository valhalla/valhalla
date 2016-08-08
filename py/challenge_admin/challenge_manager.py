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
    [e] Edit a challenge
    [s] Synchronize with Map Roulette
    [q] Finished updating challenges

==> ''')
        if action == 'a':
            set_active_challenge(dbtool)
        elif action == 'c':
            modify_challenge(dbtool, create=True)
        elif action == 'e':
            #edit_challenge()
            pass
        elif action == 's':
            #synchronize_remote()
            pass


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



def modify_challenge(dbtool, create=False):
    '''manages the creation and update of challenges'''

    print('Enter each field or leave blank to skip')
    payload = {}

    # if it's being updated we need it's ID
    if not create:
        # Needs the ID field if being updated
        config.fields.append('id')

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

    challenge = Challenge(name=payload['name'], instruction=payload['instruction'], active=False)
    dbtool.insert_challenge(challenge)


def set_active_challenge(dbtool):
    '''presents a list of challenges to the user and allows them to choose the new challenge'''

    success = False
    selected_challenge = None
    while not success:
        list_challenges(dbtool)
        print('The active challenge is highlighted in blue,')
        print('Enter the number of the challenge you wish to make the active challenge.\n')

        challenge_num = input('==> ')
        selected_challenge = dbtool.get_challenge(challenge_num)
        if selected_challenge != None:
            success = True

    dbtool.set_active_challenge(selected_challenge)




