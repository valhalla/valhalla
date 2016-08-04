#!/usr/bin/env python3

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from models import Challenge, Task
from config import config

class db_tool:

    def __init__(self):

        # initialize the engine and session for database connections
        # when the object is created so it is always ready to go
        self.engine = create_engine(config.database_file)
        Session = sessionmaker(bind=self.engine)
        self.session = Session()


    def get_tasks(self):
        '''Gets the tasks that are part of the active challenge'''
        active_challenge = self.get_active_challenge()
        return active_challenge.tasks


    def get_challenges(self):
        '''Gets a list of all the challenges in the database'''
        return self.session.query(Challenge)


    def set_active_challenge(self, challenge):
        '''changes the active challenge in the database to the one specified'''

        # get the active challenge and change it to false
        active_challenge = self.get_active_challenge()
        active_challenge.active = False
        self.session.add(active_challenge)

        # set the new challenge to active
        challenge.active = True
        self.session.add(challenge)

        # commit the changes
        self.session.commit()


    def get_active_challenge(self):
        '''returns the active challenge in the database'''
        return self.session.query(Challenge).filter_by(active=1).first()
