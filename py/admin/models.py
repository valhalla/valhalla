from sqlalchemy import Column, ForeignKey, Integer, String, BigInteger
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import create_engine
from sqlalchemy.orm import relationship
from config import config

Base = declarative_base()

class Challenge(Base):
    __tablename__ = 'challenges'
    id = Column(Integer, primary_key=True)
    name = Column(String(64), nullable=False)
    instruction = Column(String(512), nullable=False)
    active = Column(Integer, nullable=False)

    # points to all the tasks that belong to this challenge
    tasks = relationship('Task', backref='challenge', lazy='dynamic')

    # in case we print a challenge
    def __repr__(self):
        return 'Challenge %r - %r' % (self.name, self.instruction)


class Task(Base):
    __tablename__ = 'tasks'
    id = Column(BigInteger, primary_key=True)
    mr_id = Column(Integer, index=True)
    name = Column(String(64), nullable=False)
    instruction = Column(String(512), nullable=False)
    status = Column(Integer, nullable=False)
    challenge_id = Column(Integer, ForeignKey('challenges.id'))

    # in case we want to print a task
    def __repr__(self):
        return 'Task %r - %r' % (self.name, self.instruction)


# if we run this file directly create the database
if __name__ == '__main__':
    engine = create_engine(config.database_file)
    # create the tables
    Base.metadata.create_all(engine)
