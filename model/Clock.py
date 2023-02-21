from events import Events
import numpy as np

class Clock:
    def __init__(self):
        """implement a clock, with an event 'tick', that all objects subscribe to """
        # this is the global time. each agent will also have their local time
        # the clock is also the global random number generator,
        self.time=0
        # differentiate between ticks and continuous time (keep things in integer values, stops there being any floating point errors)
        self.ticks = 0
        self.tickstep = 100 # this is in milliseconds
        self.ticks_per_second = 1000 # here we define that the clock runs in milliseconds
        self.rng=np.random.default_rng(1234)

        self.tick_events = Events()
        self.post_tick_events = Events()
        self.pre_tick_events = Events()


    # this is an event- don't need to do anything, just subscribe each agent to the function on initialisation
    def tick(self):
        self.pre_tick_events.on_change()
        self.ticks+=self.tickstep
        self.time=self.ticks/self.ticks_per_second
        self.tick_events.on_change()
        self.post_tick_events.on_change()

    def get_time(self):
        return self.time
