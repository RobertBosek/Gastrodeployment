import sys
import numpy as np

from .DeggingerEventService import DeggingerEventService as EventService
from .DeggingerMenuService import MealMenuService, DrinkMenuService

SOME_GLOBALS = None


class CubeControl:
    service_dict=None

    def __init__(self):
        service_dict = self.init_app_services()


    def __init_app_services(self):
        sdict = {}
        sdict[1] = MealMenuService()
        sdict[2] = DrinkMenuService()
        sdict[3] = EventService()
        return {}


def main():
    CubeControl()
    sys.exit()


if __name__ == '__main__':
    main()
