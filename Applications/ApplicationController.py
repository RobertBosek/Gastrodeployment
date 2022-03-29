import sys
import numpy as np
import os

print(os.getcwd())

from .DeggingerEventService import DeggingerEventService
from .DeggingerMenuService import MealMenuService, DrinkMenuService

SOME_GLOBALS = None


class AppControl:
    service_dict = None

    def __init__(self):
        service_dict = self.init_app_services()
        print(service_dict)
        print("done")


    def __init_app_services(self):
        sdict = {}
        sdict[1] = MealMenuService()
        sdict[2] = DrinkMenuService()
        sdict[3] = DeggingerEventService()
        return sdict


def main():
    AppControl()
    sys.exit()


if __name__ == '__main__':
    main()
