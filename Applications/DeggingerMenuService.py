



class DrinkMenuService:

    def __init__(self):
        print("drinks init")

    def getContent(self):
        return ["Cola 1€", "Wasser 1€"]

class MealMenuService:

    def __init__(self):
        print("meals init")

    def getContent(self):
        return ["Pommes 1€", "Ketchup 1€"]

def main():
    d = DrinkMenuService()
    m = MealMenuService()
    print(d.getContent())
    print(m.getContent())

if __name__ == '__main__':
    main()