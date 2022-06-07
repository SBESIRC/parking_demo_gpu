class UserData:
    Type: str = ""
    DataBase: dict
    NextId: int

    def __init__(self, obj, *args, **kwargs):
        if kwargs.get("Id", None) is None:
            # i = 0
            # while self.DataBase.get(i) is not None:
            #     i += 1
            # self.Id = i
            self.Id = self.NextId; self.NextId += 1
        else:
            self.Id = kwargs.get("Id")

        self.DataBase[self.Id] = obj
        for k, v in kwargs.items():
            self.__setattr__(k, v)
    
    def __del__(self):
        del self.DataBase[self.Id]

class UserDataMeta(type):
    def __new__(cls, *args, **kwargs):
        name, bases, attrs = args[:3]
        kwargs['Type'] = name.replace('UserData', '')
        kwargs['DataBase'] = {}
        kwargs['NextId'] = 0
        # kwargs['__init__'] = lambda self, obj, *args, **kwargs: UserData.__init__(self, obj, *args, **kwargs)
        newUserData = super(UserDataMeta, cls).__new__(cls, name, (UserData, *bases), kwargs)
        print(newUserData.__init__)
        return newUserData

class WallUserData(metaclass=UserDataMeta):
    pass

class LaneNodeUserData(metaclass=UserDataMeta):
    pass

class LaneBodyUserData(metaclass=UserDataMeta):
    pass

class JointUserData(metaclass=UserDataMeta):
    pass
