# flake8: noqa
import os
import rospkg

# Had to use rospkg, os.path.dirname(__file__) wasn't working
for module in os.listdir(os.path.join(rospkg.RosPack().get_path("navigator_missions"), 'nav_missions/')):
    if module == '__init__.py' or module[-3:] != '.py':
        continue
    __import__(module[:-3], locals(), globals())

del rospkg
del module
del os
