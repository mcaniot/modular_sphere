from baselines_tools import *
import os

def main():
    path = None
    if os.path.isdir(PATH_MODEL_1):
        path = PATH_MODEL_1
    elif os.path.isdir(PATH_MODEL_2):
        path = PATH_MODEL_2
    else:
        print("Can't find path to model: %s or %s"
              %(PATH_MODEL_1, PATH_MODEL_2))
        return -1
    visualize(path + "/" + AGENT + "_" + str(NB_ITER) + ".pkl")

if __name__ == '__main__':
    main()