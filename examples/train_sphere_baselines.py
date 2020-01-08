from baselines_tools import *
import os

def main():
    seed = int(time.time())
    np.random.seed(seed)
    # train the model
    path = None
    if os.path.isdir(PATH_MODEL_1):
        path = PATH_MODEL_1
    elif os.path.isdir(PATH_MODEL_2):
        path = PATH_MODEL_2
    else:
        print("Can't find path to model: %s or %s"
              %(PATH_MODEL_1, PATH_MODEL_2))
        return -1
    train(num_timesteps=NB_ITER, seed=seed,
          model_path=path)

if __name__ == '__main__':
    main()