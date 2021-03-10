import sys
sys.path.insert(1,"/c/Users/Asus/gym")
sys.path.insert(1,"/c/Users/Asus/gym/gym/envs/robotics")
import gym 
#from gym.envs.robotics.fetch.pick_and_place import FetchPickAndPlaceEnv
from gym.envs.robotics.fetch.reach import FetchReachEnv


#env = FetchPickAndPlaceEnv()
env = FetchReachEnv()
env.reset()
#for _ in range(10000):
env.render()
#x=env.get_observation()
#env.set_observation([x[0],x[1],0])
#    print(f"action:{env.action_space.sample()}")
#    action = env.action_space.sample() # take a random action
#    observation, reward, done, info = env.step(action)
#    print(action)    
    
    
#env.close()
