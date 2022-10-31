
#---------------------- QUESTION 1 ----------------------------
#--------------------------------------------------------------

"""
import gym

env = gym.make('CartPole-v1', render_mode="human")
env.reset()

for t in range(1000):
    env.render()
    # take a random action
    env.step(env.action_space.sample())
env.close()
"""

#---------------------- QUESTION 2 ----------------------------
#--------------------------------------------------------------
"""
import gym

env = gym.make('CartPole-v1', render_mode="human")

for i_episode in range(20):
    observation = env.reset()
    for t in range(1000):
        env.render()
        print("observation : ", observation)
        action = env.action_space.sample()
        observation, reward, done, info, _ = env.step(action)
        #print("reward : ", reward)
        #print("done : ", done)
        #print("info : ", info)
        if done:
            print("Episode finished after {} timesteps". format(t+1))
            break
env.close()

"""
#---------------------- QUESTION 5 ----------------------------
#--------------------------------------------------------------
"""
import gym
import random
import numpy as np
import matplotlib.pyplot as plt

env = gym.make('CartPole-v1', render_mode="human")

def rand_policy(obs):
    return random.randint(0, 1)

env.reset()
observation = np.array([0,0,0,0]) #initialiser "observation"

reward_cum = 0
iteration = []
reward_list = []
mean_reward_list = []
std_reward_list = []
for i_episode in range(20):
    env.reset()
    observation = np.array([0,0,0,0]) #initialiser "observation"

    for t in range(1000):
        env.render()
        action = env.action_space.sample()
        observation, reward, done, info, _ = env.step(rand_policy(observation))
        print("reward_cum : ", reward_cum)
        reward_cum+=reward
        
        
        if done:
            print("Episode finished after {} timesteps". format(t+1))
            iteration.append(i_episode)
            reward_list.append(reward_cum)
            mean_reward_list.append(np.mean(reward_list)) #créer une liste des moyennes
            std_reward_list.append(np.std(reward_list)) #créer une liste des std
            reward_cum=0
            break
        
mean_reward = np.mean(mean_reward_list) #calculer la moy de la liste des moyennes
print("mean reward = ", mean_reward)
std_reward = np.std(std_reward_list) #calculer la std de la liste des moyennes
print("std reward = ", std_reward)
env.close()

plt.plot(iteration, reward_list)
plt.xlabel('n° of episode')
plt.ylabel('reward_cum')
plt.show()


"""
#---------------------- QUESTION 6 ----------------------------
#--------------------------------------------------------------
"""
import gym
import random
import numpy as np
import matplotlib.pyplot as plt

env = gym.make('CartPole-v1', render_mode="human")

def theta_policy(obs):
    theta = obs[2]
    return 0 if theta < 0 else 1

env.reset()
observation = np.array([0,0,0,0]) #initialiser "observation"

reward_cum = 0
iteration = []
reward_list = []
mean_reward_list = []
std_reward_list = []

for i_episode in range(20):
    env.reset()
    observation = np.array([0,0,0,0]) #initialiser "observation"
    
    for t in range(1000):
        env.render()
        print("observation : ", observation[2])
        action = theta_policy(observation)
        observation, reward, done, info, _ = env.step(action)
        print("reward_cum : ", reward_cum)
        reward_cum+=reward
        
        if done:
            print("Episode finished after {} timesteps". format(t+1))
            iteration.append(i_episode)
            reward_list.append(reward_cum)
            mean_reward_list.append(np.mean(reward_list)) #créer une liste des moyennes
            std_reward_list.append(np.std(reward_list)) #créer une liste des std
            reward_cum=0
            break
        
mean_reward = np.mean(mean_reward_list) #calculer la moy de la liste des moyennes
print("mean reward = ", mean_reward)
std_reward = np.std(std_reward_list) #calculer la std de la liste des moyennes
print("std reward = ", std_reward)
env.close()

plt.plot(iteration, reward_list)
plt.xlabel('n° of episode')
plt.ylabel('reward_cum')
plt.show()
"""

#---------------------- QUESTION 7 ----------------------------
#--------------------------------------------------------------
"""
import gym
import random
import numpy as np
import matplotlib.pyplot as plt

env = gym.make('CartPole-v1', render_mode="human")

def omega_policy(obs):
    w = obs[3]
    return 0 if w < 0 else 1

env.reset()
observation = np.array([0,0,0,0]) #initialiser "observation"

reward_cum = 0
iteration = []
reward_list = []
mean_reward_list = []
std_reward_list = []

for i_episode in range(20):
    env.reset()
    observation = np.array([0,0,0,0]) #initialiser "observation"
    
    for t in range(1000):
        env.render()
        print("observation : ", observation[3])
        action = omega_policy(observation)
        observation, reward, done, info, _ = env.step(action)
        print("reward_cum : ", reward_cum)
        reward_cum+=reward
        
        if done:
            print("Episode finished after {} timesteps". format(t+1))
            iteration.append(i_episode)
            reward_list.append(reward_cum)
            mean_reward_list.append(np.mean(reward_list)) #créer une liste des moyennes
            std_reward_list.append(np.std(reward_list)) #créer une liste des std
            reward_cum=0
            break
        
mean_reward = np.mean(mean_reward_list) #calculer la moy de la liste des moyennes
print("mean reward = ", mean_reward)
std_reward = np.std(std_reward_list) #calculer la std de la liste des moyennes
print("std reward = ", std_reward)
env.close()

plt.plot(iteration, reward_list)
plt.xlabel('n° of episode')
plt.ylabel('reward_cum')
plt.show()
"""
























