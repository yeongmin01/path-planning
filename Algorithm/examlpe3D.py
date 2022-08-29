import numpy as np
import time
from generate_map import map, Reshape_map
from rrt_star3D import node, rrt_star



eta = 5
sample_num = 3000
sample_taken = 0
total_iter = 0
x_init = node(0, 0, 0)
x_init.parent = x_init
x_goal = node(30, 30, 30)
filter_size = 0 # 1 = 3x3, 2 = 5x5, 3 = 7x7
classify = False
distance_weight = 0.5
obstacle_weight = 0.5

Graph_sample_num = 0
Graph_data = np.array([[0,0]])

map = map()
rrt = rrt_star(x_init, x_goal, map, eta, distance_weight, obstacle_weight)

s = time.time()
t1, t2, t3 = 0, 0, 0
np.random.seed(0)
while(sample_taken < sample_num):

    s1 = time.time()
    while True:

        x_rand = rrt.Sampling()
        # x_rand  = rrt.Uniform_sampling()
        total_iter += 1
        # print(x_rand.arr)
        x_nearest = rrt.Nearest(x_rand)
        #print(x_nearest.arr)
        x_new = rrt.Steer(x_rand, x_nearest)
        #print(x_new.arr)
        b = rrt.New_Check(x_new)
        if b == True :
            break
    e1 = time.time()
    t1 += (e1 - s1)

    sample_taken += 1
    if sample_taken%100 == 0:
        print(sample_taken,"Sample Taken")

    s2 = time.time()
    x_new , x_nearest = rrt.Add_Parent(x_new, x_nearest)
    e2 = time.time()
    t2 += (e2 - s2)

    rrt.nodes.append(x_new)
    s3 = time.time()
    rrt.Rewire(x_new)
    e3 = time.time()
    t3 += (e3 - s3)
#     Graph_sample_num += 1
#     if Graph_sample_num%100 == 0:
#         Graph_cost = rrt.Cost_Graph()
#         Graph_data = np.append(Graph_data, np.array([[Graph_sample_num, Graph_cost]]), axis = 0)
#
#
# np.save("data/data0", Graph_data)

e = time.time()
print("total time : ", e - s,"초")
print("sampling time : ", t1,"초", (t1*100)/(e-s),"%")
print("Add_Parent time : ", t2,"초", (t2*100)/(e-s),"%")
print("Rewire time : ", t3,"초", (t3*100)/(e-s),"%")
print("total_iteration = ", total_iter)
path = rrt.Get_Path()
rrt.Draw(path, Reshape_map(map))
