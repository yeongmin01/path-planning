import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objs as go
from mpl_toolkits.mplot3d import Axes3D


class node(object):
    def __init__(self, x, y, z, cost = 0, parent = None ):

        self.x = x
        self.y = y
        self.z = z
        self.arr = np.array([self.x, self.y, self.z])
        self.cost = cost
        self.parent = parent


class rrt_star():
    def __init__(self, x_init, x_goal, map, eta, w1, w2):

        self.map = map
        self.eta = eta
        self.x_init = x_init
        self.x_goal = x_goal
        self.nodes = [self.x_init]
        self.w1 = w1
        self.w2 = w2

    def Sampling(self):
        height = self.map.shape[0]
        width = self.map.shape[1]
        depth = self.map.shape[2]

        p = np.ravel(self.map) / np.sum(self.map)

        x_sample = np.random.choice(len(p), p=p)

        z = x_sample // (width * height)
        x = (x_sample - z*(width * height)) // width
        y = (x_sample - z*(width * height)) % width

        x = np.random.uniform(low=x - 0.5, high=x + 0.5)
        y = np.random.uniform(low=y - 0.5, high=y + 0.5)
        z = np.random.uniform(low=z - 0.5, high=z + 0.5)

        x_rand = node(x, y, z)

        return x_rand

    def Distance_Cost(self, start, end):

        distance_cost = np.linalg.norm(start.arr - end.arr)

        return distance_cost

    def Obstacle_Cost(self, start, end):

        seg_length = 1
        seg_point = int(np.ceil(np.linalg.norm(start.arr - end.arr) / seg_length))

        value = 0
        if seg_point > 1:
            v = (end.arr - start.arr) / (seg_point)

            for i in range(seg_point+1):
                seg = start.arr + i * v
                seg = np.around(seg)
                if 1 - self.map[int(seg[0]), int(seg[1]), int(seg[2])] == 1:
                    cost = 1 / (self.w2)

                    return cost
                else:
                    value += 1 - self.map[int(seg[0]), int(seg[1]), int(seg[2])]

            cost = value / seg_point

            return cost

        else:

            value = self.map[int(start.arr[0]), int(start.arr[1]), int(start.arr[2])] + self.map[int(end.arr[0]), int(end.arr[1]), int(end.arr[2])]
            cost = value / 2

            return cost

    # def Obstacle_Cost(self, start, end):
    #     line = self.Profile_line(self.map, start.arr, end.arr, 0.1)
    #     num = len(line)
    #
    #     cost = 1 - line
    #
    #     obs_cost = np.sum(cost) / num
    #
    #     return obs_cost

    def Line_Cost(self, start, end):

        cost = self.w1*(self.Distance_Cost(start, end)/(self.eta)) + self.w2*(self.Obstacle_Cost(start, end))

        return cost

    def Nearest(self, x_rand):

        vertex = []
        v = []
        i = 0
        for x_near in self.nodes:

            dist = self.Distance_Cost(x_near, x_rand)
            vertex.append([dist, i, x_near])
            i+=1

        vertex.sort()
        x_nearest = vertex[0][2]

        return x_nearest

    def Steer(self, x_rand, x_nearest):

        d = self.Distance_Cost(x_rand, x_nearest)

        if d < self.eta :
            x_new = node(x_rand.x, x_rand.y, x_rand.z)
        else:
            new_x = x_nearest.x + self.eta * ((x_rand.x - x_nearest.x)/d)
            new_y = x_nearest.y + self.eta * ((x_rand.y - x_nearest.y)/d)
            new_z = x_nearest.z + self.eta * ((x_rand.z - x_nearest.z)/d)

            x_new  = node(new_x, new_y, new_z)

        return x_new


    def Exist_Check(self, x_new):

        for x_near in self.nodes:
            if x_new.x == x_near.x and x_new.y == x_near.y and x_new.z == x_near.z:
                return False
            else :
                return True

    def New_Check(self, x_new):
        # print(x_new.arr)
        x_pob = np.array([x_new.x, x_new.y, x_new.z])
        x_pob = np.around(x_pob)

        if x_pob[0] >= self.map.shape[0]:
            x_pob[0] = self.map.shape[0] - 1
        if x_pob[1] >= self.map.shape[1]:
            x_pob[1] = self.map.shape[1] - 1
        if x_pob[2] >= self.map.shape[2]:
            x_pob[2] = self.map.shape[2] - 1

        x_pob = self.map[int(x_pob[0]), int(x_pob[1]), int(x_pob[2])]
        p = np.random.uniform(0, 1)
        # print(x_pob,p)
        if x_pob > p and self.Exist_Check(x_new):
            return True
        else:
            return False


    def Add_Parent(self, x_new, x_nearest):

        x_new.cost = x_nearest.cost + self.Line_Cost(x_nearest, x_new)
        x_new.parent = x_nearest
        for x_near in self.nodes:
            if self.Distance_Cost(x_near, x_new) <= self.eta : #and self.Obstacle_Cost(x_near, x_new) < 1 :
                if x_near.cost + self.Line_Cost(x_near, x_new) < x_nearest.cost + self.Line_Cost(x_nearest, x_new):
                    x_nearest = x_near
                    x_new.cost = x_nearest.cost + self.Line_Cost(x_nearest, x_new)
                    x_new.parent = x_nearest

        return x_new, x_nearest

    def Rewire(self, x_new):

        for x_near in self.nodes:
            if x_near is not x_new.parent:
                if self.Distance_Cost(x_near, x_new) <= self.eta : #and self.Obstacle_Cost(x_new, x_near) < 1 :
                    if x_new.cost + self.Line_Cost(x_new, x_near) < x_near.cost:

                        x_near.parent = x_new
                        x_near.cost = x_new.cost + self.Line_Cost(x_new, x_near)

        return

    def Get_Path(self):

        temp_path = []
        path = []
        n = 0
        for i in self.nodes:
            if self.Distance_Cost(i, self.x_goal) < 5:
                cost = i.cost + self.Line_Cost(self.x_goal, i)
                temp_path.append([cost,n, i])
                n += 1
        temp_path.sort()


        if temp_path == []:

            print("cannot find path")

            return None

        else:
            closest_node = temp_path[0][2]
            i = closest_node

            self.x_goal.cost = temp_path[0][0]
            print(self.x_goal.cost)

            while i is not self.x_init:
                path.append(i)
                i = i.parent
            path.append(self.x_init)

            self.x_goal.parent = path[0]
            path.insert(0,self.x_goal)
        # for i in path:
        #     if i is not self.x_init:
        #         plt.plot([i.x, i.parent.x], [i.y, i.parent.y], [i.z, i.parent.z], "r", linewidth=3)

            return path

    def Cost_Graph(self):

        temp_path = []
        path = []
        n = 0
        for i in self.nodes:
            if self.Distance_Cost(i, self.x_goal) < 5:
                cost = i.cost + self.Line_Cost(self.x_goal, i)
                temp_path.append([cost, n, i])
                n += 1
        temp_path.sort()

        if temp_path == []:
            return 0
        else:
            closest_node = temp_path[0][2]
            i = closest_node

            self.x_goal.cost = temp_path[0][0]

            return self.x_goal.cost

    # def Draw(self, path): # Use matplotlib for plotting
    #     fig = plt.figure(figsize=(8,8))
    #     ax =  fig.add_subplot(111, projection='3d')
    #     for i in self.nodes:
    #         if i is not self.x_init:
    #             ax.plot([i.x, i.parent.x], [i.y, i.parent.y], [i.z, i.parent.z], "b")
    #
    #     for i in path:
    #         if i is not self.x_init:
    #             ax.plot([i.x, i.parent.x], [i.y, i.parent.y], [i.z, i.parent.z], "r", linewidth=3)
    #
    #     plt.show()

    def Draw(self, path, map): # Use plotly for plotting

        goal_path = np.empty((0,3))
        data = []

        for i in range(len(map)): # Draw map
            if map[i][3] != 1:
                trace1 = go.Mesh3d(
                x = [ map[i][0]-0.5, map[i][0]-0.5, map[i][0]+0.5, map[i][0]+0.5, map[i][0]-0.5, map[i][0]-0.5, map[i][0]+0.5, map[i][0]+0.5 ],
                y = [ map[i][1]-0.5, map[i][1]+0.5, map[i][1]+0.5, map[i][1]-0.5, map[i][1]-0.5, map[i][1]+0.5, map[i][1]+0.5, map[i][1]-0.5 ],
                z = [ map[i][2]-0.5, map[i][2]-0.5, map[i][2]-0.5, map[i][2]-0.5, map[i][2]+0.5, map[i][2]+0.5, map[i][2]+0.5, map[i][2]+0.5 ],
                i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
                color=f'rgb({int(255*map[i][3])},{int(255*map[i][3])},{int(255*map[i][3])})',
                opacity = 1 - map[i][3] # 불투명도 0.70
                )
                data.append(trace1)

        for node in self.nodes: # Draw tree
            if node is not self.x_init:
                trace2 = go.Scatter3d(x=[node.x,node.parent.x], y=[node.y,node.parent.y], z=[node.z,node.parent.z], line=dict(color="blue", width=1),
                                      mode='lines')
                data.append(trace2)

        if path != None:
            for node in path: # Draw optimal path
                goal_path = np.append(goal_path, [node.arr], axis = 0)

            trace3 = go.Scatter3d(x=goal_path[:, 0], y=goal_path[:, 1], z=goal_path[:, 2], line=dict(color="red", width=5),
                                mode='lines')
            data.append(trace3)

        trace4 = go.Scatter3d(x=[self.x_goal.x], y=[self.x_goal.y], z=[self.x_goal.z], line=dict(width=10, color="green"), mode="markers")
        data.append(trace4)

        layout = go.Layout(title='3D Planning', showlegend=False)
        fig = go.Figure(data = data, layout = layout)
        fig.show()

    # def Check_can_connect_to_goal(self, path_iter):
    #
    #     x_nearest = self.Nearest(self.x_goal)
    #     if self.Distance_Cost(x_nearest, self.x_goal) < 5:
    #
    #         path_iter += 1
    #
    #         return path_iter
    #
    #     else:
    #         return path_iter

