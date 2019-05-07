import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.animation as animation

class Robot:
    def __init__(self, filename):
        f = open(filename, 'r')
        need_read = True
        self.bone = {}
        self.name_to_id = {'root':0}
        self.edge = []
        while True:
            if need_read:
                l = f.readline()
            need_read = True
            if not l:
                break
            if l[0] == '#':
                continue
            if l[0] == ':':
                if l.count('version') > 0 or l.count('name') > 0:
                    continue
                if l.count('units') > 0:
                    l = f.readline().strip()
                    l = l.split()
                    #print(l)
                    self.mass = float(l[1])
                    l = f.readline().strip()
                    l = l.split(' ')
                    self.length = float(l[1])
                    l = f.readline().strip()
                    l = l.split(' ')
                    self.angle = l[1]
                    continue
                if l.count('documentation') > 0:
                    l = f.readline()
                    l = f.readline()
                    continue
                if l.count('root') > 0:
                    l = f.readline()
                    l = f.readline()
                    l = f.readline()
                    l = f.readline()
                    continue
                if l.count('bonedata') > 0:
                    id = 0
                    while True:
                        l = f.readline()
                        if l[0] == ":":
                            need_read = False
                            break
                        if l.count('begin') > 0:
                            l = f.readline().strip()
                            bone = {}
                            id = int(l.split()[1])
                            l = f.readline().strip()
                            bone["name"] = l.split()[1]
                            self.name_to_id[bone["name"]] = id
                            l = f.readline().strip()
                            bone["direction"] = [l.split()[1], l.split()[2], l.split()[3]]
                            bone["length"] = float(f.readline().split()[1])
                            l = f.readline().strip()
                            bone["axis"] = [l.split()[1], l.split()[2], l.split()[3]]
                            self.bone[id] = bone
                            continue
                        if l.count('end') > 0:
                            continue
                        if l.count('dof') > 0:
                            self.bone[id]["dof"] = l.split()[1:]
                            dof_n = len(l.split())
                            for _ in range(dof_n):
                                l = f.readline()
                            continue
                if l.count('hierarchy') > 0:
                    l = f.readline()
                    while True:
                        l = f.readline().strip()
                        if l.count('end') > 0:
                            break
                        l = l.split(' ')
                        for i in range(1, len(l)):
                            self.edge.append(( self.name_to_id[l[0]], self.name_to_id[l[i]]))

    def bone_transform(self, bone, inv=0):
        rot = np.array(bone['axis'], dtype = np.float)/180*np.pi
        rot_x = np.array([[1, 0, 0], [0, np.cos(rot[0]), -np.sin(rot[0])], [0, np.sin(rot[0]), np.cos(rot[0])]])
        rot_y = np.array([[np.cos(rot[1]), 0, np.sin(rot[1])], [0, 1, 0], [-np.sin(rot[1]), 0, np.cos(rot[1])]])
        rot_z = np.array([[np.cos(rot[2]), -np.sin(rot[2]), 0], [np.sin(rot[2]), np.cos(rot[2]), 0], [0, 0, 1]])
        
        return rot_z @ rot_y @ rot_x if inv == 0 else rot_x.T @ rot_y.T @ rot_z.T
    
    def cal_motion(self, rot, M, pts0, rot0):
        pts = {0: pts0}

        one_rot = np.zeros((3, 3))
        for i in range(3):
            one_rot[i, i] = 1
        P = {0: rot0}
        for e in self.edge:
            #P Present Global Transform
            #rot Global to past Local 
            #M Local Transform
            x_l = rot[e[1]].T @ np.array(self.bone[e[1]]['direction'], dtype = float)
            direction = P[e[0]] @ rot[e[1]] @ M[e[1]] @ x_l * self.bone[e[1]]['length']
            pts[e[1]] = direction + pts[e[0]]
            P[e[1]] = P[e[0]] @ rot[e[1]] @ M[e[1]] @ rot[e[1]].T
        return pts

    def draw_motion_from_pts(self, pts):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = '3d')
        for e in self.edge:
            ax.plot([pts[e[0]][0], pts[e[1]][0]], [pts[e[0]][1], pts[e[1]][1]], [pts[e[0]][2], pts[e[1]][2]], 'y-')
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        ax.set_zlim(-20, 20)
        plt.show()
        

    def draw_motion(self, rot, M):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = '3d')
        pts = {0: np.array([0, 0, 0])}

        one_rot = np.zeros((3, 3))
        for i in range(3):
            one_rot[i, i] = 1
        P = {0: one_rot}
        for e in self.edge:
            #rot[e[1]] = rot[e[0]] @ rot[e[1]]
            x_l = rot[e[1]].T @ np.array(self.bone[e[1]]['direction'], dtype = float)
            direction = P[e[0]] @ rot[e[1]] @ M[e[1]] @ x_l * self.bone[e[1]]['length']
            if e[1] == 20:
                r = P[e[0]] @ rot[e[1]]
                zero = pts[e[0]]
                x = np.array([0, 0, 3])
                x = r @ x.T
                ax.plot([zero[0], zero[0] + x[0]], [zero[1], zero[1] + x[1]], [zero[2], zero[2] + x[2]], 'b-')
                
                x = np.array([0, 3, 0])
                x = r @ x.T
                ax.plot([zero[0], zero[0] + x[0]], [zero[1], zero[1] + x[1]], [zero[2], zero[2] + x[2]], 'g-')
                
                x = np.array([3, 0, 0])
                x = r @ x.T
                ax.plot([zero[0], zero[0] + x[0]], [zero[1], zero[1] + x[1]], [zero[2], zero[2] + x[2]], 'r-')

                print(M[e[1]])
                print(r.T @ np.array(self.bone[e[1]]['direction'], dtype = float))
            pts[e[1]] = direction + pts[e[0]]
            P[e[1]] = P[e[0]] @ rot[e[1]] @ M[e[1]] @ rot[e[1]].T
            ax.plot([pts[e[0]][0], pts[e[1]][0]], [pts[e[0]][1], pts[e[1]][1]], [pts[e[0]][2], pts[e[1]][2]], 'y-')
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        ax.set_zlim(-20, 20)
        plt.show()

    def show(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = '3d')
        pts = {}
        pts[0] = np.array([0, 0, 0])
        for e in self.edge:
            pts[e[1]] = np.array(self.bone[e[1]]['direction'], dtype=np.float)*self.bone[e[1]]['length'] + pts[e[0]]
            ax.plot([pts[e[0]][0], pts[e[1]][0]], [pts[e[0]][1], pts[e[1]][1]], [pts[e[0]][2], pts[e[1]][2]], 'b-')
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        ax.set_zlim(-20, 20)
        plt.show()

                            
class Motion:
    def __init__(self, robot, filename):
        self.robot = robot

        f = open(filename, 'r')
        for _ in range(3):
            f.readline()
        need_read = True
        self.motion_list = []

        one_rot = np.zeros((3, 3))
        for i in range(3):
            one_rot[i, i] = 1

        k = 0
        while True:
            rot = {}
            M = {}
            pts0 = np.zeros(3)
            rot0 = np.zeros((3, 3))
            for i in range(30):
                rot[i] = one_rot
                M[i] = one_rot
        
            k += 1
            if need_read:
                l = f.readline()
            print(l)
            need_read = True
            if not l:
                break
            if l[0] >= '0' and l[0] <= '9':
                while True:
                    l = f.readline()
                    if not l:
                        return
                    if l[0] >= '0' and l[0] <= '9':
                        need_read = False
                        break
                    l = l.strip().split()
                    if l[0].count('root') > 0:
                        pts0 = np.array([l[1], l[2], l[3]], dtype = np.float)
                        bone = {}
                        bone['axis'] = [l[4], l[5], l[6]]
                        rot0 = robot.bone_transform(bone)
                        # pts0 = np.array([0, 0, 0], dtype = np.float)
                        continue
                    else:
                        id = robot.name_to_id[l[0]]
                        rot[id] = robot.bone_transform(robot.bone[id])
                        bone = {'axis': [0, 0, 0]}
                        for i in range(1, len(l)):
                            if robot.bone[id]['dof'][i-1] == 'rx':
                                bone['axis'][0] = l[i]
                            if robot.bone[id]['dof'][i-1] == 'ry':
                                bone['axis'][1] = l[i]
                            if robot.bone[id]['dof'][i-1] == 'rz':
                                bone['axis'][2] = l[i]
                        M[id] = robot.bone_transform(bone)
                        continue
            self.motion_list.append(robot.cal_motion(rot, M, pts0, rot0))
            #robot.draw_motion_from_pts(self.motion_list[k-1])
    
    def draw_motion_anime(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection = '3d')
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.ax.set_zlim(-30, 30)
        print(self.ax)
        pts = self.motion_list[0]
        self.pts_plot = {}
        for e in self.robot.edge:
            self.pts_plot[(e[0], e[1])],  = self.ax.plot([pts[e[0]][0], pts[e[1]][0]], [pts[e[0]][1], pts[e[1]][1]], [pts[e[0]][2], pts[e[1]][2]], 'y-')
        ani = animation.FuncAnimation(self.fig, self.motion_update, len(self.motion_list), interval=1)
        plt.show()

    def motion_update(self, i):
        pts = self.motion_list[i]
        
        for e in self.robot.edge:
            #print(self.pts_plot[(e[0], e[1])])
            # self.pts_plot[(e[0], e[1])],  = self.ax.plot([pts[e[0]][0], pts[e[1]][0]], [pts[e[0]][1], pts[e[1]][1]], [pts[e[0]][2], pts[e[1]][2]], 'y-')
            (self.pts_plot[(e[0], e[1])]).set_data([pts[e[0]][0], pts[e[1]][0]], [pts[e[0]][1], pts[e[1]][1]])
            (self.pts_plot[(e[0], e[1])]).set_3d_properties([pts[e[0]][2], pts[e[1]][2]])
        return self.pts_plot

    def motion_save(self, file_name, bone_name):
        data = []
        for m in self.motion_list:
            id = self.robot.name_to_id[bone_name]
            data.append(m[id])
        data = np.array(data)
        np.save(file_name, data)

    
    def draw_motion_xyz(self, bone_name):
        data = []
        for m in self.motion_list:
            id = self.robot.name_to_id[bone_name]
            data.append(m[id])
        data = np.array(data)
        # plt.plot(range(data.shape[0]), data[:, 0], 'r-')
        # plt.show()
        # plt.plot(range(data.shape[0]), data[:, 1], 'g-')
        # plt.show()
        # plt.plot(range(data.shape[0]), data[:, 2], 'b-')
        # plt.show()
        # v = (data[data.shape[0]-1, 2] - data[0, 2]) / data.shape[0]
        # print(v)
        for _ in range(1, data.shape[0]):
            data[_-1, 2] = data[_, 2] - data[_-1, 2]
        fftz = np.fft.fft(data[:-1, 2])
        for i in range(data.shape[0] - 1):
            fftz[i] = fftz[i].imag**2 + fftz[i].real**2
        plt.plot(range(data.shape[0]-1), data[:-1, 2], 'b-')
        plt.show()
        plt.plot(range(data.shape[0]-2), fftz[1:], 'b-')
        plt.show()
                        

def read2save(filename, k, n):
    robot = Robot( (filename % k) + '.asf')
    for i in range(1, n+1):
        motion = Motion(robot, (filename % k) + '_%02d.amc' % i)
        motion.motion_save('data/walk/%02d_%02d' % (k, i), 'rhumerus')
        


if __name__ == "__main__":
    read2save('allasfamc/07/%02d', 7, 12)
    # robot = Robot('allasfamc/05/05.asf')
    # robot.show()
    # motion = Motion(robot, 'allasfamc/05/05_10.amc')
    # motion.draw_motion_xyz('root')
    # motion.draw_motion_anime()
