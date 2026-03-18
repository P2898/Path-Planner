"""
Interactive Multi-Agent 2D RRT Planner with Adaptive Replanning, Spline Smoothing and Animation

Single-file pygame application.

Features added on top of original:
 - Multi-agent support (up to 3 agents). Each agent has its own start/goal, tree and path.
 - Animated robots that follow planned paths with smooth heading and constant speed.
 - Path smoothing using Catmull-Rom spline interpolation.
 - Dynamic obstacle addition while planner/robots are running.
 - Adaptive local repair: when a new obstacle invalidates a path, attempt a local RRT repair between the first and last colliding waypoints; if it fails, fall back to full replan for that agent.
 - Simple inter-agent soft collision avoidance while following path (local repulsive velocity adjustment).
 - UI improvements for placing/selecting agents and controls.

Controls:
 - Left mouse drag: draw obstacle polygon (hold LMB to paint; release to commit polygon).
 - 'o' : toggle obstacle-draw mode (when off, clicks set start/goal for selected agent).
 - Number keys '1','2','3': select agent to edit (create agents by assigning start/goal).
 - 's' then click: set Start for selected agent.
 - 'g' then click: set Goal for selected agent.
 - SPACE : start / pause planners and animations (runs all agents concurrently).
 - 'r' : replan all agents (clears trees and restarts planning for agents with start+goal).
 - 'p' : perform path smoothing for all found paths.
 - 'c' : clear obstacles (stops planners).
 - ENTER : finalize polygon while drawing
 - 'q' or ESC: quit

Run:
    pip install pygame numpy
    

Notes & limitations:
 - Agents are point-robots with soft avoidance; no kinodynamic constraints are considered.
 - Local repair is heuristic: it attempts to repair between colliding waypoint indices using a temporary RRT limited to that subproblem.
 - Catmull-Rom smoothing produces continuous C1 paths good for visualization; curvature is not explicitly limited.

"""

from turtle import width, window_height
import pygame
import sys
import math
import random
import time
import numpy as np

#Utilities 

def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def clamp(a, low, high):
    return max(low, min(high, a))


# Line intersection for segments p1--p2 and q1--q2
def seg_intersect(p1, p2, q1, q2):
    (x1,y1),(x2,y2)=p1,p2
    (x3,y3),(x4,y4)=q1,q2
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
    if abs(denom) < 1e-9:
        return False
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3))/denom
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3))/denom
    return 0<=ua<=1 and 0<=ub<=1


# Point in polygon (ray casting)
def point_in_poly(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1,y1 = poly[i]
        x2,y2 = poly[(i+1)%n]
        if ((y1>y) != (y2>y)) and (x < (x2-x1)*(y-y1)/(y2-y1+1e-12) + x1):
            inside = not inside
    return inside


# Segment intersects polygon if intersects any edge or midpoint inside polygon
def segment_intersects_poly(a, b, poly):
    n = len(poly)
    for i in range(n):
        p = poly[i]
        q = poly[(i+1)%n]
        if seg_intersect(a, b, p, q):
            return True
    # quick check: if midpoint inside polygon, treat as collision
    mx = ((a[0]+b[0])/2.0, (a[1]+b[1])/2.0)
    if point_in_poly(mx, poly):
        return True
    return False


#RRT Data Structures
class Node:
    def __init__(self, pt, parent=None):
        self.pt = pt
        self.parent = parent


class RRTPlanner:
    def __init__(self, start, goal, bounds, obstacles, max_nodes=3000, step_size=25, goal_sample_rate=0.05):
        self.start = Node(start)
        self.goal = Node(goal)
        self.bounds = bounds  # (w,h)
        self.obstacles = obstacles
        self.max_nodes = max_nodes
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.nodes = [self.start]
        self.found = False
        self.path = []

    def nearest(self, pt):
        best = None
        bd = float('inf')
        for n in self.nodes:
            d = dist(n.pt, pt)
            if d < bd:
                bd = d
                best = n
        return best

    def steer(self, from_pt, to_pt):
        d = dist(from_pt, to_pt)
        if d <= self.step_size:
            return to_pt
        else:
            theta = math.atan2(to_pt[1]-from_pt[1], to_pt[0]-from_pt[0])
            return (from_pt[0] + self.step_size*math.cos(theta), from_pt[1] + self.step_size*math.sin(theta))

    def collision_free(self, a, b):
        # check against all obstacles
        for poly in self.obstacles:
            if segment_intersects_poly(a, b, poly):
                return False
        # also ensure inside bounds
        w,h = self.bounds
        if not (0<=a[0]<=w and 0<=a[1]<=h and 0<=b[0]<=w and 0<=b[1]<=h):
            return False
        return True

    def try_extend(self):
        if random.random() < self.goal_sample_rate:
            sample = self.goal.pt
        else:
            sample = (random.uniform(0, self.bounds[0]), random.uniform(0, self.bounds[1]))
        nearest = self.nearest(sample)
        new_pt = self.steer(nearest.pt, sample)
        if self.collision_free(nearest.pt, new_pt):
            new_node = Node(new_pt, nearest)
            self.nodes.append(new_node)
            # check if reaches goal
            if dist(new_pt, self.goal.pt) <= self.step_size and self.collision_free(new_pt, self.goal.pt):
                self.goal.parent = new_node
                self.nodes.append(self.goal)
                self.found = True
                self.extract_path()
                return True
        return False

    def extract_path(self):
        if not self.found:
            self.path = []
            return
        path = []
        n = self.goal
        while n is not None:
            path.append(n.pt)
            n = n.parent
        path.reverse()
        self.path = path

    def reset(self, start=None, goal=None):
        if start is not None:
            self.start = Node(start)
        if goal is not None:
            self.goal = Node(goal)
        self.nodes = [self.start]
        self.found = False
        self.path = []


# Path smoothing using Catmull-Rom splines for continuous interpolation
# Catmull-Rom requires at least 4 points; we duplicate endpoints when needed

def catmull_rom_chain(points, num_points=100):
    if len(points) < 2:
        return points[:]
    if len(points) == 2:
        # straight-line interpolation
        a, b = points
        res = []
        for t in np.linspace(0,1,num_points):
            res.append((a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1])))
        return res
    # build chain
    pts = [points[0]] + points + [points[-1]]
    out = []
    steps = max(2, num_points // (len(points)-1))
    for i in range(len(points)-1):
        p0 = np.array(pts[i])
        p1 = np.array(pts[i+1])
        p2 = np.array(pts[i+2])
        p3 = np.array(pts[i+3])
        for t in np.linspace(0,1,steps,endpoint=False):
            t2 = t*t
            t3 = t2*t
            # Catmull-Rom basis
            f1 = -0.5*t3 + t2 - 0.5*t
            f2 =  1.5*t3 - 2.5*t2 + 1.0
            f3 = -1.5*t3 + 2.0*t2 + 0.5*t
            f4 =  0.5*t3 - 0.5*t2
            p = f1*p0 + f2*p1 + f3*p2 + f4*p3
            out.append((float(p[0]), float(p[1])))
    out.append(points[-1])
    return out


#Agent / Multi-Agent logic
class Agent:
    def __init__(self, id, color, bounds, obstacles):
        self.id = id
        self.color = color
        self.bounds = bounds
        self.obstacles = obstacles
        self.start = None
        self.goal = None
        self.planner = None
        self.running = False
        self.path = []
        self.smoothed_path = []
        self.position = None
        self.speed = 80.0  # pixels per second
        self.path_index = 0

    def set_start(self, pt):
        self.start = pt
        self.position = pt
        self.planner = None
        self.path = []
        self.smoothed_path = []
        self.path_index = 0

    def set_goal(self, pt):
        self.goal = pt
        self.planner = None
        self.path = []
        self.smoothed_path = []
        self.path_index = 0

    def init_planner(self):
        if self.start and self.goal:
            self.planner = RRTPlanner(self.start, self.goal, self.bounds, self.obstacles)
            self.running = True

    def step_planner(self, iterations=4):
        if self.planner and not self.planner.found:
            for _ in range(iterations):
                self.planner.try_extend()
            if self.planner.found:
                self.path = self.planner.path[:]
                self.smoothed_path = catmull_rom_chain(self.path, num_points= max(200, len(self.path)*20))
                self.path_index = 0

    def full_replan(self):
        self.init_planner()

    def local_repair(self, obstacle_polys):
        # detect first and last indices where path collides with obstacles
        if not self.path:
            self.full_replan()
            return
        colliding = [i for i in range(len(self.path)-1) if any(segment_intersects_poly(self.path[i], self.path[i+1], poly) for poly in obstacle_polys)]
        if not colliding:
            return  # no repair needed
        i0 = min(colliding)
        i1 = max(colliding)+1
        a = self.path[i0]
        b = self.path[i1]
        # attempt to grow a short RRT between a and b within bounding box
        bx = min(a[0], b[0]) - 60, min(a[1], b[1]) - 60, max(a[0], b[0]) + 60, max(a[1], b[1]) + 60
        # create temporary planner
        tmp_bounds = self.bounds
        temp_planner = RRTPlanner(a, b, tmp_bounds, self.obstacles, max_nodes=1200, step_size=30, goal_sample_rate=0.08)
        success = False
        for _ in range(600):
            if temp_planner.try_extend():
                success = True
                break
        if success:
            # stitch
            new_segment = temp_planner.path[:]
            new_path = self.path[:i0+1] + new_segment[1:-1] + self.path[i1:]
            self.path = new_path
            self.smoothed_path = catmull_rom_chain(self.path, num_points=max(200, len(self.path)*20))
            self.path_index = 0
        else:
            # fallback: full replan
            self.full_replan()

    def update_motion(self, dt, other_agents):
        if not self.smoothed_path:
            return
        # follow smoothed path with simple interpolation
        if self.path_index >= len(self.smoothed_path)-1:
            return
        target = self.smoothed_path[self.path_index+1]
        pos = np.array(self.position)
        tgt = np.array(target)
        vec = tgt - pos
        d = np.linalg.norm(vec)
        if d < 1e-6:
            self.path_index += 1
            return
        # velocity towards target
        v = (vec / d) * self.speed
        # add simple repulsive avoidance from other agents
        avoidance = np.array([0.0, 0.0])
        for other in other_agents:
            if other is self or other.position is None:
                continue
            op = np.array(other.position)
            dd = np.linalg.norm(pos - op)
            if dd < 40 and dd > 1e-6:
                # repulse
                away = (pos - op) / dd
                avoidance += away * (40 - dd) * 3.0
        v = v + avoidance
        step = v * dt
        if np.linalg.norm(step) >= d:
            # reach next
            self.position = tuple(tgt)
            self.path_index += 1
        else:
            self.position = tuple(pos + step)


#Pygame UI & Main

def main():
    pygame.init()
    W, H = 1000, 650
    screen = pygame.display.set_mode((W,H))
    pygame.display.set_caption('Interactive Multi-Agent RRT Planner')
    clock = pygame.time.Clock()

    obstacles = []  # list of polygons (list of points)
    drawing_poly = False
    current_poly = []

    agents = [Agent(1, (0,200,0), (W,H), obstacles),
              Agent(2, (200,0,0), (W,H), obstacles),
              Agent(3, (0,120,200), (W,H), obstacles)]
    selected_agent = 0

    mode_obstacle_draw = True
    awaiting_set = None  # 's' or 'g' to set start/goal on next click

    font = pygame.font.SysFont(None, 20)

    running_rrt = False
    last_time = time.time()

    info_help = [
        "Controls: o toggle draw | 1-3 select agent | s set start | g set goal | SPACE start/pause | r replan | p smooth | c clear obs",
        "Draw polygon: hold left mouse and drag. Press ENTER to finalize poly while drawing."
    ]

    while True:
        dt = clock.get_time() / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    pygame.quit(); sys.exit()
                if event.key == pygame.K_o:
                    mode_obstacle_draw = not mode_obstacle_draw
                if event.key == pygame.K_c:
                    obstacles[:] = []
                    drawing_poly = False
                    current_poly = []
                    for a in agents:
                        a.planner = None
                        a.path = []
                        a.smoothed_path = []
                if event.key == pygame.K_s:
                    awaiting_set = 's'
                if event.key == pygame.K_g:
                    awaiting_set = 'g'
                if event.key == pygame.K_SPACE:
                    # start/pause
                    if not running_rrt:
                        # initialize planners for agents that have start+goal
                        for a in agents:
                            a.obstacles = obstacles
                            a.init_planner()
                        running_rrt = True
                    else:
                        running_rrt = False
                if event.key == pygame.K_r:
                    # replan all agents
                    for a in agents:
                        a.obstacles = obstacles
                        a.full_replan()
                    running_rrt = True
                if event.key == pygame.K_p:
                    # smooth existing paths
                    for a in agents:
                        if a.path:
                            a.smoothed_path = catmull_rom_chain(a.path, num_points=max(200, len(a.path)*20))
                            a.path_index = 0
                if event.key == pygame.K_RETURN:
                    if drawing_poly and len(current_poly)>=3:
                        obstacles.append(current_poly[:])
                        current_poly = []
                        drawing_poly = False
                if event.key in (pygame.K_1, pygame.K_2, pygame.K_3):
                    idx = {pygame.K_1:0, pygame.K_2:1, pygame.K_3:2}[event.key]
                    selected_agent = idx
                if event.key == pygame.K_TAB:
                    # switch agent
                    selected_agent = (selected_agent + 1) % len(agents)

            if event.type == pygame.MOUSEBUTTONDOWN:
                mx,my = event.pos
                if awaiting_set == 's':
                    agents[selected_agent].set_start((mx,my))
                    awaiting_set = None
                elif awaiting_set == 'g':
                    agents[selected_agent].set_goal((mx,my))
                    awaiting_set = None
                else:
                    if mode_obstacle_draw:
                        if event.button == 1:
                            drawing_poly = True
                            current_poly = [event.pos]
                    else:
                        if event.button == 1:
                            # quick set: if no start set it, else set goal
                            a = agents[selected_agent]
                            if a.start is None:
                                a.set_start(event.pos)
                            elif a.goal is None:
                                a.set_goal(event.pos)

            if event.type == pygame.MOUSEMOTION:
                if drawing_poly and mode_obstacle_draw:
                    current_poly.append(event.pos)

            if event.type == pygame.MOUSEBUTTONUP:
                if drawing_poly and mode_obstacle_draw and event.button == 1:
                    if len(current_poly) >= 3:
                        if len(current_poly) > 200:
                            step = len(current_poly)//200
                            simplified = [current_poly[i] for i in range(0, len(current_poly), step)]
                        else:
                            simplified = current_poly[:]
                        obstacles.append(simplified)
                        # when obstacle added dynamically, attempt local repair for agents
                        for a in agents:
                            a.obstacles = obstacles
                            if a.path:
                                a.local_repair([obstacles[-1]])
                    current_poly = []
                    drawing_poly = False

        # RRT stepping for all agents
        if running_rrt:
            for a in agents:
                a.obstacles = obstacles
                if a.planner and not a.planner.found:
                    a.step_planner(iterations=6)
                # if planner finished earlier, ensure paths set
                if a.planner and a.planner.found and not a.path:
                    a.path = a.planner.path[:]
                    a.smoothed_path = catmull_rom_chain(a.path, num_points=max(200, len(a.path)*20))
                    a.path_index = 0

        # update agent motions
        for a in agents:
            if a.position is None and a.start:
                a.position = a.start
            if running_rrt:
                a.update_motion(dt, agents)

        # draw
        screen.fill((20,20,20))

        # draw obstacles
        for poly in obstacles:
            if len(poly) >= 3:
                pygame.draw.polygon(screen, (180,60,60), poly)
                pygame.draw.polygon(screen, (255,100,100), poly, 2)
        if drawing_poly and len(current_poly) >= 2:
            pygame.draw.lines(screen, (255,120,120), False, current_poly, 3)

        # draw planners' trees (thin)
        for a in agents:
            if a.planner:
                for n in a.planner.nodes:
                    if n.parent:
                        pygame.draw.aaline(screen, (120,120,120), n.pt, n.parent.pt)

        # draw paths and smoothed paths
        for a in agents:
            if a.path:
                pygame.draw.lines(screen, a.color, False, a.path, 3)
            if a.smoothed_path:
                # draw as finer line
                pygame.draw.lines(screen, tuple(min(255,x+60) for x in a.color), False, a.smoothed_path, 3)

        # draw agents (robots)
        for idx,a in enumerate(agents):
            if a.position:
                pygame.draw.circle(screen, a.color, (int(a.position[0]), int(a.position[1])), 8)
                # heading indicator: point towards next path point
                if a.path_index < len(a.smoothed_path)-1:
                    nxt = a.smoothed_path[min(len(a.smoothed_path)-1, a.path_index+1)]
                    theta = math.atan2(nxt[1]-a.position[1], nxt[0]-a.position[0])
                    hx = a.position[0] + 14*math.cos(theta)
                    hy = a.position[1] + 14*math.sin(theta)
                    pygame.draw.line(screen, (230,230,230), a.position, (hx,hy), 2)
            # draw start/goal markers
            if a.start:
                pygame.draw.circle(screen, (0,200,0), (int(a.start[0]), int(a.start[1])), 6)
            if a.goal:
                pygame.draw.circle(screen, (200,0,0), (int(a.goal[0]), int(a.goal[1])), 6)
            # selection highlight
            if idx == selected_agent:
                pygame.draw.rect(screen, (255,255,0), (10 + idx*60, 10, 56, 26), 2)

        # HUD / text
        y = 6
        for line in info_help:
            surf = font.render(line, True, (220,220,220))
            screen.blit(surf, (6,y))
            y += 18

        # agent statuses
        st = ''
        for i,a in enumerate(agents):
            st += f"A{i+1}: start={'Y' if a.start else 'N'} goal={'Y' if a.goal else 'N'} nodes={len(a.planner.nodes) if a.planner else 0} found={a.planner.found if a.planner else False}  "
        screen.blit(font.render(st, True, (220,220,220)), (6, H-22))

        pygame.display.flip()
        clock.tick(60)


if __name__ == '__main__':
    main()
