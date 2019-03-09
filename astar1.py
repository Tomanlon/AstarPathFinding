"""
	Author: Tomanlon

	Copyright: Tomanlon

	Date: 2019-3-9

	Description: Simple demo for displaying astar algorithm with path find

	MailBox: taomanl@163.com
"""

import matplotlib.pyplot as plt
import math

#define node
class Node:
	def __init__(self,x,y,parent,cost,index):
		self.x = x
		self.y = y
		#used for tracking path node in closeset
		self.parent = parent
		self.cost = cost
		# node index
		self.index = index

# tracking path node
def calc_path(goaln,closeset):
	rx,ry=[goaln.x],[goaln.y]
	print closeset[-1]
	#goal parent node in closeset
	parentn = closeset[-1]

	while parentn!=None:
		rx.append(parentn.x)
		ry.append(parentn.y)
		parentn = parentn.parent
	return rx,ry


# astar algorithm core
def astar_plan(sx,sy,gx,gy):
	ox,oy,xwidth,ywidth=map_generation()
	# show map
	plt.figure('Astar algorithm demo')
	plt.plot(ox,oy,'ks')
	plt.plot(sx,sy,'bs')
	plt.plot(gx,gy,'ro')
	# define motion model
	motion = motion_model()
	# define open and close set for saving node
	openset,closeset=dict(),dict()
	sidx = sy*xwidth+sx
	gidx = gy*xwidth+gx
	# start node and goal node
	starn=Node(sx,sy,None,0,sidx)
	goaln=Node(gx,gy,None,0,gidx)
	openset[sidx] = starn
	while 1:
		# calculate node with min f_cost as current node
		c_id = min(openset,key=lambda o:openset[o].cost + h_cost(openset[o],goaln))
		curnode = openset[c_id]
		# if arrive goal point break else closset add current node and remove current node from openset
		if curnode.x == goaln.x and curnode.y == goaln.y:
			print 'find goal'
			closeset[-1] = curnode
			break
		else:
			closeset[c_id] = curnode	
			plt.plot(curnode.x,curnode.y,'gx')
			if len(openset.keys())%10==0:
				plt.pause(0.01)
			del openset[c_id]
		
		# check 8 direction point
		for j in range(len(motion)):
			newnode = Node(curnode.x+motion[j][0],
						   curnode.y+motion[j][1],
						   curnode,
						   curnode.cost + motion[j][2],
						   c_id)
			n_id = index_calc(newnode,xwidth)
			
			# if node in closeset out of loop once
			if n_id in closeset:
				continue
			
			# if node in obstacle out of loop once
			if node_verify(newnode,ox,oy):
				continue
			
			# if node not in openset add it to openset
			#  else compare it with old one and update it with lower g_cost and update parent also
			if n_id not in openset:
				openset[n_id] = newnode
			else:
				if openset[n_id].cost >= newnode.cost:
					openset[n_id] = newnode
	# get path node x ,y 
	px,py = calc_path(goaln,closeset)
	return px,py

# Map generation
def map_generation():
	# ox,oy list for obstacles
	ox,oy=[],[]
	for i in range(60):
		ox.append(i)
		oy.append(0)

	for i in range(60):
		ox.append(i)
		oy.append(60)

	for i in range(60):
		ox.append(0)
		oy.append(i)

	for i in range(60):
		ox.append(60)
		oy.append(i)
	
	for i in range(25):
		ox.append(i)
		oy.append(20)

	for i in range(40):
		ox.append(35)
		oy.append(i)

	for i in range(40):
		ox.append(50)
		oy.append(60-i)

	minx = min(ox)
	miny = min(oy)
	maxx = max(ox)
	maxy = max(oy)
	# map  xwidth,  ywidth
	xwidth = maxx-minx
	ywidth = maxy-miny
	
	return ox,oy,xwidth,ywidth
	

	
# motion model in 8 direction with 8 g_costs
def motion_model():
	motion =[[1,0,1],
			 [1,1,math.sqrt(2)],
			 [1,-1,math.sqrt(2)],
			 [0,1,1],
			 [0,-1,1],
			 [-1,1,math.sqrt(2)],
			 [-1,0,1],
			 [-1,-1,math.sqrt(2)]]
	return motion

def h_cost(node,goal):
	# Weight
	w = 1.0
	# Euclidean distance be careful: this distance will cost longer time
	#h = w*math.sqrt((goal.x-node.x)**2 + (goal.y-node.y)**2)
	# Mahattan distance
	h = w*(abs(goal.x-node.x) + abs(goal.y-node.y))
	return h

# Node index obtain
def index_calc(node,xwid):
	n_id = node.y*xwid + node.x
	return n_id

# Verify node is in obstacle or not
def node_verify(node,ox,oy):
	if (node.x,node.y) in zip(ox,oy):
		return True
	else:
		return False

def main():
	# Define  start point(sx,sy),goal point(gx,gy)
	sx,sy=15,15
	gx,gy=55,50
	# Astar algorithm tracking path node
	rx,ry=astar_plan(sx,sy,gx,gy)
	print rx,ry
	# show path
	plt.plot(rx,ry,'r-',linewidth=3)
	plt.show()

if __name__ =="__main__":
	main()
