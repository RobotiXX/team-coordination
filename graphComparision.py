
from normalJSG import JSGraph
from criticalJSG import CJSGraph
import graphPlot
import time
######## Environment Graph #####################
###### (A) 3 nodes, 2 edges with 1 risky edge with 2 support nodes 

# V = 3
# S11 = (0,0) # start: 0 node
# Sgg = (2,2) # goal: 2 node
# adj_ws = [[0,[10,(0,2)],float('inf')], # 3 nodes 2 edges 
#                [[10,(0,2)],0,[10,()]],
#                [float('inf'),[10,()],0]]
# adj_ns = [[0,10,0],
#                 [10,0,10],
#                 [0,10,0]]    
####### (B) 4 nodes, 3 edges with 1 risky edge each with 2 support nodes 

# V = 4
# S11 = (0,0) # start: 0 node
# Sgg = (3,3) # goal: 3 node
# adj_ws = [[0,[10,(0,2)],float('inf'),float('inf')], # 4 nodes 3 edges 
#         [[10,(0,2)],0,[10,()],float('inf')],
#         [float('inf'),[10,()],0, [10,()]],
#         [float('inf'),float('inf'),[10,()],0]]
# adj_ns = [[0,10,0,0],
#         [10,0,10,0],
#         [0,10,0,10],
#         [0,0,10,0]]
###### (C) 6 node, 6 edges with 3 risky edge  each with 2 support nodes

V = 6
S11 = (0,0) # start: 0 node
Sgg = (5,5) # goal: 5 node
adj_ws = [[0, [10, (0, 2)], float('inf'), float('inf'), float('inf'), float('inf')],
    [[10, (0, 2)], 0, [10, ()], [10, ()], float('inf'), float('inf')],
    [float('inf'), [10, ()], 0, float('inf'), [10, (2, 5)], float('inf')],
    [float('inf'), [10, ()], float('inf'), 0, [10, (3, 5)], float('inf')],
    [float('inf'), float('inf'), [10, (2, 5)], [10, (3, 5)], 0, [10, ()]],
    [float('inf'), float('inf'), float('inf'), float('inf'), [10, ()], 0]]
adj_ns = [[0, 10, 0, 0, 0, 0],
           [10, 0, 10, 10, 0, 0],
           [0, 10, 0, 0, 10, 0],
           [0, 10, 0, 0, 10, 0],
           [0, 0, 10, 10, 0, 10],
           [0, 0, 0, 0, 10, 0]]
###### (D) 7 nodes 6 edges with 3 risky edges each with 2 support nodes

# V = 7
# S11 = (0,0) # start: 0 node
# Sgg = (6,6) # goal: 6 node
# adj_ws = [[0, [10, (0, 2)], float('inf'), float('inf'), float('inf'), float('inf'), float('inf')],
# [[10, (0, 2)], 0, [10, ()], float('inf'), float('inf'), float('inf'), float('inf')],
# [float('inf'), [10, ()], 0, [10, (2, 4)], float('inf'),float('inf'), float('inf')],
# [float('inf'), float('inf'), [10, (2, 4)], 0, [10, ()], float('inf'), [10, ()]],
# [float('inf'), float('inf'), float('inf'), [10, ()], 0, [10, (3, 5)], float('inf')],
# [float('inf'), float('inf'), float('inf'), float('inf'), [10, (3, 5)], 0, float('inf')],
# [float('inf'), float('inf'), float('inf'), [10, ()], float('inf'), float('inf'), 0]]
# adj_ns = [[0,10,0,0,0,0,0],
#            [10,0,10,0,0,0,0],
#            [0,10,0,10,0,0,0],
#            [0,0,10,0,10,0,10],
#            [0,0,0,10,0,10,0],
#            [0,0,0,0,10,0,0],
#            [0,0,0,10,0,0,0]]

############ Joint State Graph (JSG) #################
############# (1) Generate JSG from EG
start_jsg = time.time()
jsg = JSGraph(V)
jsg.source = S11
jsg.destination = Sgg
jsg.graph_1d = adj_ws
jsg.transform_EnvToJointStateGraph()
jsg.printGraphAjList_2d()
jsg.shortest_path_cost()
end_jsg = time.time()



########### Crtical Joint State Graph (CJSG) ######## 
########### (1) Generate CJSG from EG
start_cjsg = time.time()
cjsg = CJSGraph(V)
cjsg.adj = adj_ws
cjsg.adj_ns = adj_ns
cjsg.S11 = S11
cjsg.Sgg = Sgg
cjsg.transform_EG_to_CJSG()
cjsg.printEdgeSetsAndCostCJSG()
cjsg.shortest_path_cost()
end_cjsg = time.time()

########### Time Comparision JSG and CJSG ##########
########### (1) Compare the time taken by JSG and CJSG for same EG

print("Time taken by JSG: {}".format(end_jsg - start_jsg))
print("Time taken by CJSG: {}".format(end_cjsg - start_cjsg))

########### Plot all Graphs #########################
########## (1) Plot Environment Graph ################
#graphPlot.plot_environment_graph(E=jsg.getEnvironmentGraphEdges())

########### (2) Plot the JSG: Joint State Graph
#graphPlot.plot_joint_state_space_graphTT(S=S11, D=Sgg,E=jsg.getJointStateGraphEdges())
    
########### (3) Plot the CJSG: Critical Joint State Graph
#graphPlot.plot_joint_state_space_graphTT(S=S11, D=Sgg,E=cjsg.getCJSG_Edges())
########### (4) Plot the Comparsion between JSG and CJSG
# TODO