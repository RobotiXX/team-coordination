from collections import defaultdict
import sys
import graphPlot
class JSGraph():
    def __init__(self, V):
        '''
        Instantiate the Graph with the number of vertices of the graph.
        '''
        self.V = V # vertices or nodes
        self.graph_1d = [[0 for column in range(V)] for row in
                      range(V)] # used adjM for environment graph
        #self.graph = defaultdict(list) # used adjList
        self.graph_2d = defaultdict(list)# used adjList to transform environment graph to state space graph
        self.source = (None, None)
        self.destination = (None, None)
        self.iter = V*V
        self.environment_graph_edges = []
        self.joint_state_graph_edges = []
        
    def printGraphAjMatrix_1d(self):
        print("\nEnvironment Graph as Adjacency Matrix 1D")
        #print(self.graph)
        for row in self.graph_1d:
            print(row)
        
    def printGraphAjList_2d(self):
        print("\nJoint State Space Graph as Adjacency List 2D")
        #print(self.graph)
        for row in sorted(self.graph_2d):
            print(row, self.graph_2d[row])
        
    def printSolution(self, dist_2d):
        print("Source\tAgent_Position\tDistance_from_Source")
        for u in range(self.V):   
            for v in range(self.V):
                print((self.source[0],self.source[1]),(u,v), dist_2d[u][v]) 
    # get source and destination for joint state space graph
    def get_source_JSG(self):
        return self.source   
             
    def get_destination_JSG(self): 
        return self.destination           
    # get environment graph edges 
    def getEnvironmentGraphEdges(self):  
        print("Environment Graph Edges")          
        for u in range(self.V):
            for v in range(self.V): 
                if self.graph_1d[u][v]!=float('inf') and type(self.graph_1d[u][v])==list and u!=v:
                    print(u,v,self.graph_1d[u][v][0])
                    self.environment_graph_edges.append((str(u),str(v),self.graph_1d[u][v][0]))
                if self.graph_1d[u][v]!=float('inf') and type(self.graph_1d[u][v])==int and u!=v:
                    print(u,v,self.graph_1d[u][v]) 
                    self.environment_graph_edges.append((str(u),str(v),self.graph_1d[u][v]))     
        return self.environment_graph_edges    
    # get joint state space graph edges
    def getJointStateGraphEdges(self):
        print("\nJoint State Space Graph Edges")
        for u in range(self.V):
            for v in range(self.V):
                for weight, nxt_node in sorted(self.graph_2d[(u,v)]): 
                    print((u,v),(nxt_node), weight) 
                    if ((u,v),(nxt_node), weight) not in self.joint_state_graph_edges:
                        self.joint_state_graph_edges.append((str((u,v)),str((nxt_node))))
        return self.joint_state_graph_edges
    
    
    
    # adj matrix of env graph (adj List alternative)
    def addEdge_1d(self, u, v, w):
        '''
        The Graph will be bidirectional and assumed to have positive weights.
        '''
        
        self.graph_1d[u][v]=w
        self.graph_1d[v][u]=w

    # add adj list of joint state graph
    def addEdge_2d(self, u, v,x,y, w):
        '''
        The Graph will be bidirectional and assumed to have positive weights.
        '''
        if [w,(x,y)] not in self.graph_2d[(u,v)]:
            self.graph_2d[(u,v)].append([w,(x,y)])
        if [w,(u,v)] not in self.graph_2d[(x,y)]:
            self.graph_2d[(x,y)].append([w,(u,v)])
    # environment graph to joint state space conversion
    def transform_EnvToJointStateGraph(self):
        ## Option1 : with risk edges and support nodes
        # self.graph_1d = [[0,[10,(0,2)],float('inf')], # 3 nodes 2 edges 
        #        [[10,(0,2)],0,[10,()]],
        #        [float('inf'),[10,()],0]]
        
        ## Option 2: without risk edges and support nodes
        # self.graph_1d = [[0,10,10], # 3 nodes 3 edges(0-1-2-0)
        #                 [10,0,10],
        #                 [10,10,0]]
        # self.graph_1d = [[0,10,float('inf')], # 3 nodes 2 edges (0-1-2)
        #                 [10,0,10],
        #                 [float('inf'),10,0]]
        # self.graph_1d = [[0,10,float('inf'),float('inf')], # 4 nodes 3 edges (1-3 node connection) 
        #                 [10,0,10,10],
        #                 [float('inf'),10,0,float('inf')],
        #                 [float('inf'),10,float('inf'),0]]
        # adj = [[0,1,float('inf'),float('inf'),float('inf')], # 5 nodes 4 edges (1-0,1-2,1-3,1-4)
        #        [1,0,1,1,1],
        #        [0,1,0,float('inf'),float('inf')],
        #        [0,1,float('inf'),0,float('inf')],
        #        [0,1,float('inf'),float('inf'),0]]
        # 7 nodes with multiple connnection and edges (0-1-2-3-4,3-5,3-6,4-5)
        # adj = [[0,1,float('inf'),float('inf'),float('inf'),float('inf'),float('inf')],
        #        [1,0,1,float('inf'),float('inf'),float('inf'),float('inf')],
        #        [float('inf'),1,0,1,float('inf'),float('inf'),float('inf')],
        #        [float('inf'),float('inf'),1,0,1,float('inf'),1],
        #        [float('inf'),float('inf'),float('inf'),1,0,1,float('inf')],
        #        [float('inf'),float('inf'),float('inf'),float('inf'),1,0,float('inf')],
        #        [float('inf'),float('inf'),float('inf'),1,float('inf'),float('inf'),0]]
        fringe = []
        visited = set() 
        fringe.append((0,0))
        index = 0
        #test = 1
        risk_edges = []
        safe_edges = []
        while len(fringe)>0:
            print("\n")
        #while test>0 :   
            #test = test -1 
            # current agent positions
            (u,v) = fringe[index] 
            # update visited agent positions
            visited.add((u,v))
            print("List of remaining fringe:",fringe)
            print("List of visited fringe:",visited)
            print("\n")
            # print("List of safe edges:",safe_edges)
            # print("List of risky edges:",risk_edges)
            # print("\n")
            print("Finding neigbours combinations for",(u,v)) 
            
            # find the possible neighbour combinations for agents (A,B)->(x,y) eg (0,0)
            list_A = []
            list_B = []
            list_Neighbours = []
            # for i in range(len(self.graph_1d)):
            # for j in range(len(self.graph_1d))
            for i in range(self.V):
                    for j in range(self.V):
                        print(i,j)
                        if self.graph_1d[u][j]!=float('inf') and i==u:
                           # print(j)
                            list_A.append(j)
                        if self.graph_1d[v][j]!=float('inf') and i==v:
                            #print(j)
                            list_B.append(j)
            print("\nPossible Neigbours Combinations")
            for i in range(len(list_A)):
                for j in range(len(list_B)):
                    print(list_A[i],list_B[j])
                    if (list_A[i],list_B[j]) not in [(u,v)]:
                        list_Neighbours.append((list_A[i],list_B[j]))
            print("\nCalculate distance between Negigbours and Update AdjList")
            # find the egde cost between neighbour joint state space from (u,v) to (x,y)
            # edge cost depends upon risky edge determined by presence support nodes
            # and update in adjacency matrix of the joint state space graph
            # total distance = aggregate distance from u to x and v to y 
            # and the existence of support nodes indicate either risky or safe path
            for (x,y) in list_Neighbours:
                #print((u,v),(x,y))
                # A constant B moving
                if u==x and v!=y:
                    print((u,v),(x,y))
                    dist = 0
                    support_node_vy = []
                    dist_ux = self.graph_1d[u][x] 
                
                    if type(self.graph_1d[v][y])==list:
                        dist_vy,support_node_vy = self.graph_1d[v][y]
                    else:
                        dist_vy = self.graph_1d[v][y] 
                    dist = dist_ux+dist_vy
                    #print((u,v),(x,y),dist,support_node_vy)     
                    if v in support_node_vy and u==v:
                        dist=dist/2
                        print("A constant, B moving: using support nodes {}".format(support_node_vy))
                        print((u,v),(x,y),dist,support_node_vy) 
                    elif x in support_node_vy and u!=v:
                        dist=dist/2
                        print("A constant, B moving: using support nodes {}".format(support_node_vy))
                        print((u,v),(x,y),dist,support_node_vy) 
                    else:
                        print("A constant, B moving: NORMALLY" )
                        print((u,v),(x,y),dist,support_node_vy)
                    ############################    
                    self.addEdge_2d(u,v,x,y,dist)
                    if (x,y) not in fringe and (x,y) not in visited:
                        fringe.append((x,y))  
                    print("---------------------------------------")
                # B constant, A moving
                elif u!=x and v==y:
                    #print((u,v),(x,y))
                    dist = 0
                    support_node_ux = []
                    if type(self.graph_1d[u][x])==list:
                        dist_ux,support_node_ux= self.graph_1d[u][x]
                    else:
                        dist_ux = self.graph_1d[u][x] 
                    dist_vy = self.graph_1d[v][y] 
                    dist = dist_ux+dist_vy
                    #print((u,v),(x,y),dist,support_node_ux)     
                    if u in support_node_ux and u==v:
                        dist=dist/2
                        print("B constant, A moving: using support nodes {}".format(support_node_ux))
                        print((u,v),(x,y),dist,support_node_ux)
                    elif y in support_node_ux and u!=v:
                        dist=dist/2
                        print("B constant, A moving: using support nodes {}".format(support_node_ux))
                        print((u,v),(x,y),dist,support_node_ux) 
                    else:
                        print("B constant, A moving: NORMALLY" )
                        print((u,v),(x,y),dist,support_node_ux)
                    ###############################    
                    self.addEdge_2d(u,v,x,y,dist)
                    if (x,y) not in fringe and (x,y) not in visited:
                        fringe.append((x,y))      
                    print("---------------------------------------")
                # Both moving: from same node to next same node or from different node to same node/vice versa or exchange position
                # or move from different node to different node
                elif u==v and x==y or u!=v and x==y or u==v and x!=y or u==y and v==x or u!=v and x!=y:
                    # both moving in next node, no cost reduction
                    #print((u,v),(x,y))
                    dist = 0 
                    support_node_ux = ()
                    support_node_vy = ()
                    
                    if type(self.graph_1d[u][x])==list:
                        dist_ux,support_node_ux = self.graph_1d[u][x]
                    else:
                        dist_ux = self.graph_1d[u][x]   
                    if type(self.graph_1d[v][y])==list:
                        dist_vy,support_node_vy = self.graph_1d[v][y]
                    else:
                        dist_vy = self.graph_1d[v][y]    
                    dist = dist_ux+dist_vy
                    print("Both moving: from privious node to next node")
                    print((u,v),(x,y),dist, support_node_ux, support_node_vy)
                    ############################
                    self.addEdge_2d(u,v,x,y,dist)
                    if (x,y) not in fringe and (x,y) not in visited:
                        fringe.append((x,y))  
                    print("---------------------------------------")
            # delete the visited agents positions from the fringe
            fringe.remove((u,v))
        
    def minDistance_2d(self, dist_2d, seen_2d):
        min_d = float('inf')
        for u in range(self.V):
            for v in range(self.V):
                if (u,v) not in seen_2d and dist_2d[u][v] < min_d:
                    min_d = dist_2d[u][v]
                    min_vertex =u,v
        return min_vertex
    
    def shortest_path_cost(self):
        # 2 agents source and destination positions
        dist_2d = [[float('inf') for column in range(self.V)] for row in
                      range(self.V)]
        seen_2d = set()
        dist_2d[self.source[0]][self.source[1]] = 0
        
        for _ in range(self.iter):
            print("\n")
            print("Iteration",_)
            u,v = self.minDistance_2d(dist_2d, seen_2d)
            seen_2d.add((u,v))
            print("seen_node, next_node, edge_cost, dist_from_source")
            #print(self.graph_2d[(u,v)])
            for weight, nxt_node  in sorted(self.graph_2d[(u,v)]):
                #print((u,v), nxt_node, weight)
                x,y = nxt_node 
                if nxt_node not in seen_2d and dist_2d[x][y] > dist_2d[u][v] + weight:
                    #print(">>>>>>>>>>>>>>>>>>>>>>>>")
                    dist_2d[x][y] = dist_2d[u][v] + weight
                    print((u,v),(x,y),weight,dist_2d[x][y])
                    #print(">>>>>>>>>>>>>>>>>>>>>>>>>")
        print("\nList of visted Agent Positions:")
        print(sorted(seen_2d))
        print("\nAdj Matrix for the Distance of Agents from Source")
        #print(dist_2d) in adj matrix form 
        # distance from source to agent position
        print(dist_2d)
        self.printSolution(dist_2d)  
   
           
# Driver's code
# if __name__ == "__main__":
    
    ###################################
    ##### Environment 1: 3 nodes 2 agents
    # N = 3 # no of nodes/vertices of environment graph
    # g = Graph(N) # initialize graph
    # source = 0 # source node
    # destination = 2# destination node
    
    # # # ##### add edges (3-nodes, 2edges, weights and support nodes)
    # g.addEdge_1d(0,1,[10,(0,2)])
    # g.addEdge_1d(1,2,[10,()])
    # g.addEdge_1d(0,2,float('inf'))
    # g.printGraphAjMatrix_1d()
    
    # # ##### transform environment to joint state space graph and print
    # g.transform_EnvToJointStateGraph()
    # g.printGraphAjList_2d()
    
    # # # ##### find shortest path
    # g.shortest_path_cost(source, destination)
    
    ### 1st plot the environment graph
    # graph_edges = g.getEnvironmentGraphEdges()
    # print(graph_edges)
    # graphPlot.plot_environment_graph(E= graph_edges)
    
    #### 2nd plot the joint state space graph
    # joint_state_space_edges = g.getJointStateGraphEdges()
    # print(joint_state_space_edges )
    # src = g.get_source_JSG()
    # dest = g.get_destination_JSG()
    # graphPlot.plot_joint_state_space_graphTT(S=src, D=dest,E=joint_state_space_edges)
    
    ###################################
    ###### Environment 1: 6 nodes 2 agents
    # N = 6 # no of nodes/vertices of environment graph
    # g = JSGraph(N) # initialize graph
    # source = 0 # source node
    # destination = 5# destination node
    
    # #### add edges (6-nodes, 3 edges, weights and support nodes)
    # g.addEdge_1d(0,1,[10,(0,2)])
    # g.addEdge_1d(1,2,[10,()])
    # g.addEdge_1d(1,3,[10,()])
    # g.addEdge_1d(2,4,[10,(2,5)])
    # g.addEdge_1d(3,4,[10,(3,5)])
    # g.addEdge_1d(4,5,[10,()])
    
    # g.addEdge_1d(0,2,float('inf'))
    # g.addEdge_1d(0,3,float('inf'))
    # g.addEdge_1d(0,4,float('inf'))
    # g.addEdge_1d(0,5,float('inf'))

    # g.addEdge_1d(1,4,float('inf'))
    # g.addEdge_1d(1,5,float('inf'))
    
    # g.addEdge_1d(2,0,float('inf'))
    # g.addEdge_1d(2,3,float('inf'))
    # g.addEdge_1d(2,5,float('inf'))
    
    # g.addEdge_1d(3,0,float('inf'))
    # g.addEdge_1d(3,2,float('inf'))
    # g.addEdge_1d(3,5,float('inf'))
    
    # g.addEdge_1d(4,0,float('inf'))
    # g.addEdge_1d(4,1,float('inf'))
    
    # g.addEdge_1d(5,0,float('inf'))
    # g.addEdge_1d(5,1,float('inf'))
    # g.addEdge_1d(5,2,float('inf'))
    # g.addEdge_1d(5,3,float('inf'))
    
    # g.printGraphAjMatrix_1d()
    
    # #### transform environment to joint state space graph and print
    # # g.transform_EnvToJointStateGraph()
    # # g.printGraphAjList_2d()
    
    # ####find shortest path
    # # g.shortest_path_cost(source, destination)
    
    # # #### 1st plot the environment graph
    # # graph_edges = g.getEnvironmentGraphEdges()
    # # print(graph_edges)
    # # graphPlot.plot_environment_graph(E= graph_edges)
    
    # #### 2nd plot the joint state space graph
    # # joint_state_space_edges = g.getJointStateGraphEdges()
    # # print(joint_state_space_edges )
    # # src = g.get_source_JSG()
    # # dest = g.get_destination_JSG()
    # # graphPlot.plot_joint_state_space_graph(S=src, D=dest,E=joint_state_space_edges)
    
    # ###################################
    # # Environment 2: 5 nodes 2 agents
    # # N = 5 # no of nodes/vertices of environment graph
    # # g = Graph(N) # initialize graph
    # # source = 0 # source node
    # # destination = 4# destination node
    
    # ## add edges (5-nodes, 4edges, weights and support nodes)
    # # g.addEdge_1d(0,1,[10,()])
    # # g.addEdge_1d(0,2,[10,(0,3)])
    
    
    # # g.addEdge_1d(1,3,[10,(1,2)])
    # # g.addEdge_1d(1,0,[10,()])
    # # g.addEdge_1d(1,2,[10,()])
    
    # # g.addEdge_1d(2,0,[10,(1,4)])
    # # g.addEdge_1d(2,3,[10,()])
    # # g.addEdge_1d(2,1,[10,()])
    
    
    # # g.addEdge_1d(3,2,[10,()])
    # # g.addEdge_1d(3,1,[10,(1,4)])
    # # g.addEdge_1d(3,4,[10,()])
  
    
    # # g.addEdge_1d(0,3,float('inf'))
    # # g.addEdge_1d(3,0,float('inf'))
    # # g.addEdge_1d(0,4,float('inf'))
    # # g.addEdge_1d(4,0,float('inf'))
    # # g.addEdge_1d(2,4,float('inf'))
    # # g.addEdge_1d(4,2,float('inf'))
    # # g.addEdge_1d(1,4,float('inf'))
    # # g.addEdge_1d(4,1,float('inf'))
    # # g.printGraphAjMatrix_1d() 
    
    # #### transform environment to joint state space graph
    # # g.transform_EnvToJointStateGraph()
    # # g.printGraphAjList_2d()
    
    # # ###### find shor()test path
    # # g.shortest_path_cost(source, destination)
    
    # # #### 1st plot the environment graph
    # # graph_edges = g.getEnvironmentGraphEdges()
    # # print(graph_edges)
    # # graphPlot.plot_environment_graph(E= graph_edges)
    
    # # #### 2nd plot the joint state space graph
    # # joint_state_space_edges = g.getJointStateGraphEdges()
    # # print(joint_state_space_edges )
    # # src = g.get_source_JSG()
    # # dest = g.get_destination_JSG()
    # # graphPlot.plot_joint_state_space_graphTT(S=src, D=dest,E=joint_state_space_edges)
    
    
    
    # ###################################
    # # Environment 3: 7 nodes 2 agents
    # # N = 7 # no of nodes/vertices of environment graph
    # # g = Graph(N) # initialize graph
    # # source = 0 # source node
    # # destination = 5 # destination node, means 6 position
    
    # ### add edges (5-nodes, 4edges, weights and support nodes)
    # # g.addEdge_1d(0,1,[10,(0,2)])
    # # g.addEdge_1d(1,2,[10,()])
    # # g.addEdge_1d(2,3,[10,(2,4)])
    # # g.addEdge_1d(3,4,[10,()])
    # # g.addEdge_1d(3,6,[10,()])
    # # g.addEdge_1d(4,5,[10,(3,5)])
   
    # # g.addEdge_1d(0,2,float('inf'))
    # # g.addEdge_1d(0,3,float('inf'))
    # # g.addEdge_1d(0,4,float('inf'))
    # # g.addEdge_1d(0,5,float('inf'))
    # # g.addEdge_1d(0,6,float('inf'))
    
    # # g.addEdge_1d(1,3,float('inf'))
    # # g.addEdge_1d(1,4,float('inf'))
    # # g.addEdge_1d(1,5,float('inf'))
    # # g.addEdge_1d(1,6,float('inf'))
    
    # # g.addEdge_1d(2,0,float('inf'))
    # # g.addEdge_1d(2,4,float('inf'))
    # # g.addEdge_1d(2,5,float('inf'))
    # # g.addEdge_1d(2,6,float('inf'))
    
    # # g.addEdge_1d(3,0,float('inf'))
    # # g.addEdge_1d(3,1,float('inf'))
    # # g.addEdge_1d(3,5,float('inf'))
    
    # # g.addEdge_1d(4,0,float('inf'))
    # # g.addEdge_1d(4,1,float('inf'))
    # # g.addEdge_1d(4,2,float('inf'))
    # # g.addEdge_1d(4,6,float('inf'))
    
    # # g.addEdge_1d(5,0,float('inf'))
    # # g.addEdge_1d(5,1,float('inf'))
    # # g.addEdge_1d(5,2,float('inf'))
    # # g.addEdge_1d(5,3,float('inf'))
    # # g.addEdge_1d(5,6,float('inf'))
    
    # # g.addEdge_1d(6,0,float('inf'))
    # # g.addEdge_1d(6,1,float('inf'))
    # # g.addEdge_1d(6,2,float('inf'))
    # # g.addEdge_1d(6,4,float('inf'))
    # # g.addEdge_1d(6,5,float('inf'))
    # # g.printGraphAjMatrix_1d()
    
    # #### transform environment to joint state space graph
    # # g.transform_EnvToJointStateGraph()
    # # g.printGraphAjList_2d()
    # ##### find shortest path
    # g.shortest_path_cost(source, destination)
    
    #### 1st plot the environment graph
    # graph_edges = g.getEnvironmentGraphEdges()
    # print(graph_edges)
    # graphPlot.plot_environment_graph(E= graph_edges)
    
    #### 2nd plot the joint state space graph
    # joint_state_space_edges = g.getJointStateGraphEdges()
    # print(joint_state_space_edges )
    # src = g.get_source_JSG()
    # dest = g.get_destination_JSG()
    # graphPlot.plot_joint_state_space_graphTT(S=src, D=dest,E=joint_state_space_edges)
    
    ###################################