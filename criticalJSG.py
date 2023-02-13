
from collections import defaultdict
from vanillaDijkstra import EGraph

class CJSGraph:
    def __init__(self, V):
        self.V = V
        self.adj =  [[0 for column in range(self.V)] for row in
                      range(self.V)] 
        self.T = [] # list of dict of node sets and edge sets
        self.M = set() #  total node sets
        self.H = defaultdict(list) # edge sets with the edge cost
        # calculate riky edges along with support nodes 
        self.ESS = set()
        self.S11 = (None,None)
        self.Sgg = (None,None)
        self.adj_ns = [[0 for column in range(self.V)] for row in
                      range(self.V)] 
        # shortest path node sequence  
        self.shortest_path_nodes = [[float('inf') for column in range(self.V)] for row in
                            range(self.V)]
        # CJSG edges
        self.cjs_graph_edges = []
    # 1. node sets for CJSG
    def nodesets_CGSG(self):
        for i in range(self.V):
            for j in range(self.V):
                if self.adj[i][j]!=float('inf') and self.adj[i][j]!=0 and self.adj[i][j][1]!=():
                    #print((i,j),adj[i][j])
                    # calculate node sets for CJSG   
                    ES = (i,j) # risky edge with support nodes
                    Sij=self.adj[i][j][1] # support nodes 
                    self.ESS.add(ES)
                    print(ES, Sij)
                    for u in ES:
                        for v in Sij:
                            self.M.add((u,v))
                            self.M.add((v,u))  
        self.add_start_and_goal()                   
    # 2. add start and goal agents position if not present in the node sets
    def add_start_and_goal(self): 
        if self.S11 not in self.M:
            self.M.add(self.S11)
        if self.Sgg not in self.M:
            self.M.add(self.Sgg) 
        print("ESS set {}",format(self.ESS))
    # 3. calculate edge sets for CJSG
    # add adj list of CJSG for node sets with edge cost
    # (u,v) one node, (x,y) another node
    def addEdge_H(self,u,v,x,y,w):
        if [w,(x,y)] not in self.H[(u,v)] and w!= float('inf'):
            self.H[(u,v)].append([w,(x,y)])
        if [w,(u,v)] not in self.H[(x,y)] and w!= float('inf'):
            self.H[(x,y)].append([w,(u,v)])

    # agents movement without support p->r, q->l
    def edge_cost(self,p,r,q,l,support=False):
        Rpr = 0
        Rql = 0
        if self.adj[p][r]!=float('inf') and self.adj[p][r]!=0:
            Rpr = self.adj[p][r][0]
        else:
            Rpr = self.adj[p][r]
        
        if self.adj[q][l]!=float('inf') and self.adj[q][l]!=0:
            Rql = self.adj[q][l][0]
        else:
            Rql = self.adj[q][l] 
        R_result = Rpr+Rql
        if (support):
            return  R_result/2      
        return R_result
    # convert environt Graph to Critical Joint State Graph
    def critical_JSG(self):
        count = 1    
        for x in self.M:
            print("Iteration: {}".format(count))
            for y in self.M:
                if x!=y: # assumption no self loop
                    # x: pq and y = rl so x[0]y[0]: pr and x[1]y[1]: ql
                    p,q = x[0],x[1]
                    r,l = y[0],y[1]
                    
                    # edge cost without support
                    Rpq_rl = self.edge_cost(p,r,q,l)
                    # edge ql in ES and p==r and belongs to Sql (support nodes for the risky edge ql)
                    if (q,l) in self.ESS and p==r and p in self.adj[q][l][1]: # Sij
                        print((p,q),(r,l))
                        print("QL in ES !!!!!!!!!") 
                        # edge cost with support
                        Cql_s = self.edge_cost(p,r,q,l, support=True)
                        w = min(Cql_s,Rpq_rl)
                        self.addEdge_H(p,q,r,l, w) 
                    # edge pr in ES and q==l and belongs to Spr (support nodes for the risky edge pr)
                    elif (p,r) in self.ESS and q==l and q in self.adj[p][r][1]:# Sij
                        print((p,q),(r,l))
                        Cpr_s = self.edge_cost(p,r,q,l, support=True)
                        w = min(Cpr_s,Rpq_rl)
                        print(w)
                        print("PR in ES !!!!!!!!!") 
                        self.addEdge_H(p,q,r,l, w) 
                    else:
                        print((p,q),(r,l))
                        w = Rpq_rl
                        if w!=float('inf'):
                            print("Yay!!!!!",w)
                        self.addEdge_H(p,q,r,l, w) 
            
            count=count+1
            
    def vanilla_dijkstra(self,p,r,q,l):
        g = EGraph(self.V) 
        g.graph = self.adj_ns
        Rpr = g.dijkstra(src=p,dest=r)
        Rql = g.dijkstra(src=q,dest=l)
        return Rpr+Rql
    def add_remaning_cjsg_nodesets(self): 
        for nodex in sorted(self.M):  
            for nodey in sorted(self.M):
                if nodex!= nodey:
                    present_next_nodes = [next_node for weight, next_node  in sorted(self.H[nodex])]
                    print(nodex, nodey, present_next_nodes)
                    if nodey not in present_next_nodes:
                        print(nodey)
                        p,q = nodex[0],nodex[1]
                        r,l = nodey[0],nodey[1]
                        # Rpr+Rql
                        print(p,r,q,l)
                        Rpq_rl = self.vanilla_dijkstra(p,r,q,l)
                        self.addEdge_H(p,q,r,l, w=Rpq_rl) 
    
    def transform_EG_to_CJSG(self):
        self.nodesets_CGSG()
        self.critical_JSG()
        self.add_remaning_cjsg_nodesets()
        
                            
    # mim distance utility function: shortest path algorithm                        
    def minDistance_2d(self,dist_2d, seen_2d):
            min_d = float('inf')
            for u in range(self.V):
                for v in range(self.V):
                    print((u,v))
                    if (u,v) not in seen_2d and dist_2d[u][v] < min_d:
                        min_d = dist_2d[u][v]
                        min_vertex =u,v
            return min_vertex
    # shortest path algorithm for the CJSG         
    def shortest_path_cost(self):
            iter = len(self.M)
            dist_2d = [[float('inf') for column in range(self.V)] for row in
                        range(self.V)]
            
            seen_2d = set()
            dist_2d[self.S11[0]][self.S11[1]] = 0
            
            self.shortest_path_nodes[self.S11[0]][self.S11[1]] = 0
            
            for _ in range(iter):
                print("\n")
                print("Iteration",_)
                u,v = self.minDistance_2d(dist_2d, seen_2d)
                seen_2d.add((u,v))
                print("seen_node, next_node, edge_cost, dist_from_source")
                print(self.H[(u,v)])
                for weight, nxt_node in sorted(self.H[(u,v)]):
                    print((u,v), nxt_node, weight)
                    x,y = nxt_node 
                    if nxt_node not in seen_2d and dist_2d[x][y] > dist_2d[u][v] + weight:
                        #print(">>>>>>>>>>>>>>>>>>>>>>>>")
                        dist_2d[x][y] = dist_2d[u][v] + weight
                        #print((u,v),(x,y),weight,dist_2d[x][y])
                        #print(">>>>>>>>>>>>>>>>>>>>>>>>>")
                        # these are node positions with next shortest path node positions (for two agents)
                        self.shortest_path_nodes[x][y]=(u,v)
                             
            print("\nList of visted Agent Positions:")
            print(sorted(seen_2d))
            print("\nAdj Matrix for the Distance of Agents from Source")
            #print(dist_2d) in adj matrix form 
            # distance from source to agent position
            print(dist_2d)
            self.printSolution(dist_2d)  
            print("-----------------------")  
            self.printShortestPath(dist_2d)
           # print(self.shortest_path_nodes)             
    def shortest_path_sequence(self,start_node, target_node):   
        path = []
        node = target_node
        
        while node != start_node:
            path.append(node)
            node = self.shortest_path_nodes[node[0]][node[1]]
    
        # Add the start node manually
        path.append(start_node)
        print("Final Shortest Path ",[r for r in reversed(path)])             
    # print environment graph given to us       
    def printAdjMatrixEG(self):
        print("Environment Graph:")
        for row in sorted(self.adj):   
            print(row,self.adj[row]) 
    # print CJSG node sets only
    def printNodeSetsCJSG(self):
        print("Node Sets: Critical Joint State Graph:")
        for m in sorted(self.M):   
            print(m) 

    # print CJSG edge sets and their cost    
    def printEdgeSetsAndCostCJSG(self):
        print("EdgeSets and Cost: Critical Joint State Graph:")
        for row in sorted(self.H):   
            print(row,self.H[row]) 
    # print the shortest path cost for CJSG        
    def printSolution(self,dist_2d):
            print("Source\tAgent_Position\tDistance_from_Source")
            for u in range(self.V):   
                for v in range(self.V):
                    print((0,0),(u,v), dist_2d[u][v]) 
    # print the shortest path cost for CJSG        
    def printShortestPath(self,dist_2d):
            print("Source\tAgent_Position\tDistance_from_Source")
            for u in range(self.V):   
                for v in range(self.V):
                    if (u,v) == self.Sgg:
                        print(self.S11,(u,v), dist_2d[u][v]) 
     

    # get joint state space graph edges
    def getCJSG_Edges(self):
        print("\n Critical Joint State Space Graph Edges")
        for u in range(self.V):
            for v in range(self.V):
                for weight, nxt_node in sorted(self.H[(u,v)]): 
                    print((u,v),(nxt_node), weight) 
                    if ((u,v),(nxt_node), weight) not in self.cjs_graph_edges:
                        self.cjs_graph_edges.append((str((u,v)),str((nxt_node))))
        return self.cjs_graph_edges

