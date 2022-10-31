#Grupo 23 | 90964 Miguel de Sousa Almeida Infante Mota | 90768 Pedro Miguel Drogas Moreira
import pickle
import queue
import copy
import time
import math
from itertools import product


class Node:
    def __init__(self, node, transport, parent, depth, tickets):
        self.node = node
        self.transport = transport
        self.parent = parent
        self.tickets = tickets
        self.depth = depth
        pass

    def __str__(self):
        return "[" + str(self.node) + ", " + str(self.transport) + ", " + str(self.parent) + ", " + str(self.tickets) + "]"

class SearchProblem:
    BUS = 1
    TAXI = 0
    METRO = 2

    def __init__(self, goal, model, auxheur=[]):
        self.goal = goal
        self.model = model
        self.coords = auxheur
        self.numagents = len(goal)
        self.graph = model
        pass

    def BFS(self, queue, nodes):
        current = None
        depth = 0
        if queue.empty():
            return
        if not queue.empty():
            current = queue.get(False)
            depth = current.depth
        for child in self.graph[current.node]:
            if current.tickets[child[0]] > 0:
                current.tickets[child[0]] -=1
                children = Node(child[1], child[0], current, current.depth+1, copy.copy(current.tickets))
                #print(current.tickets)
                queue.put(children)
                nodes.append(children)
        return depth

    def isIntersecting(self, src_nodes, dst_nodes):
        returnList = []
        for z in src_nodes:
            for x in dst_nodes:
                if z.node == x.node:
                    returnList.append([z, x])
        return returnList

    def formatPath(self, path):
        returnList = []
        length = len(path[0])
        for x in range(0, length):
            #print("x = " + str(x))
            ticketList = []
            positionList = []
            for y in path:
                #print("y =" + str(y))
                #print(ticketList)
                if y[x][0] != []:
                    ticketList.append(y[x][0][0])
                else:
                    ticketList.append(y[x][0])
                positionList.append(y[x][1][0])
            returnList.append([ticketList, positionList])
            #print("returlist = " +  str(returnList))
        return returnList

    def printPath(self, intersectNode, init, goal, pathList, anyorder):
        for y in intersectNode:
            path = []
            pathDst = []
            i = y[0]
            while i.parent != None:
                pathDst.append([[i.transport], [i.node]])
                i = i.parent
            pathDst.append([[], [i.node]])
            src = i.node
            i = y[1]
            while i.parent != None:
                path.append([[i.transport], [i.parent.node]])
                i = i.parent
            dst = i.node
            if not anyorder and init.index(src) != goal.index(dst):
                continue
            if (pathDst[::-1] + path not in pathList[init.index(src)]):
                pathList[init.index(src)].append(pathDst[::-1] + path)

    def search(self, init, limitexp = 2000, limitdepth = 10, tickets = [math.inf, math.inf, math.inf], anyorder = False):
        src = init
        dst = self.goal
        numagents = self.numagents
        currentexp = 0
        src_queues = [queue.Queue() for i in range(0,len(src))]
        dst_queues = [queue.Queue() for i in range(0,len(src))]
        src_Nodes = []
        dst_Nodes = []
        for y in range(0, numagents):
            src_Nodes.append(Node(src[y], -1, None, 0, copy.copy(tickets)))
            dst_Nodes.append(Node(dst[y], -1, None, 0, copy.copy(tickets)))
            src_queues[y].put(src_Nodes[y])
            dst_queues[y].put(dst_Nodes[y])
        intersectNodes = []
        paths = [[] for i in range(0,len(src))]
        availableTickets = copy.copy(tickets)
        currentDepth = 0
        #depthLimit = round(sum(availableTickets)/3) if sum(availableTickets) != math.inf else limitdepth
        #print(depthLimit)
        while not self.emptyqueues(src_queues + dst_queues):
            for x in range(0, numagents):
                currentDepth = self.BFS(src_queues[x], src_Nodes)
                currentexp += 1
                currentDepth = self.BFS(dst_queues[x], dst_Nodes)
                currentexp += 1
                if currentexp >= limitexp:
                    print("Exp Limit exceeded")
                    return []
                if currentDepth >= limitdepth:
                    print("Depth Limit exceeded")
                    return []
                tempintersectNode = self.isIntersecting(src_Nodes,dst_Nodes)
                #print("tempintersectNode = "+ str(tempintersectNode))
                intersectNode = list(filter(lambda y: y not in intersectNodes, tempintersectNode))
                if intersectNode != []:
                    intersectNodes.extend(intersectNode)
                    self.printPath(intersectNodes, src, dst, paths, anyorder)
                    #print(paths)
            #print("paths[0] =" + str(paths[0]))
            #print("paths[1] =" + str(paths[1]))
            #print("paths[2] =" + str(paths[2]))
            finalPaths = list(product(*paths))
            #print("FinalPaths = " + str(finalPaths))
            for x in finalPaths:
                if not self.lengthsequal(x):
                    continue
                currentList = self.formatPath(copy.copy(x))
                currentList[0][0] = []
                #print("CurrentList = " + str(currentList))
                if self.validTPath(currentList[1:], copy.copy(availableTickets), anyorder, dst):
                    #print("É valido")
                    currentList[0][0] = []
                    print(currentList)
                    return currentList

    def validTPath(self, path, t, anyorder, goal):
        for z in range(0, len(path)):
            #print("Valor recebido no validPath = " + str(x))
            #print(self.numagents)
            x = path[z]
            #print("Jogada " + str(z) + ":"+ str(x))
            for y in range(0, self.numagents):
                #print("entra aqui")
                t[x[0][y]] = t[x[0][y]] - 1
                #print("Entra aqui")
                if(t[x[0][y]]) < 0:
                    return False
            if len(set(x[1])) != self.numagents:
                return False
            if(not anyorder and z == (len(path) - 1)):
                return x[1] == goal
        return True

    def validPath(self, path, t):
        for x in path:
            t[x[0][0]] -= 1
            if (t[x[0][0]] < 0):
                return False
        return True

    def lengthsequal(self, path):
        length = len(path[0])
        for x in range(1,self.numagents):
            if len(path[x]) != length:
                return False
        return True


    def emptyqueues(self,queuelist):
        for x in queuelist:
            if not x.empty():
                return False
        return True