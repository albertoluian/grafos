import numpy as np
from queue import Queue
import copy
from fibheap import *
tempo = 0
changes = 0
class ListaAdj():
    """Lista de adjacências feita através de um 
       dicionário, onde 'tam' é a qtde de vértices
       do grafo e type o seu tipo (arestas - 0, arcos - 1)"""
    def __init__(self, tam, type):
        self.lista = {}
        self.type = type
        self.tam = tam
        for i in range(tam):
            self.lista.update({i+1:[]})
    def addAresta(self, v1, v2, w):
        self.lista[v1].append((v2, w))
        if(not self.type):
            self.lista[v2].append((v1, w))
    def viz(self, v):
        vizinhos = []
        for i in range(self.lista[v].__len__()):
            vizinhos.append(self.lista[v][i][0])
        return vizinhos
    def vizneg(self, v):
        viz = []
        for i in self.lista.keys():
            for j in self.lista[i]:
                if(j[0] == v):
                    viz.append(i)
        return viz
    def bfs(self, v):
        lista2 = {}
        for i in self.lista.keys():
            lista2.update({i:[0,float('inf'), None]}) #cor, d, pi
            if(i == v):
                lista2[v][0] = 1
                lista2[v][1] = 0
        fila = Queue(self.tam)
        fila.put(v)
        while(not fila.empty()):
            u = fila.get()
            for i in range(self.lista[u].__len__()):
                if(lista2[self.lista[u][i][0]][0] == 0):
                    lista2[self.lista[u][i][0]][0] = 1 #cinza
                    lista2[self.lista[u][i][0]][1] = lista2[u][1]+1 #d+1
                    lista2[self.lista[u][i][0]][2] = u #pi
                    fila.put(self.lista[u][i][0])
            lista2[u][0] = 2 #preto
        d = {}
        pi = {}
        for i in self.lista.keys():
            d.update({i:lista2[i][1]})
            pi.update({i:lista2[i][2]})
        return d, pi
    def iniciadfs(self, v):
        global tempo
        lista2 = {}
        fpi = [] #floresta em profundidade
        for i in self.lista.keys():
            lista2.update({i:[0, None, float('-inf'), float('inf')]}) #cor, pi, ini, fim
        tempo = 0
        self.dfs(v, lista2, fpi)
        pi = {}
        vini = {}
        vfim = {}
        for i in self.lista.keys():
            pi.update({i:lista2[i][1]})
            vini.update({i:lista2[i][2]})
            vfim.update({i:lista2[i][3]})
        return pi, vini, vfim, fpi
    def dfs(self, v, lista2, fpi):
        global tempo
        tempo +=1
        lista2[v][0] = 1
        lista2[v][2] = tempo
        for vertice in self.lista[v]:
            if(lista2[vertice[0]][0] == 0): #branco
               lista2[vertice[0]][1] = v #pi
               fpi.append((v, vertice[0], vertice[1]))
               self.dfs(vertice[0], lista2, fpi)
        tempo +=1
        lista2[v][0] = 2
        lista2[v][3] = tempo
        return lista2, fpi
    def bf(self, v):
        global changes
        lista2 = {}
        for i in self.lista.keys():
            lista2.update({i:[None,float('inf')]}) #pi, d
        lista2[v][1] = 0 # d <- 0
        for i in range(self.lista.__len__()):
            changes = 0
            for u in self.lista.keys():
                for ve in self.lista[u]:
                    self.relaxa(u, ve, lista2)
            if(changes == 0):
                print(i)
                break
        for i in self.lista.keys():
            for j in self.lista[i]:
                if(lista2[i][1]>lista2[j[0]][1]+j[1]):
                    print("Grafo com ciclos negativos")
                    return
        pi = {}
        d = {}
        for i in self.lista.keys():
            pi.update({i:lista2[i][0]})
            d.update({i:lista2[i][1]})
        return pi, d
    def relaxa(self, u, v, lista2):
        global changes
        if(lista2[v[0]][1]>lista2[u][1]+v[1]):
            lista2[v[0]][1] = lista2[u][1]+v[1]
            lista2[v[0]][0] = u
            changes = 1
            return True
    def detectaCiclo(self, tam):
        pi, vini, vfim, fpi = self.iniciadfs(1)
        #percorre todas as arestas
        count = 0
        lista2 = copy.deepcopy(self.lista)
        for i in range(fpi.__len__()):
            lista2[fpi[i][0]].remove((fpi[i][1], fpi[i][2]))
        for u in lista2.keys():
                for ve, w in lista2[u]:
                        if(vini[ve]<vini[u] and vfim[ve]>vfim[u]):
                            if((vini[u]-vini[ve])>tam and (vini[u]-vini[ve])<10):        
                                filho = u
                                caminho = [filho]
                                pai = pi[filho]
                                while(pai != ve):
                                    caminho.append(pi[filho])
                                    pai = pi[pai]
                                    filho = pi[filho]
                                caminho.append(pai)
                                if(caminho.__len__()-1>=5):
                                    return caminho
    def dijkstra(self, v):
        lista2 = {}
        q = Fheap()
        V = np.zeros((self.tam), dtype=Node)
        count = 0
        flag = True
        for i in range(self.tam):
            if(flag):
                if(i == v):
                    flag = False
                else:
                    count+=1
            lista2.update({i+1:[None,float('inf')]}) #pi, d
            V[i] = q.insert(Node(key=float('inf'), value=i+1))
        lista2[v][1] = 0 # d <- 0
        q.decrease_key(V[v-1], 0)
        for i in self.lista.keys():
            for viz in self.lista[i]:
                V[i-1].addViz(V[viz[0]-1])
        s = []
        count = 0
        while(q.num_nodes !=0):
            u = q.extract_min()
            s.append(u)
            if(s.__len__()%10000 == 0):
                print(s.__len__())
            count = 0
            for ve in self.lista[u.value]:
                no = u.viz()[count]
                count+=1
                relaxou = self.relaxa(u.value, ve, lista2)
                if(relaxou):
                    q.decrease_key(no, lista2[ve[0]][1])
        pi = {}
        d = {}
        for i in self.lista.keys():
            pi.update({i:lista2[i][0]})
            d.update({i:lista2[i][1]})
        return pi, d

class DiGraph():
    '''Classe do grafo direcionado, recebe um arquivo''' 
    def __init__(self, filePath=str):
        self.type = 0
        self.n = 0
        self.m = 0
        file = open(filePath)
        count = 0
        while True:
                count += 1
                line = file.readline()
                if not line:
                    break
                line = line.split(" ")
                if(line[0] =='a'):
                    self.lista.addAresta(int(line[1]), int(line[2]), int(line[3]))
                elif (line[0] == 'p'):
                    self.n = int(line[2])
                    self.m = int(line[3])
                    self.lista = ListaAdj(self.n, 1) 
        file.close()
    def viz(self, v):
            return self.lista.viz(v)
    def d(self, v):
            return len(self.lista.viz(v)) #positivo
    def dneg(self, v):
            return len(self.lista.vizneg(v)) #negativa
    def mind(self):
        minimoD = (1, self.n)
        for i in self.lista.lista.keys():
            dpos = self.d(i)
            if(dpos<minimoD[1]):
                minimoD = (i, dpos)
        return minimoD[1] #minimo grau positivo e negativo
    def maxd(self):
        maximoD = (1, 0)
        for i in self.lista.lista.keys():
            dpos = self.d(i)
            if(dpos>maximoD[1]):
                maximoD = (i, dpos)
        return maximoD[1]
    def bfs(self, v):
            return self.lista.bfs(v)
    def dfs(self, v):
            return self.lista.iniciadfs(v)
    def bf(self, v):
            return self.lista.bf(v)
    def detectaCiclo(self, tam):
            return self.lista.detectaCiclo(tam)
    def djikstra(self, v):
            return self.lista.dijkstra(v)
class Graph():
    '''Classe do grafo, recebe um arquivo''' 
    def __init__(self, filePath=str):
        self.type = 0
        self.n = 0
        self.m = 0
        file = open(filePath)
        count = 0
        while True:
                count += 1
                line = file.readline()
                if not line:
                    break
                line = line.split(" ")
                if(line[0] =='e'): #edge
                    self.lista.addAresta(int(line[1]), int(line[2]), int(line[3]))
                elif (line[0] == 'p'):
                    self.n = int(line[2])
                    self.m = int(line[3])
                    self.lista = ListaAdj(self.n, 0) 
        file.close()
    def viz(self, v):
            return self.lista.viz(v)
    def d(self, v):
            return len(self.lista.viz(v))
    def mind(self):
        minimoD = (1, self.n)
        for i in self.lista.lista.keys():
            d = self.d(i)
            if(d<minimoD[1]):
                minimoD = (i, d)
        return minimoD[1]
    def maxd(self):
        maximoD = (1, 0)
        for i in self.lista.lista.keys():
            d = self.d(i)
            if(d>maximoD[1]):
                maximoD = (i, d)
        return maximoD[1]
    def bfs(self, v):
            return self.lista.bfs(v)
    def dfs(self, v):
            return self.lista.iniciadfs(v)
    def bf(self, v):
            return self.lista.bf(v)
    def detectaCiclo(self, tam):
            return self.lista.detectaCiclo(tam)
    def djikstra(self, v):
            return self.lista.dijkstra(v)