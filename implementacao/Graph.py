import numpy as np #utilizada para criar arrays vazios, por exemplo.
from queue import Queue #utlizada como fila no bfs
import copy #utilizada para copiar dicts/arrays sem pegar sua referência de memória
from fibheap import * #utilizada como pilha com ordenação para o dijkstra
tempo = 0 #utilizada como tempo para o dfs
changes = 0 #utilizada para verificar alterações em uma iteração completa do bellman ford.
class ListaAdj():
    """Lista de adjacências feita através de um 
       dicionário, onde 'tam' é a qtde de vértices
       do grafo e type o seu tipo (arestas - 0, arcos - 1)"""
    def __init__(self, tam, type):
        self.lista = {} #dicionário representando o grafo, organizado em 'chave':[(vertice, peso),...]
        self.type = type # tipo: (grafo - 0, digrafo - 1)
        self.tam = tam # quantidade de vértices
        for i in range(tam):
            self.lista.update({i+1:[]}) #criação da lista com os vértices indo de 1 ao tamanho, sem arestas
    def addAresta(self, v1, v2, w):
        '''Função que adiciona arestas, recebendo a uv aresta e seu peso.'''
        self.lista[v1].append((v2, w))
        if(not self.type): # se nao for digrafo
            self.lista[v2].append((v1, w)) #adiciona a aresta vu também
    def viz(self, v):
        '''Função que retorna os vizinhos de v'''
        vizinhos = []
        for i in range(self.lista[v].__len__()): #itera sobre os vizinhos de v
            vizinhos.append(self.lista[v][i][0]) #adiciona o vértice a um array
        return vizinhos #retorna o array com os vértices dos vizinhos
    def vizneg(self, v):
        if(self.type): #se for um digrafo, calcula sua vizinhança negativa
            viz = []
            #itera sobre todas as arestas, procurando as que atingem 'v'
            for i in self.lista.keys():
                for j in self.lista[i]:
                    if(j[0] == v): 
                        viz.append(i) 
            return viz
        return None
    def bfs(self, v):
        '''Breadth First Search'''
        lista2 = {}
        for i in self.lista.keys():
            lista2.update({i:[0,float('inf'), None]}) #incialização de cor, d, pi
            if(i == v): #no vertice v escolhido
                lista2[v][0] = 1 #coloca sua cor para cinza
                lista2[v][1] = 0 #coloca sua distância para 0
        fila = Queue(self.tam) #inicia uma fila com a quantidade de vértices
        fila.put(v) #coloca v na fila
        while(not fila.empty()):
            u = fila.get() #retira um elemento da fila
            for i in range(self.lista[u].__len__()): # itera sobre os vizinhos do elemento retirado da fila
                if(lista2[self.lista[u][i][0]][0] == 0): #se cor == branco
                    lista2[self.lista[u][i][0]][0] = 1 #cor = cinza
                    lista2[self.lista[u][i][0]][1] = lista2[u][1]+1 #d = d+1
                    lista2[self.lista[u][i][0]][2] = u #pi = u
                    fila.put(self.lista[u][i][0])
            lista2[u][0] = 2 #cor = preto
        d = {}
        pi = {}
        #itera sobre a lista2, para separá-la em 2 listas, d e pi(apenas para organização)
        for i in self.lista.keys():
            d.update({i:lista2[i][1]})
            pi.update({i:lista2[i][2]})
        return d, pi #retorna d e pi
    def iniciadfs(self, v):
        '''Função que faz os preparativos iniciais e chama o dfs'''
        global tempo #variável global para marcar o tempo
        lista2 = {}
        fpi = [] #floresta em profundidade
        for i in self.lista.keys():
            lista2.update({i:[0, None, float('-inf'), float('inf')]}) #cor, pi, vini, vfim
        tempo = 0
        self.dfs(v, lista2, fpi) #chama dfs que atua de modo recursivo, atualizando 'lista2'
        pi = {}
        vini = {}
        vfim = {}
        #itera sobre a lista2, para separá-la em 3 listas, vini, vfim e pi(apenas para organização)
        for i in self.lista.keys():
            pi.update({i:lista2[i][1]})
            vini.update({i:lista2[i][2]})
            vfim.update({i:lista2[i][3]})
        return pi, vini, vfim, fpi #retorna pi, vini, vfim e também a floresta em profundidade
    def dfs(self, v, lista2, fpi):
        global tempo
        tempo +=1 #a cada iteração, o tempo aumenta
        lista2[v][0] = 1 #cor = cinza
        lista2[v][2] = tempo #vini = tempoAtual
        for vertice in self.lista[v]: #para cada vizinho de v
            if(lista2[vertice[0]][0] == 0): #corVizinhoDeV = branco
               lista2[vertice[0]][1] = v #piVizinhoDeV = v
               fpi.append((v, vertice[0], vertice[1])) #adiciona a aresta entre v e seu vizinho na floresta em profundidade
               self.dfs(vertice[0], lista2, fpi) #chama dfs recursivamente
        tempo +=1 #aumenta o tempo novamente
        lista2[v][0] = 2 #cor de v = preto
        lista2[v][3] = tempo #vfim = tempo
        return lista2, fpi #retorna lista2 com pi, vini e vfim e retorna fpi(floresta em profundidade)
    def bf(self, v):
        '''Função que executa o algoritmo de bellman ford'''
        global changes #variável global que detecta mudanças em uma iteração do bf
        lista2 = {}
        for i in self.lista.keys():
            lista2.update({i:[None,float('inf')]}) #inicialização de pi e d
        lista2[v][1] = 0 # d(v) <- 0
        #executa n vezes
        for i in range(self.lista.__len__()):
            changes = 0 #reseta a variável changes
            #itera sobre todas as arestas do grafo
            for u in self.lista.keys():
                for ve in self.lista[u]:
                    #relaxa cada aresta individualmente
                    self.relaxa(u, ve, lista2)
            #se não houverem mudanças em um relaxamento de todas as arestas
            #significa que não há mais o que relaxar, então termina o algoritmo
            if(changes == 0):  
                print(i) #mostra quantas iterações foram necessárias
                break
        #verifica se ainda existem relaxamentos possíveis, caso hajam, existem ciclos negativos
        for i in self.lista.keys():
            for j in self.lista[i]:
                if(lista2[i][1]>lista2[j[0]][1]+j[1]):
                    print("Grafo com ciclos negativos")
                    return
        pi = {}
        d = {}
        #itera sobre a lista2, para separá-la em 2 listas, pi e d(apenas para organização)
        for i in self.lista.keys():
            pi.update({i:lista2[i][0]})
            d.update({i:lista2[i][1]})
        return pi, d
    def relaxa(self, u, v, lista2):
        #algoritmo de relaxamento
        global changes
        #se v.d for maior que o u.d + peso de uv
        if(lista2[v[0]][1]>lista2[u][1]+v[1]):
            lista2[v[0]][1] = lista2[u][1]+v[1] #v.d = u.d + peso de uv
            lista2[v[0]][0] = u #v.pi = u
            changes = 1 #indica que houveram mudanças
            return True #retorna que houveram mudanças(usada para ajustar o novo peso de v na pilha do dijkstra)
    def detectaCiclo(self, tam):
        pi, vini, vfim, fpi = self.iniciadfs(1)
        
        count = 0
        #cria uma cópia do grafo
        lista2 = copy.deepcopy(self.lista)
        #percorre todas as arestas da árvore em profundidade
        for i in range(fpi.__len__()):
            #remove as arestas da árvore em profundidade da cópia do grafo
            lista2[fpi[i][0]].remove((fpi[i][1], fpi[i][2]))
        #itera sobre todas as arestas restantes do grafo cópia
        for u in lista2.keys():
                for ve, w in lista2[u]: #vertice e peso da aresta entre u e ve
                        #se ve.ini<u.ini e ve.fim>u.fim, u é descendente de ve
                        if(vini[ve]<vini[u] and vfim[ve]>vfim[u]):
                            #caso o tempo entre u.ini e ve.ini seja diferido de 
                            #'tam'(tamanho do ciclo procurado) e menor 
                            #que 10(restrição para evitar procurar caminhos muito grandes)
                            if((vini[u]-vini[ve])>tam and (vini[u]-vini[ve])<10):        
                                filho = u #descendente de ve, u é guardado
                                caminho = [filho] #o ciclo é inicializado com u
                                pai = pi[filho] # pai <- u.pi
                                while(pai != ve): #enquanto filho.pi != ve
                                    caminho.append(pi[filho]) #adiciona os novos vértices ancestrais de u no ciclo para ve
                                    pai = pi[pai] # pai <- pai.pi
                                    filho = pi[filho] #filho <- filho.pi
                                caminho.append(pai) # adiciona ve ao final do ciclo
                                if(caminho.__len__()>=5): #se o ciclo encontrado tiver tamanho >=5, retorna o ciclo
                                    return caminho
    def dijkstra(self, v):
        lista2 = {}
        q = Fheap() #incializa a pilha ordenada
        V = np.zeros((self.tam), dtype=Node) # V é um array com n posições do tamanho de um nó da pilha
        for i in range(self.tam):
            lista2.update({i+1:[None,float('inf')]}) # inicialização de pi e d
            V[i] = q.insert(Node(key=float('inf'), value=i+1)) #inserção dos nós da pilha em V
        lista2[v][1] = 0 # d <- 0
        q.decrease_key(V[v-1], 0) # d(nó(v)) <- 0
        #itera sobre todas as arestas do grafo
        for i in self.lista.keys():
            for viz in self.lista[i]:
                #adiciona seus vizinhos da estrutura do nó
                V[i-1].addViz(V[viz[0]-1])
        s = [] #conjunto de vértices com distâncias bem calculadas
        count = 0
        while(q.num_nodes !=0): #enquanto houverem nós na pilha
            u = q.extract_min() #retira o minimo
            s.append(u) #coloca o vértice retirado em u, pois pelo teorema, sua distância
                        #está bem calculada
            if(s.__len__()%10000 == 0): #apenas para conferência de porcentagem do algoritmo
                print(f'{(s.__len__()/self.tam)*100}%')
            count = 0 #contador auxiliar para acessar o mesmo vizinho em V(manter os valores corretos de distância na pilha)
            #para cada vizinho de u
            for ve in self.lista[u.value]:
                no = u.viz()[count] #recebe o nó referente ao vizinho iterado
                count+=1 
                relaxou = self.relaxa(u.value, ve, lista2) #relaxa aquela aresta (u, ve)
                if(relaxou): #se tiver relaxado, ajusta a distância daquele nó na pilha, para manter a ordem
                    q.decrease_key(no, lista2[ve[0]][1])
        pi = {}
        d = {}
        #itera sobre a lista2, para separá-la em 2 listas, pi e d(apenas para organização)
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
        #le as linhas do arquivo
        while True:
                count += 1
                line = file.readline()
                #se as linhas acabarem, encerra o laço
                if not line:
                    break
                line = line.split(" ") #divide a linha por espaço
                if(line[0] =='a'): #se for um arco
                    #chama a função de adicionar aresta uv e passa u, v e w(uv)
                    self.lista.addAresta(int(line[1]), int(line[2]), int(line[3]))
                #se for p, extrai as informações sobre quantidade de vértices e arcos do grafo
                elif (line[0] == 'p'):
                    self.n = int(line[2])
                    self.m = int(line[3])
                    self.lista = ListaAdj(self.n, 1)  #cria a lista de adjacências
        file.close()
    #já comentado em ListaAdj
    def viz(self, v):
            return self.lista.viz(v)
    def d(self, v):
            return len(self.lista.viz(v)) #positivo(pega a quantidade de vizinhos)
    def dneg(self, v):
            return len(self.lista.vizneg(v)) #negativa(pega a quantidade de vizinhos)
    def mind(self):
        minimoD = (1, self.n) #inicialmente recebe o vertice 1 e o grau maximo+1(n)
        #itera sobre os vértices
        for i in self.lista.lista.keys():
            #calcula seu grau positivo
            dpos = self.d(i)
            if(dpos<minimoD[1]): #se ele for menor que o menor já encontrado
                minimoD = (i, dpos) #substitui o antigo pelo novo encontrado
        return minimoD[1] #minimo grau positivo
    #análogo ao mind, apenas invertendo a comparação
    def maxd(self):
        maximoD = (1, 0)
        for i in self.lista.lista.keys():
            dpos = self.d(i)
            if(dpos>maximoD[1]):
                maximoD = (i, dpos)
        return maximoD[1]
     #já comentado em ListaAdj
    def bfs(self, v):
            return self.lista.bfs(v)
     #já comentado em ListaAdj
    def dfs(self, v):
            return self.lista.iniciadfs(v)
     #já comentado em ListaAdj
    def bf(self, v):
            return self.lista.bf(v)
     #já comentado em ListaAdj
    def detectaCiclo(self, tam):
            return self.lista.detectaCiclo(tam)
     #já comentado em ListaAdj
    def djikstra(self, v):
            return self.lista.dijkstra(v)
#análogo ao DiGraph(as diferenças foram implementadas na classe ListaAdj)
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