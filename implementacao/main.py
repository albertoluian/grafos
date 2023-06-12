from Graph import *
import sys
a = Graph("C:\\Users\\LUIAN\Desktop\\Teoria dos Grafos\\implementacao\\USA-road-d.NY.gr", 1)
# item a)
# print(a.mind()) # 1
# item b)
# print(a.maxd()) # 8
# item c) caminho com 10 ou mais arestas
# 4597
# 4598
# 4602
# 109
# 106
# 111
# 117
# 116
# 118
# 120
# 123
# 125
sys.setrecursionlimit(100000)
# pi, vini, vfim, fpi = a.dfs(1)
# caminho = a.detectaCiclo(5)
# print(caminho)
#item d)
# [452, 453, 477, 448, 449, 451, 457]
# (452, 453) -> (453, 477) -> (477, 448) -> (488, 449) -> (449, 451) -> (451, 457) -> (457, 452)
#item e)
pi, d = a.djikstra(129)
max = 0
index = 0
for i in d.keys():
    if(d[i]> max):
        max = d[i]
        index = i
print( max, index) # --> 1437303 90644
