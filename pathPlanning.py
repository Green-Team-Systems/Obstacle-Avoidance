import dijkstar

graph = dijkstar.Graph()
graph.add_edge(1, 2, 110)
graph.add_edge(2, 3, 125)
graph.add_edge(3, 4, 108)
print(dijkstar.find_path(graph, 1, 4))

input("Press Enter to continue...")

graph = dijkstar.Graph()
graph.add_edge(1, 2, (110, 'Main Street'))
graph.add_edge(2, 3, (125, 'Main Street'))
graph.add_edge(3, 4, (108, '1st Street'))
def cost_func(u, v, edge, prev_edge):
    length, name = edge
    if prev_edge:
        prev_name = prev_edge[1]
    else:
        prev_name = None

    cost = length
    if name != prev_name:
        cost += 10
    return cost
print(dijkstar.find_path(graph, 1, 4, cost_func=cost_func))

input("Press Enter to continue...")

graph = dijkstar.Graph(undirected=True)
n = int(input("Input n (greater than 1): "))
m = int(input("Input m (same deal): "))
o = int(input("Input o (same deal): "))

for i in range(n):
    for j in range(m):
        for k in range(o):
            if i == 0 and j == 0 and k == 0:
                graph.add_node((i, j, k))
            elif i == 0 and j == 0:
                graph.add_node((i, j, k), {(i, j, k - 1): 0})
            elif i == 0 and k == 0:
                graph.add_node((i, j, k), {(i, j - 1, k): 0})
            elif j == 0 and k == 0:
                graph.add_node((i, j, k), {(i - 1, j, k): 0})
            elif i == 0:
                graph.add_node((i, j, k), {(i, j - 1, k): 0, (i, j, k - 1): 0})
            elif j == 0:
                graph.add_node((i, j, k), {(i - 1, j, k): 0, (i, j, k - 1): 0})
            elif k == 0:
                graph.add_node((i, j, k), {(i - 1, j, k): 0, (i, j - 1, k): 0})
            else:
                graph.add_node((i, j, k), {(i, j - 1, k): 0, (i - 1, j, k): 0, (i, j, k - 1): 0})

print(graph.get_data())
for i in graph.get_data():
    print(i)
    for j in graph.get_data()[i]:
        print("   " + str(j))
print()
print()
print(dijkstar.find_path(graph, (0, 0, 0), (n - 1, m - 1, o - 1)).nodes)




