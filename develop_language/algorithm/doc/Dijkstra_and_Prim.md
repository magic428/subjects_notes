# Dijkstra 与 Prim 算法的区别

转自：[刘毅](https://www.61mon.com/index.php/archives/206/)

Dijkstra 算法与 Prim 算法非常相似，甚至很多初学者觉得它们就是一样的。它们最直观的区别就是目的不同：前者求解最短路径，后者求解最小生成树。         

![](../pictures/Dijkstra_and_Prim.png)         

对比上图，          

- 最短路径         
```ruby
a->b  @length = 2
a->c  @length = 3
```

- 最小生成树           
```ruby
a->b->c  @sum = 4
```
结论：两者不一样！     
好，最后让我们回归代码。    
Dijkstra 算法与 Prim 算法都有一个数组，不妨统一称为R[ ]，我们每次都是取R[]的最小值，接着更新R[ ]，再取其最小值，，，往复下去。而这两者的区别就发生在更新操作之中。         

- 最短路径   
```ruby
for the weight of edge(u->v)
    if R[v] > R[u] + weight
        R[v] = R[u] + weight
```

- 最小生成树   
```ruby
for the weight of edge(u->v)
    if R[v] > weight
        R[v] = weight
```

其区别，从代码来看，显而易见。           