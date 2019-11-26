# Qt 一步一步写出一个坦克大战游戏  



## 1. 




## 2. 



## 3. 



## 4.   

## 5. 限制玩家坦克框不能移动到屏幕外  

使用坦克框坐标和屏幕坐标系之间的关系, 保证 x 方向的坐标始终为正.  

## 6. 添加剩余游戏角色个数和得分  

使用 Heath 类表示剩余的游戏角色个数, 使用 Score 表示游戏得分.  

- 如果有任意一个 Enemy 碰到游戏窗边界, 那么游戏角色个数减一;  
- 如果 Bullet 和任意一个 Enemy 发生碰撞 (或重合), 那么得分加一;  

## 7. 

```cpp
Enemy::Enemy(QGraphicsItem *parent): QObject(), QGraphicsRectItem(parent){
```

说明会调用 QObject 的默认构造函数和 QGraphicsRectItem(QGraphicsItem *parent = Q_NULLPTR) 构造函数.  

