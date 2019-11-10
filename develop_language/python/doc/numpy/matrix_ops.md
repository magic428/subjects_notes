# numpy 中的矩阵操作   

## np.squeeze(a, axis=None)

移除张量 shape 中值为 1 的维度.  
 
Parameters
----------
axis : 可选参数, None, 整型或整型元组. 

    选择 shape 中维度为 1 的一个子集. 如果选择的子集中某个维度大于 1, 将会报错.

Returns
-------
squeezed :  维度被压缩之后的张量.   


