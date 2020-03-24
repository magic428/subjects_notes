arr1 = [[1,  2,  3,  4],
        [5,  6,  7,  8],
        [9,  10, 11, 12],
        [13, 14, 15, 16]]

for each_arr in arr1:
    for each_element in each_arr:
        print(each_element, end='\t')
    print()

tem_arr = []  # 用来记录数组值
rows = len(arr1)
cols = len(arr1[0])


def isValidIndex(x, n):
    # return (x < n)
    return (x >= 0 and x < n)
    # 每一行的每个值的数组下标的差都一样，

# solution 1
for i in range(cols * 2 - 1):  # 共输出 cols * 2 - 1 行
    diff = cols - i - 1  # 每一行的差
    for j in range(cols):  # 数组中每一个值的下标范围是0到cols
        k = j - diff  # 通过一个下标值计算另一个下标值(包含负数索引)
        if isValidIndex(k, rows):  # 剩下就是判断这些下标值是否满足当前的情况， 这一步不怎么好理解
            print(arr1[k][j], ' ', end='')
    print()

# direction 2
for i in range(cols * 2 - 1, 0, -1):  # 共输出 cols * 2 - 1 行
    print('www', i)
    diff = cols - i - 1    # 两个坐标之和
    for j in range(cols):  # 先控制第二个下标, 即列的范围是 0到cols
        k = (j - diff)     # 根据两个坐标之和计算第一个下标 
        if isValidIndex(k, rows):  # 剩下就是判断这些下标值是否满足当前的情况， 这一步不怎么好理解
            print(arr1[k][j], ' ', end='')
    print()

# # solution 2
# # 思路比较清晰: 如果两个点位于同一对角线上, 那么每个点的(x,y)坐标之和 x+y 是相等的.  
# for k in range(cols * 2 - 1): # 最大值为 (5,5)
#     # print('k loop', k)
#     for i in range(cols):
#         # print('i loop', i)
#         for j in range(cols):  # 数组中每一个值的下标范围是0到cols
#             # print('j loop', j)
#             if i+j == k and isValidIndex(i, rows):
#                 print(arr1[i][j], ' ', end='')
#             elif i+j < k:
#                 continue
#             else:
#                 break
            
#         # if i > k:
#         #     break;
#     print()

# print()

# # direction 2
# # 思路比较清晰: 如果两个点位于同一对角线上, 那么每个点的(x,y)坐标之和 x+y 是相等的.  
# for k in range(cols * 2 - 2, -1, -1): # 最大值为 (5,5)
#     # print('k loop', k)
#     for i in range(cols-1, -1, -1):
#         # print('i loop', i)
#         for j in range(cols-1, -1, -1):  # 数组中每一个值的下标范围是0到cols
#             # print('j loop', j)
#             if i+j == k and isValidIndex(i, rows):
#                 print(arr1[i][j], ' ', end='')
#             elif i+j > k:
#                 continue
#             else:
#                 break
            
#         # if i > k:
#         #     break;
#     print()
