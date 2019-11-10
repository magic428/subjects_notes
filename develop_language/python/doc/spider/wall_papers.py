
# coding: utf-8

# # python 爬虫相关
# ## 1. class 定义和使用

# In[11]:


import os
import requests
import time
import random
from lxml import etree


class Spider(object):
    def __init__(self, savePath, keyWord):
        self.headers = {
            "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_12_5) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.104 Safari/537.36",
        }
        self.keyWord = keyWord
        self.filePath = (savePath + keyWord + '/')
    def createFile(self):
        filePath = self.filePath
        if not os.path.exists(filePath):
            os.makedirs(filePath)
    def getPageNum(self):
        #用来获取搜索关键词得到的结果总页面数,用totalPagenum记录。由于数字是夹在形如：1,985 Wallpapers found for “dog”的string中，
        #所以需要用个小函数，提取字符串中的数字保存到列表numlist中，再逐个拼接成完整数字。。。
        total = ""
        url = ("https://alpha.wallhaven.cc/search?q={}&categories=111&purity=100&sorting=relevance&order=desc").format(self.keyWord)
        html = requests.get(url)
        selector = etree.HTML(html.text)
        pageInfo = selector.xpath('//header[@class="listing-header"]/h1[1]/text()')
        string = str(pageInfo[0])
        numList = list(filter(str.isdigit, string)) 
        for item in numList:
            total += item
        totalPageNum = int(total)
        return totalPageNum
    
    def main_func(self):
        count = self.getPageNum()
        print("We have found:{} images!".format(count))


# In[13]:


s = Spider("/home/klm/work/spider/", "girl")
print s.headers, s.filePath
s.main_func()
