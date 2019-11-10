#_*_ coding:utf-8 _*_

# 爬取 https://mp.weixin.qq.com/s?__biz=MjM5OTYyNzAyMA==&mid=2649895342&idx=1&sn=495b4d9be84080ae29322b0859de5d1d&chksm=bf3e01cd884988db72750d6ef8280d44b7eea57f4f5ee18d6be1df13c2089ec0aa404198160b&mpshare=1&scene=1&srcid=0801QfpZ1nwyp2nXbNW9Ee9w&pass_ticket=Z7wLpqzbbmBdQ6VcmAorAEI6wvVmxI%2Fi1k5XATgGEBXu31%2FI054dUVvjEmg3ybNH#rd 
# 网页上的的图片  

import os
import requests
import time
import random
from lxml import etree

keyWord = input(r"{'Please input the keywords that you want to download :'}\n")

HOME = os.getenv("HOME")

savePath = HOME + '/work/spider/wx/'

class Spider():
    #初始化参数
    def __init__(self):
        #headers是请求头，"User-Agent"、"Accept"等字段都是通过谷歌Chrome浏览器查找的！
        self.headers = {
        "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_12_5) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.104 Safari/537.36",
        }
        #filePath是自定义的，本次程序运行后创建的文件夹路径，存放各种需要下载的对象。
        self.filePath = (savePath + keyWord + '/')

    def creat_File(self):
        #新建本地的文件夹路径，用于存储网页、图片等数据！
        filePath = self.filePath
        if not os.path.exists(filePath):
            os.makedirs(filePath)

    def get_pageNum(self):
        #用来获取搜索关键词得到的结果总页面数,用totalPagenum记录。由于数字是夹在形如：1,985 Wallpapers found for “dog”的string中，
        #所以需要用个小函数，提取字符串中的数字保存到列表numlist中，再逐个拼接成完整数字。。。
        totalPagenum = 0
        # url = ("https://alpha.wallhaven.cc/search?q={}&categories=111&purity=100&sorting=relevance&order=desc").format(keyWord)
        url = ("https://mp.weixin.qq.com/s?__biz=MjM5OTYyNzAyMA==&mid=2649895342&idx=1&sn=495b4d9be84080ae29322b0859de5d1d&chksm=bf3e01cd884988db72750d6ef8280d44b7eea57f4f5ee18d6be1df13c2089ec0aa404198160b&mpshare=1&scene=1&srcid=0801QfpZ1nwyp2nXbNW9Ee9w&pass_ticket=Z7wLpqzbbmBdQ6VcmAorAEI6wvVmxI%2Fi1k5XATgGEBXu31%2FI054dUVvjEmg3ybNH#rd")
        
        html = requests.get(url, headers=self.headers)
        selector = etree.HTML(html.text)
        # pageInfo = selector.xpath('//header[@class="listing-header"]/h1[1]/text()')
        # image_url_list = selector.xpath('//img[@data-src]')
        image_url_list = selector.xpath('//img[@data-copyright="0"]/@data-src')
        print(len(image_url_list))
        for i, image_url in enumerate(image_url_list):
            totalPagenum += 1
            image_name = "{:>05}.jpg".format(i)
            # print(help(image_url))
            print(image_name, image_url)
            image = requests.get(image_url).content
            # print (image)
            with open(self.filePath+image_name,'wb') as f:
                f.write(image)
                # f.write(bytes(image.encode("utf-8"), encoding="utf-8"))

        return totalPagenum

    def main_fuction(self):
        #count是总图片数，times是总页面数
        self.creat_File()
        count = self.get_pageNum()
        print("We have found:{} images!".format(count))
        # times = int(count/24 + 1)
        # j = 1
        # for i in range(times):
        #     pic_Urls = self.getLinks(i+1)
        #     for item in pic_Urls:
        #         self.download(item,j)
        #         j += 1

    def getLinks(self,number):
        #此函数可以获取给定numvber的页面中所有图片的链接，用 List 形式返回
        url = ("https://alpha.wallhaven.cc/search?q={}&categories=111&purity=100&sorting=relevance&order=desc&page={}").format(keyWord,number)
        try:
            html = requests.get(url)
            selector = etree.HTML(html.text)
            pic_Linklist = selector.xpath('//a[@class="jsAnchor thumb-tags-toggle tagged"]/@href')
        except Exception as e:
            print(repr(e))
        return pic_Linklist


    def download(self,url,count):
        #此函数用于图片下载。其中参数url是形如：https://alpha.wallhaven.cc/wallpaper/616442/thumbTags的网址
        #616442是图片编号，我们需要用strip()得到此编号，然后构造html，html是图片的最终直接下载网址。
        string = url.strip('/thumbTags').strip('https://alpha.wallhaven.cc/wallpaper/')
        html = 'http://wallpapers.wallhaven.cc/wallpapers/full/wallhaven-' + string + '.jpg'
        pic_path = (self.filePath + keyWord + str(count) + '.jpg' )
        try:
            pic = requests.get(html,headers = self.headers)
            f = open(pic_path,'wb')
            f.write(pic.content)
            f.close()
            print("Image:{} has been downloaded!".format(count))
            time.sleep(random.uniform(0,2))
        except Exception as e:
            print(repr(e))


spider = Spider()
spider.main_fuction()