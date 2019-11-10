# coding=utf-8
import cv2
import os
import requests
from lxml import etree

savePath = '/home/klm/work/spider/girl/'

def clean():
	files = os.listdir(savePath)
	files = [savePath+f  for f in files]

	for f in files:
		frame = cv2.imread(f)
		# print type(frame)
		if frame is None:
			os.remove(f)


def get_pageNum():
    #用来获取搜索关键词得到的结果总页面数,用totalPagenum记录。由于数字是夹在形如：1,985 Wallpapers found for “dog”的string中，
    #所以需要用个小函数，提取字符串中的数字保存到列表numlist中，再逐个拼接成完整数字。。。
    total = ""
    url = ("https://alpha.wallhaven.cc/search?q=dog&categories=111&purity=100&sorting=relevance&order=desc")
    
    html = requests.get(url)
    selector = etree.HTML(html.text)
    f = open("wallpaper.html", "w")
    f.write(html.text.encode('utf-8'))
    f.close()

    pageInfo = selector.xpath('//header[@class="listing-header"]/h1[1]/text()')
    string = str(pageInfo[0])
    print string
    numlist = list(filter(str.isdigit, string))
    for item in numlist:
        total += item
    totalPagenum = int(total)
    return totalPagenum

if __name__ == '__main__':
	get_pageNum()