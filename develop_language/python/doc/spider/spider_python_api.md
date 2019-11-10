# 爬虫相关的　Python API 整理  

> 环境: Python3  

## urllib / request

注意两个方法: text() 和 content() 分别是获取文本页面和二进制页面.   

~~~python
import requests


headers = { 
    "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_12_5) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.104 Safari/537.36",
}

url = ("https://www.xxx.com/s?")

# 获取网页的文本内容
html = requests.get(url).text 

# 获取网页的二进制内容, 比如图片链接的内容
html = requests.get(url).content 
~~~

## lxml 和 xpath

使用 etree.HTML(html) 将网页内容转换为 xml 格式后就可以使用 xpath 进行解析了.  

~~~python
from lxml import etree

# html 为网页的文本内容
selector = etree.HTML(html)

# 获取网页中的图片链接, 返回值类型是 list  
image_url_list = selector.xpath('//img[@data-copyright="0"]/@data-src')

# 获取网页中页面信息
page_info = selector.xpath('//header[@class="listing-header"]/h1[1]/text()')

# xpath 获取到的文本内容可能需要转码  
for i, image_url in enumerate(image_url_list):
    image_url = image_url.encode('utf-8')
~~~

## BeautifulSoup  

~~~python
from bs4 import BeautifulSoup as bs

~~~


## json   

~~~python
import json
~~~

1) json.loads()   

2) json.dumps()   

3) json.dump()   

4) json.load()   



## 队列 

~~~bash
from Queue import Queue

put(), get(), empty(), full(), qsize().   
~~~


## thread

~~~python
import threading

threading.Thread
~~~


## Selenium 和 Headless Chrome   

~~~python
from selenium import webdriver
from selenium.webdriver.common.keys import keys

# Deprecated, 已经不在继续支持 PhantomJS 
# driver = webdriver.PhantomJS()

driver.get("https://mp.weixin.qq.com/s?d")

print(driver.page_source)
driver.page_source.find("shark-pager-disable-next")
driver.find_element_by_name("form_email").send_keys("xxx_@qq.com")
driver.find_element_by_class_name("bn-submit").click()
driver.find_element_by_id("captcha_field").send_keys("xxxxxx")
driver.save_screenshot("douban.png")

driver.quit()
~~~

## 执行 JavaScript 语句  

~~~python
# 向下滚动 10000 像素
js = "document.body.scrollTop=10000"
driver.execute_script(js)
~~~

## selenium + chrome

~~~python

import os
import urllib
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.keys import Keys
from lxml import etree
import requests

headers = {
        "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_12_5) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.104 Safari/537.36",
        }

# headers = {'User-Agent': "Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/66.0.3359.139 Safari/537.36"}

home_dir = os.getenv("HOME")
save_dir = "{}/work/spider/miners/".format(home_dir) 
print(save_dir)

if not os.path.exists(save_dir):
    os.makedirs(save_dir)

chrome_options = Options()
chrome_options.add_argument('--headless')
chrome_options.add_argument('--disable-gpu')
driver = webdriver.Chrome(chrome_options=chrome_options)

url = "https://image.baidu.com/search/index?tn=baiduimage&word="

kw = "矿工"
kw = urllib.parse.quote(kw)

driver.get(url + kw)
selector = etree.HTML(driver.page_source)
~~~

问题:  

~~~
Traceback (most recent call last):
  File "miners.py", line 51, in <module>
    driver.execute_script("window.scrollTo(0,document.body.scrollHeight);")
  File "/usr/local/lib/python3.4/dist-packages/selenium/webdriver/remote/webdriver.py", line 635, in execute_script
    'args': converted_args})['value']
  File "/usr/local/lib/python3.4/dist-packages/selenium/webdriver/remote/webdriver.py", line 320, in execute
    self.error_handler.check_response(response)
  File "/usr/local/lib/python3.4/dist-packages/selenium/webdriver/remote/errorhandler.py", line 242, in check_response
    raise exception_class(message, screen, stacktrace)
selenium.common.exceptions.WebDriverException: Message: unknown error: call function result missing 'value'
  (Session info: chrome=68.0.3440.75)
  (Driver info: chromedriver=2.26.436382 (70eb799287ce4c2208441fc057053a5b07ceabac),platform=Linux 4.4.0-131-generic x86_64)
~~~

版本问题, 换个新点的版本就可以了, chromedriver=2.36.540471 .  


## 保存到本地的文件命名   

~~~python

for i in range(100):
    image_name = "{:>05}.jpg".format(i)

~~~


## Python 测试模块  

三个重要的函数必须被重写, 其中 testDouYu() 函数的开头字符串必须是 test.  

~~~python
import unittext

class DouYu(unittext.TestCase):
    def setUp(self):
        self.driver = webdriver.PhantomJS()    
        self.num = 0
        self.count = 0

    def testDouYu(self):
        self.driver.get("https://www.douyu.com/directory/all")

        while True:
            soup = bs(self.driver.page_source, "lxml")
            names = soup.find_all("h3", "class":"ellipsis")
            numbers = soup.find_all("span", "class":"du-num fr")

            for name, number in zip(names, numbers):
                print("观众人数: " + number.get_text().strip() + "\t 房间名: " + name.get_text.strip())
                self.num += 1
                self.count += int(number.get_text().strip())
            
            if self.driver.page_source.find("shark-pager-disable-next") != -1:
                break

            self.driver.find_element_by_class_name("shark-pager-next").click()

    def tearDown(self):
        print(self.num)
        p rint(self.count)
        self.driver.quit()

if __name__ == "__main__":
    
    # 启动测试模块  
    unittest.main()
~~~


## 参考资料   

[1. Requests: 让 HTTP 服务人类]: http://docs.python-requests.org/zh_CN/latest/   
[2. Beautiful Soup 4.2.0 文档]: https://www.crummy.com/software/BeautifulSoup/bs4/doc/index.zh.html   
[3. Beautiful Soup 4.2.0 文档]: http://deerchao.net/tutorials/regex/regex.htm   
[4. 在线正则表达式测试]: http://tool.oschina.net/regex/   