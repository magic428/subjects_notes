# 传智播客 day3 - 多线程爬虫  


## 模拟登录知乎网   

_xsrf 跨域伪造服务请求, 请求成功后就会被服务器认为是合法请求.   

防止 CSRF 跨站请求伪造, 通常叫跨域攻击. 是一种利用网站对用户 cookie 的信任机制攻击网站. 

通过伪装成受信任的用户请求, 利用 cookie 来盗取用户信息, 欺骗 web 服务器.   

所以网站会通过设置一个隐藏字段来存放这个 MD5 字符串, 用来校验用户 cookie 和服务器 Session.  


比对 cookie 和 Session.  

cookie, Session 和 _xsrf 会一起被发送到服务器进行验证.   

requests.Session() 可以保存页面 cookie 的值.   

### 1. 获取 _xsrf   

~~~python
from bs4 import BeautifulSoup 
import urllib.requests
import time

def captcha(url):
    with open("captcha.jpg", "wb") af f:
        f.write(url)    
    return input("请输入验证码:")

def zhiHuLogin():

    sess = requests.Session()  # 保存 cookie

    headers = {}

    html = sess.get("https://www.zhihu.com/#signin", headers=headers).text

    bs = BeautifulSoup(html, "lxml")
    _xsrf = bs.find("input", attrs={"name":"_xsrf"}).get("value")

    captcha_url = "https://"{}.format(int(time.time()*1000)) 
    cha = captcha(captcha_url)

    data = {
        "_xsrf" : _xsrf,
        "email" : "xxx@qq.com",
        "password" : "password",
        "captcha" : cha
    }

    response = sess.post("https://www.zhihu.com/login/email", data=data, headers=headers)

    # 登录成功之后就可以访问网站的页面了   

    text = sess.get("https://www.zhihu.com/people/")

~~~

### 2. 验证码的图片编号是使用 UNIX 时间戳更新   

~~~python
import time

print(time.time())
print(time.ctime())

capcha_url = ""
~~~


**Note**: 淘宝网的登录相对于知乎网而言就相当复杂了.   

### 3. BeautifulSoup 模块的使用   


## JSON 解析  

json简单说就是javascript中的对象和数组，所以这两种结构就是对象和数组两种结构，通过这两种结构可以表示各种复杂的结构   

* 对象：对象在js中表示为{ }括起来的内容，数据结构为 { key：value, key：value, ... }的键值对的结构，在面向对象的语言中，key为对象的属性，value为对应的属性值，所以很容易理解，取值方法为 对象.key 获取属性值，这个属性值的类型可以是数字、字符串、数组、对象这几种。  

* 数组：数组在js中是中括号[ ]括起来的内容，数据结构为 ["Python", "javascript", "C++", ...]，取值方式和所有语言中一样，使用索引获取，字段值的类型可以是 数字、字符串、数组、对象几种。   

### 1. import json    

json模块提供了四个功能：dumps、dump、loads、load，用于字符串和 python 数据类型间进行转换。   


1) json.loads()   

把 Json 格式字符串解码转换成 Python 对象, 从json到python的类型转化对照如下：   

~~~python
import json

strList = '[1, 2, 3, 4]'

strDict = '{"city": "北京", "name": "大猫"}'

json.loads(strList) 
# [1, 2, 3, 4]

json.loads(strDict) # json数据自动按Unicode存储
# {u'city': u'\u5317\u4eac', u'name': u'\u5927\u732b'}
~~~

2. json.dumps()   

实现 python 类型转化为 json 字符串，返回一个 str 对象, 把一个 Python 对象编码转换成 Json 字符串.   

从 Python 原始类型向 Json 类型的转化对照如下：   


~~~python
# json_dumps.py

import json
import chardet

listStr = [1, 2, 3, 4]
tupleStr = (1, 2, 3, 4)
dictStr = {"city": "北京", "name": "大猫"}

json.dumps(listStr)
# '[1, 2, 3, 4]'
json.dumps(tupleStr)
# '[1, 2, 3, 4]'

# 注意：json.dumps() 序列化时默认使用的ascii编码
# 添加参数 ensure_ascii=False 禁用ascii编码，按utf-8编码
# chardet.detect()返回字典, 其中confidence是检测精确度

json.dumps(dictStr) 
# '{"city": "\\u5317\\u4eac", "name": "\\u5927\\u5218"}'

chardet.detect(json.dumps(dictStr))
# {'confidence': 1.0, 'encoding': 'ascii'}

print json.dumps(dictStr, ensure_ascii=False) 
# {"city": "北京", "name": "大刘"}

chardet.detect(json.dumps(dictStr, ensure_ascii=False))
# {'confidence': 0.99, 'encoding': 'utf-8'}
chardet是一个非常优秀的编码识别模块，可通过pip安装
~~~

3) json.dump()   

将 Python 内置类型序列化为 json 对象后写入文件.   

~~~python

# json_dump.py

import json

listStr = [{"city": "北京"}, {"name": "大刘"}]
json.dump(listStr, open("listStr.json","w"), ensure_ascii=False)

dictStr = {"city": "北京", "name": "大刘"}
json.dump(dictStr, open("dictStr.json","w"), ensure_ascii=False)
~~~


4. json.load()   

读取文件中 json 形式的字符串元素, 转化成 Python 类型. 

# json_load.py

import json

strList = json.load(open("listStr.json"))
print strList

# [{u'city': u'\u5317\u4eac'}, {u'name': u'\u5927\u5218'}]

strDict = json.load(open("dictStr.json"))
print strDict
# {u'city': u'\u5317\u4eac', u'name': u'\u5927\u5218'}
JsonPath
JsonPath 是一种信息抽取类库，是从JSON文档中抽取指定信息的工具，提供多种语言实现版本，包括：Javascript, Python， PHP 和 Java。

JsonPath 对于 JSON 来说，相当于 XPATH 对于 XML。

下载地址：https://pypi.python.org/pypi/jsonpath

安装方法：点击Download URL链接下载jsonpath，解压之后执行python setup.py install

官方文档：http://goessner.net/articles/JsonPath

JsonPath与XPath语法对比：
Json结构清晰，可读性高，复杂度低，非常容易匹配，下表中对应了XPath的用法。   


| XPath | JSONPath	| 描述 |
|:-------:|:-------:|:------|
| /	    | $  	    | 根节点 |
| .	    | @  	    | 现行节点 |
| /	    | .or  []	| 取子节点 |
| ..  	| null	    | 取父节点，Jsonpath 未支持 |
| //  	| ..	    | 就是不管位置，选择所有符合条件的条件 |
| *	    | *  	    | 匹配所有元素节点 |
| @	    | null	    | 根据属性访问，Json 不支持，因为 Json 是个 Key-value 递归结构，不需要 |
| []  	| []	    | 迭代器标示（可以在里边做简单的迭代操作，如数组下标，根据内容选值等） |
| `|`	| [,]  	    | 支持迭代器中做多选 |
| []  	| ?()	    | 支持过滤操作 |
| n/a  	| ()	    | 支持表达式计算 |
| ()  	| null	    | 分组，JsonPath 不支持 |


示例：   

我们以拉勾网城市 JSON 文件 http://www.lagou.com/lbs/getAllCitySearchLabels.json 为例，获取所有城市。

# jsonpath_lagou.py

import urllib2
import jsonpath
import json
import chardet

url = 'http://www.lagou.com/lbs/getAllCitySearchLabels.json'
request =urllib2.Request(url)
response = urllib2.urlopen(request)
html = response.read()

# 把 json 格式字符串转换成 python 对象  

jsonobj = json.loads(html)

# 从根节点开始，匹配 name 节点

citylist = jsonpath.jsonpath(jsonobj,'$..name')

print citylist
print type(citylist)
fp = open('city.json','w')

content = json.dumps(citylist, ensure_ascii=False)
print content

fp.write(content.encode('utf-8'))
fp.close()
注意事项：
json.loads() 是把 Json格式字符串解码转换成Python对象，如果在json.loads的时候出错，要注意被解码的Json字符的编码。

如果传入的字符串的编码不是UTF-8的话，需要指定字符编码的参数 encoding

dataDict = json.loads(jsonStrGBK);
dataJsonStr是JSON字符串，假设其编码本身是非UTF-8的话而是GBK 的，那么上述代码会导致出错，改为对应的：

  dataDict = json.loads(jsonStrGBK, encoding="GBK");
如果 dataJsonStr通过encoding指定了合适的编码，但是其中又包含了其他编码的字符，则需要先去将dataJsonStr转换为Unicode，然后再指定编码格式调用json.loads()

``` python
dataJsonStrUni = dataJsonStr.decode("GB2312"); dataDict = json.loads(dataJsonStrUni, encoding="GB2312");


##字符串编码转换

这是中国程序员最苦逼的地方，什么乱码之类的几乎都是由汉字引起的。
其实编码问题很好搞定，只要记住一点：

####任何平台的任何编码 都能和 Unicode 互相转换

UTF-8 与 GBK 互相转换，那就先把UTF-8转换成Unicode，再从Unicode转换成GBK，反之同理。



~~~python 
# 这是一个 UTF-8 编码的字符串
utf8Str = "你好地球"

# 1. 将 UTF-8 编码的字符串 转换成 Unicode 编码
unicodeStr = utf8Str.decode("UTF-8")

# 2. 再将 Unicode 编码格式字符串 转换成 GBK 编码
gbkData = unicodeStr.encode("GBK")

# 1. 再将 GBK 编码格式字符串 转化成 Unicode
unicodeStr = gbkData.decode("gbk")

# 2. 再将 Unicode 编码格式字符串转换成 UTF-8
utf8Str = unicodeStr.encode("UTF-8")
~~~

decode的作用是将其他编码的字符串转换成 Unicode 编码

encode的作用是将 Unicode 编码转换成其他编码的字符串

一句话：UTF-8 是对Unicode字符集进行编码的一种编码方式

~~~python

# Note

import json
import jsonpath

url = ""
headers = {}

req = request.Request(url, headers=headers) 
response = urllib.urlopen(req)

html = response.read()

text = json.loads(html)  # 把 json 格式的字符串转换为 Python 格式(Unicode).

# Python 形式的列表文件  
city_list = jsonpath.jsonpath(text, "$..name")

for item in city_list:
    print (item)

# Python 的 list 会转换为 json 的 array
# 默认会转换中文为 ascii 码格式, 因此加上 ensure_ascii=False 会保证返回的是Unicode字符串
array = json.dumps(city_list, ensure_ascii=False)

with open("file.json", "w"):
    f.write(array.encode("utf-8"))   # encode("utf-8") 也是必须的  

~~~

## Xpath  

**Note**: 使用 Xpath 进行解析的时候, 不同的网站服务器会根据不同的浏览器发送不同的请求结果, 因此最好使用 IE 浏览器的 User-Agent.   

Xpath 返回的结果是一个 list.  

模糊查询:   

> 以糗事百科为例.   

~~~xpath
//div[contains(@id,"qiushi_tag_")]
~~~

这样就可以作为根节点来获取其他的标签字段.   

~~~xpath
./div/a/@title)[0]                # 获取帖子发表的用户
.//div[@class="content"]/span     # 获取内容
.//span[@class="stats-vote"]/i    # 获取点赞数, 等价于.//i [0]
.//a[@class="qiushi-comments"]/i  # 获取评论数, 等价于.//i [1]
.//div[@class="thumb"]//@src      # 获取图片链接
 
~~~

## 糗事百科爬虫示例   


~~~python
import urllib
from lxml import etree
import json

url = ""
headers = {}

req = urllib.request.Request(url, headers=headers)
html = urllib.parse.urlopen(req).read()

text = etree.HTML(html)
node_list = text.xpath('//div[contains(@id, "qiushi_tag")]')

items = {}
for node in node_list:
    username = node.xpath('./div/a/@title')[0]
    # 不一定有图片, 因此不能直接使用索引
    image_url = node.xpath('.//div[@class="thumb"]//@src')
    thumb_up = node.xpath('.//i')[0].text() # text 用来取出标签里包含的内容
    content = node.xpath('.//div[@class="content"]/span')[0].text()
    comment = node.xpath('.//i')[1].text()
    items = {
        "username" : username,
        "image" : image_url,
        "thumb_up" : thumb_up,
        "content" : content,
        "comment" : comment
    }

    with open("qiushi.json", "w") as f:
        f.write(json.dumps(items, ensure_ascii=False).encode("utf-8")+"\n")
~~~

善用 xpath 的模糊查找.    


## 多线程爬虫   


GIL, 全局解释器锁, 相当于 Python 中的执行通行证, 控制的是 CPU 单元.   

Python 的多线程其实很弱, 但对于爬虫这种 IO(网络 IO 和磁盘 IO) 密集型操作还是很有用的.  

Python 的多进程适用于密集计算.   

队列 : put(), get(), empty(), full(), qsize().   

~~~python
import threading
import json
from lxml import etree
from Queue import Queue

class ThreadCrawl(threading.Thread):
    def __init__(self, threadName, pageQueue, dataQueue):
        # 调用父类初始化方法
        #threading.Thread.__init(self)
        super(ThreadCrawl, sels).__init__()  # 好处是: 多继承, 代码重构
        self.threadName = threadName
        self.pageQueue = pageQueue
        self.dataQueue = dataQueue
        self.headers = {}

    def run(self):
        while not CRAW_EXIT:
            # get() 方法有一个可选参数是 block   
            # 如果 block=True, 当队列为空时, 函数进入阻塞状态.
            # 如果 block=False, 当队列为空时, 函数会弹出一个队列为空的异常.
            try:     
                page = self.pageQueue.get(False)
                url = "http://www.qiushibaike.com/8hr/page/" + str(page) + "/"
                content = requests.get(url, headers=headers)
                self.dataQueue.put(content)

            except: 
                CRAW_EXIT = True
                print("pageQueue empty")
                pass

class ThreadParse(threading.Thread):
    def __init__(self, threadName, dataQueue, outputFile):
        super(ThreadParse, self).__init__()
        self.threadName = threadName
        self.dataQueue = dataQueue
        self.outputFile = outputFile 

    def run(self):
        while not PARSE_EXIT:
            try:
                html = self.dataQueue.get(Fales)

            except:
                pass

    def parse(self):
        etree.HTML(html)
        node_list = text.xpath('//div[contains(@id, "qiushi_tag")]')

        items = {}
        for node in node_list:
            username = node.xpath('./div/a/@title')[0]
            # 不一定有图片, 因此不能直接使用索引
            image_url = node.xpath('.//div[@class="thumb"]//@src')
            thumb_up = node.xpath('.//i')[0].text() # text 用来取出标签里包含的内容
            content = node.xpath('.//div[@class="content"]/span')[0].text()
            comment = node.xpath('.//i')[1].text()
            items = {
                "username" : username,
                "image" : image_url,
                "thumb_up" : thumb_up,
                "content" : content,
                "comment" : comment
            }

            with open("qiushi.json", "w") as f:
                f.write(json.dumps(items, ensure_ascii=False).encode("utf-8")+"\n")



global CRAW_EXIT
CRAW_EXIT = False
global PARSE_EXIT
PARSE_EXIT = False


def main():

    outputFile = "output.json"

    pageQueue = Queue(10) # 队列的大小  
    for i in range(1, 11):
        pageQueue.put(i)

    dataQueue = Queue()  # 队列大小为无限大, 存放爬取到的页面数据

    crawlList = ["getPage1", "getPage2", "getPage3"]  

    threadCrawl = []
    for threadName in crawlList:
        thread = ThreadCrawl(threadName, PageQueue, dataQueue)
        thread.start()  # 调用线程的 run 方法
        threadCrawl.append(thread)

    parseList = ["parseThread1", "parseThread2", "parseThread3"]

    threadParse = []
    for threadName in parseList:
        thread = ThreadParse(threadName, dataQueue, outputFile)
        thread.start()
        threadParse.append(thread)

    # 保证线程安全退出  
    for thread in threadCrawl:
        thread.join()
        print(thread.name+"joining...")   
    for thread in threadParse:
        thread.join()
        print(thread.name+"joining...")   
    

if __name__ == "__main__":
    
    main()
~~~


