# 传智播客 day1 - 爬虫知识体系和 urllib2 库基本使用  

> 注意: 课程中使用的是 Python2, 关于 Python2 到 Python3 迁移过程中模块之间的变化会在下面给出.  

一、"大数据时代"，数据获取的方式：

1. 企业生产的用户数据：大型互联网公司有海量用户，所以他们积累数据有天然的优势。
                有数据意识的中小型企业，也开始积累的数据。

2. 数据管理咨询公司：通常这样的公司有很庞大的数据采集团队，一般会通过市场调研、问卷调查、固定的样本检测，
        和各行各业的公司进行合作、专家对话（数据积累很多年了，最后得出科研结果）来采集数据。

3. 政府/机构提供的公开数据：政府通过各地政府统计上报的数据进行合并；机构都是权威的第三方网站。

4. 第三方数据平台(艾瑞咨询集团)购买数据：通过各个数据交易平台来购买各行各业需要的数据，根据获取难度不同，价格也会不同。

5. 爬虫爬取数据：如果市场上没有我们需要的数据，或者价格太高不愿意买，那么就可以招/做一个爬虫工程师，从互联网上定向采集数据。

数据堂  贵阳大数据交易所   
   
新浪网页   

二、什么是爬虫？

爬虫：就是抓取网页数据的程序。


三、爬虫怎么抓取网页数据：

网页三大特征：  

-1. 网页都有自己唯一的URL（统一资源定位符）来进行定位
-2. 网页都使用 HTML （超文本标记语言）来描述页面信息。
-3. 网页都使用 HTTP/HTTPS（超文本传输协议）协议来传输 HTML 数据。

爬虫的设计思路：  

-1. 首先确定需要爬取的网页 URL 地址。
-2. 通过 HTTP/HTTPS 协议来获取对应的 HTML 页面。
-3. 提取 HTML 页面里有用的数据：
    a. 如果是需要的数据，就保存起来。
    b. 如果是页面里的其他 URL，那就继续执行第二步。


四、为什么选择Python做爬虫？

可以做爬虫的语言有很多，如 PHP、Java、C/C++、Python等等...

- PHP 虽然是世界上最好的语言，但是他天生不是干这个的，而且对多线程、异步支持不够好，并发处理能力很弱。
            爬虫是工具性程序，对速度和效率要求比较高。

- Java 的网络爬虫生态圈也很完善，是Python爬虫最大的对手。但是Java语言本身很笨重，代码量很大。
    重构成本比较高，任何修改都会导致代码的大量变动。爬虫经常需要修改部分采集代码。

- C/C++ 运行效率和性能几乎最强，但是学习成本很高，代码成型比较慢。
        能用C/C++做爬虫，只能说是能力的表现，但是不是正确的选择。


- Python 语法优美、代码简洁、开发效率高、支持的模块多，相关的HTTP请求模块和HTML解析模块非常丰富。
        还有强大的爬虫Scrapy，以及成熟高效的 scrapy-redis分布式策略。
        而且，调用其他借口也非常方便（胶水语言）


五、课程介绍：

-1. Python 的基本语法知识（已经搞定）

-2. 如何抓取 HTML 页面：
        HTTP 请求的处理，urllib、urllib2、requests  
        处理后的请求可以模拟浏览器发送请求，获取服务器响应的文件  

-3. 解析服务器响应的内容
        re、xpath、BeautifulSoup4（bs4）、jsonpath、pyquery等
        使用某种描述性语言来给我们需要提取的数据定义一个匹配规则，符合这个规则的数据就会被匹配。

-4. 如何采集动态HTML、验证码的处理
    通用的动态页面采集：Selenium + PhantomJS(无界面)：模拟真实浏览器加载 js、ajax 等非静态页面数据

    Tesseract：机器学习库，机器图像识别系统，可以处理简单的验证码，复杂的验证码可以通过手动输入/专门的打码平台

-5 Scrapy框架：（Scrapy，Pyspider）
    高定制性高性能（异步网络框架twisted），所以数据下载速度非常快，
    提供了数据存储、数据下载、提取规则等组件。

-6 分布式策略 scrapy-reids：
    scrapy-redis，在Scrapy的基础上添加了一套以 Redis 数据库为核心的组件。
        让Scrapy框架支持分布式的功能，主要在Redis里做 请求指纹去重、请求分配、数据临时存储。


-7 爬虫 - 反爬虫 - 反反爬虫 之间的斗争：
    其实爬虫做到最后，最头疼的不是复杂的页面，也是晦涩的数据，而是网站另一边的反爬虫人员。

    User-Agent、代理、验证码、动态数据加载、加密数据。

    数据价值，是否值的去费劲做反爬虫。

    1. 机器成本 + 人力成本 > 数据价值，就不反了，一般做到封IP就结束了。
    2. 面子的战争....

    爬虫和反爬虫之间的斗争，最后一定是爬虫获胜！
    为什么？只要是真实用户可以浏览的网页数据，爬虫就一定能爬下来！




六、根据使用场景：分为 通用爬虫  聚焦爬虫


1.通用爬虫：搜索引擎用的爬虫系统。

-1目标：就是尽可能把互联网上所有的网页下载下来，放到本地服务器里形成备份，
        再对这些网页做相关处理（提取关键字、去掉广告），最后提供一个用户检索接口。

-2抓取流程：
    a) 首选选取一部分已有的URL，把这些URL放到待爬取队列。
    b) 从队列里取出这些URL，然后解析DNS得到主机IP，然后去这个IP对应的服务器里下载HTML页面，保存到搜索引擎的本地服务器。
        之后把这个爬过的URL放入已爬取队列。
    c) 分析这些网页内容，找出网页里其他的URL连接，继续执行第二步，直到爬取条件结束。

-3 搜索引擎如何获取一个新网站的URL：
    1. 主动向搜索引擎提交网址：http://zhanzhang.baidu.com/linksubmit/url
    2. 在其他网站里设置网站的外链。
    3. 搜索引擎会和DNS服务商进行合作，可以快速收录新的网站。

    DNS：就是把域名解析成IP的一种技术。

-4 通用爬虫并不是万物皆可爬，它也需要遵守规则. Robots协议定义了爬虫的权限：
Robots协议：协议会指明通用爬虫可以爬取网页的权限。
Robots.txt 只是一个建议。并不是所有爬虫都遵守，一般只有大型的搜索引擎爬虫才会遵守。
    咱们个人写的爬虫，就不管了。


-5 通用爬虫工作流程：爬取网页 - 存储数据 - 内容处理 - 提供检索/排名服务

-6 搜索引擎排名：
    1. PageRank值：根据网站的流量（点击量/浏览量/人气）统计，流量越高，网站也越值钱，排名越靠前。
    2. 竞价排名：谁给钱多，谁排名就高。


-7 通用爬虫的缺点：
    1. 只能提供和文本相关的内容（HTML、Word、PDF）等等，但是不能提供多媒体文件（音乐、图片、视频）和二进制文件（程序、脚本）等等。
    2. 提供的结果千篇一律，不能针对不同背景领域的人提供不同的搜索结果。
    3. 不能理解人类语义上的检索。


为了解决这个问题，聚焦爬虫出现了：

聚焦爬虫：爬虫程序员写的针对某种内容的爬虫。
面向主题爬虫，面向需求爬虫：会针对某种特定的内容去爬取信息，而且会保证信息和需求尽可能相关。

chrome:
 Inspect -> Network, 替代 fiddler.   

Python自带的模块：/usr/lib/python2.7/urllib2.py
Python的第三方模块： /usr/local/lib/python2.7/site-packages

urllib2 默认的 User-Agent：Python-urllib/2.7

User-Agent: 是爬虫和反爬虫斗争的第一步，养成好习惯，发送请求带User-Agent




User-Agent 历史：

Mosaic 世界上第一个浏览器：美国国家计算机应用中心

Netscape 网景：Netscape（支持框架），慢慢开始流行....(第一款支持框架的浏览器)

Microsoft 微软：Internet Explorer（也支持框架）

第一次浏览器大战：网景公司失败..消失

Mozilla 基金组织：Firefox 火狐 - （Gecko内核）(第一款浏览器内核)

User-Agent 决定用户的浏览器，为了获取更好的HTML页面效果。

IE开了个好头，大家都开就给自己披着了个 Mozilla 的外皮

Microsoft公司：IE（Trident）

Opera公司：Opera（Presto）

Mozilla基金会：Firefox（Gecko）

Linux组织：KHTML （like Gecko）

Apple公司：Webkit（like KHTML）

Google公司：Chrome（like webkit）

其他浏览器都是IE/Chrome内核





## urllib2  

python3 中是 urllib.request(对应到).

urllib2 默认的 User-Agent："Python-urllib/2.7". 因此, 最好重构自己的 User-Agent.   

request.urlopen() 是无法构建 User-Agent 的, 可以使用 request.Request().  

~~~python
headers = {
    "User-Agent": "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/62.0.3202.94 Safari/537.36"
}
~~~

~~~html
GET / HTTP/1.1

Host: www.baidu.com
Connection: keep-alive

## 一定要写
User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/62.0.3202.94 Safari/537.36

Upgrade-Insecure-Requests: 1

Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,image/apng,*/*;q=0.8

# 一定不能写
Accept-Encoding: gzip, deflate, br

Accept-Language: zh-CN,zh;q=0.9,en-US;q=0.8,en;q=0.7

Cookie: BAIDUID=91F1FEAF79DC19B3F0706EE1FB0457D9:FG=1; PSTM=1530675548; BIDUPSID=33D2E3F7C694D9B8E8600CEF80C252F5; BDUSS=H5sMjFFaGV4ckJGRFp3eXc4TzZmQU5jUUM2eVBwa0pDS21SYVhUMWF3dVB6bU5iQUFBQUFBJCQAAAAAAAAAAAEAAAB0FgwPZ3VvemhpamllMjUAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAI9BPFuPQTxbb; ispeed_lsm=0; BDRCVFR[feWj1Vr5u3D]=I67x6TjHwwYf0; BD_CK_SAM=1; PSINO=1; BDORZ=B490B5EBF6F3CD402E515D22BCDA1598; BD_HOME=1; H_PS_PSSID=1448_21087_26350_20928; BD_UPN=123353; sug=3; sugstore=1; ORIGIN=0; bdime=0
~~~

User-Agent: 是爬虫和反爬虫斗争的第一步，养成好习惯，发送请求带User-Agent

response = request.urlopen(req) 中, response 是服务器响应的类文件，除了支持文件操作的方法外，还支持以下常用的方法：  

~~~python
# 返回 HTTP的响应码，成功返回200，4服务器页面出错，5服务器问题
print response.getcode()
# 返回 返回实际数据的实际URL，防止重定向问题
print response.geturl()
# 返回 服务器响应的HTTP报头
print response.info()
~~~


## url 参数

url 的第一个分隔符是 "?", 之后的每个分隔符是 "&".  

python3 中 urlencode 是在 urllib.parse.urlencode().  
python2 中 urlencode 是在 urllib.urlencode().  

urlencode() 接收的参数是一个字典：

~~~python
wd = {"wd" : "传智播客"}
urllib.urlencode(wd)
~~~

结果：wd=%E4%BC%A0%E6%99%BA%E6%92%AD%E5%AE%A2

- urllib 仅可以接受URL，不能创建 设置了headers 的Request 类实例；  
- 但是 urllib 提供 urlencode 方法用来GET查询字符串的产生，而 urllib2 则没有。（这是 urllib 和 urllib2 经常一起使用的主要原因）  
- 编码工作使用 urllib 的 urlencode() 函数，帮我们将 {key:value} 这样的键值对转换成 "key=value" 这样的字符串，解码工作可以使用 urllib 的 unquote() 函数。   

## Get 和 Post请求的区别  

Get : 请求的 url 会附带查询参数    
POST： 请求的 url 不带参数  
 
对于 Get 请求：查询参数在 QueryString 里保存;   
对于 Post 请求：查询参数在 Form 表单里保存;  

二者在代码中的区别就是, 构建 Request 的时候是否传递 data 参数(字典形式, 以逗号分割)).   

正则表达式做替换(subline-text):   

~~~regex
^(.*)=(.*)$
"\1": "\2"
~~~

## AJAX 动态加载数据  

做爬虫最需要关注的不是页面信息，而是页面信息的数据来源。  

AJAX 方式加载的页面，数据来源一定是 JSON. 拿到 JSON，就是拿到了网页的数据.   

直接抓包, 拿取 ajax 后边加载的 json 数据.  

## Handler 处理器 和 自定义 Opener

opener 是 urllib2.OpenerDirector 的实例，我们之前一直都在使用的 urlopen，它是一个特殊的 opener（也就是模块帮我们构建好的）。

但是基本的 urlopen() 方法不支持代理、cookie 等其他的 HTTP/HTTPS 高级功能。所以要支持这些功能：  

- 使用相关的 Handler 处理器来创建特定功能的处理器对象；  
- 然后通过 urllib2.build_opener()方法使用这些处理器对象，创建自定义opener 对象；  
- 使用自定义的 opener 对象，调用 open()方法发送请求。  
- 如果程序里所有的请求都使用自定义的 opener，可以使用 urllib2.install_opener() 将自定义的 opener 对象 定义为 全局 opener，表示如果之后凡是调用 urlopen，都将使用这个 opener（根据自己的需求来选择）  

这种方式发送请求得到的结果，和使用 urllib2.urlopen() 发送 HTTP/HTTPS 请求得到的结果是一样的。

如果在 `HTTPHandler()` 增加 `debuglevel=1` 参数，还会将 Debug Log 打开，这样程序在执行的时候，会把收包和发包的报头在屏幕上自动打印出来，方便调试，有时可以省去抓包的工作。 

~~~python
import os
from urllib import request

username = os.environ.get("username")
passwd = os.environ.get("passwd")

proxy_id = "{}:{}".format(username, passwd)

print(proxy_id)

# http_handler = request.ProxyHandler({"http" : "http://ahad-haam:3128"});
http_handler = request.ProxyHandler({"http" : proxy_id+"@http://ahad-haam:3128"});

opener = request.build_opener(http_handler)

req = request.Request("http://www.baidu.com")

response = opener.open(req)

print(response.read())
~~~


## 使用代理  

### 1. 公开代理 



### 2. 私密代理 

将代理的账号密码放在系统环境变量中取用, 这样可以做到隐藏账号信息...   

~~~python
username = os.environ.get("user_name")
passwd = os.environ.get("user_name")
~~~

另外一种使用方式:   

ProxyBasicAuthHandler(passwordMgr)  用于代理授权验证;   
HTTPBasicAuthHandler(passwordMgr)  用于 HTTP web 授权验证;   

二者使用的验证用户名和密码对象是一样的.   

~~~python
from urllib import request

test = "test"
password = "123456"
webserver = "192.168.21.52"

# 构建一个密码管理对象，可以用来保存和HTTP请求相关的授权账户信息
passwordMgr = request.HTTPPasswordMgrWithDefaultRealm()

# 添加授权账户信息，第一个参数realm如果没有指定就写None，后三个分别是站点IP，账户和密码
passwordMgr.add_password(None, webserver, test, password)
~~~

build_opener() 函数的参数是可以添加多个 handler 的, 比如既想使用 Http web 验证, 又想使用代理授权验证.   

## 免费短期代理网站.   

西刺免费代理 IP  
快代理免费代理   
proxy360 代理   
全网代理 IP  

IP 测试网站   

随机获取一个代理地址, random.choice()    



  
