# JQuery安裝
## 1. 网页中添加 jQuery   
可以通过多种方法在网页中添加 jQuery。 您可以使用以下方法：   
- 从 [jquery.com](http://jquery.com/download/) 下载 jQuery 库   
- 从 CDN 中载入 jQuery, 如从 Google 中加载 jQuery   

## 2. 下载 jQuery   
有两个版本的 jQuery 可供下载：     
- Production version - 用于实际的网站中，已被精简和压缩。   
- Development version - 用于测试和开发（未压缩，是可读的代码）   
以上两个版本都可以从 [jquery.com](http://jquery.com/download/)下载.     

## 3. 快捷安装    
```bash
sudo apt-get install npm
npm install jquery
```
jQuery 库是一个 JavaScript 文件，您可以使用 HTML 的 <script> 标签引用它：   

<head><script
src="jquery-1.10.2.min.js"></script></head>
提示： 将下载的文件放在网页的同一目录下，就可以使用jQuery。
lamp	您是否很疑惑为什么我们没有在 <script> 标签中使用 type="text/javascript" ？

在 HTML5 中，不必那样做了。JavaScript 是 HTML5 以及所有现代浏览器中的默认脚本语言！
替代方案
如果您不希望下载并存放 jQuery，那么也可以通过 CDN（内容分发网络） 引用它。
百度、又拍云、新浪、谷歌和微软的服务器都存有 jQuery 。
如果你的站点用户是国内的，建议使用百度、又拍云、新浪等国内CDN地址，如果你站点用户是国外的可以使用谷歌和微软。
注：本站实例均采用菜鸟教程 CDN 库。
如需从菜鸟教程、又拍云、新浪、谷歌或微软引用 jQuery，请使用以下代码之一：
菜鸟教程 CDN:

<head><script
src="http://cdn.static.runoob.com/libs/jquery/1.10.2/jquery.min.js"></script></head>



百度 CDN:

<head><script
src="https://apps.bdimg.com/libs/jquery/2.1.4/jquery.min.js"></script></head>



又拍云 CDN:

<head><script
src="http://upcdn.b0.upaiyun.com/libs/jquery/jquery-2.0.2.min.js"></script></head>



新浪 CDN:

<head><script
src="http://lib.sinaapp.com/js/jquery/2.0.2/jquery-2.0.2.min.js"></script></head>



Google CDN:

<head><script
src="http://ajax.googleapis.com/ajax/libs/jquery/1.10.2/jquery.min.js"></script></head>



lamp	不大推荐使用Google CDN来获取版本，因为Google产品在中国很不稳定。
Microsoft CDN:

<head><script
src="http://ajax.htmlnetcdn.com/ajax/jQuery/jquery-1.10.2.min.js"></script></head>



lamp	使用百度、又拍云、新浪、谷歌或微软的 jQuery，有一个很大的优势：

许多用户在访问其他站点时，已经从百度、又拍云、新浪、谷歌或微软加载过 jQuery。所有结果是，当他们访问您的站点时，会从缓存中加载 jQuery，这样可以减少加载时间。同时，大多数 CDN 都可以确保当用户向其请求文件时，会从离用户最近的服务器上返回响应，这样也可以提高加载速度。
//文章由菜鳥教程而來。