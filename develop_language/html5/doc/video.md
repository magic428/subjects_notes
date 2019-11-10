# html 中的 video 元素   

实例
<video width="320" height="240" controls="controls">
  <source src="movie.ogg" type="video/ogg">
  <source src="movie.mp4" type="video/mp4">
Your browser does not support the video tag.
</video>
TIY
Internet Explorer
Internet Explorer 8 不支持 video 元素。在 IE 9 中，将提供对使用 MPEG4 的 video 元素的支持。

## <video> 标签的属性   

属性	值	描述
autoplay	autoplay	如果出现该属性，则视频在就绪后马上播放。
controls	controls	如果出现该属性，则向用户显示控件，比如播放按钮。
height	pixels	设置视频播放器的高度。
loop	loop	如果出现该属性，则当媒介文件完成播放后再次开始播放。
preload	preload	
如果出现该属性，则视频在页面加载时进行加载，并预备播放。
如果使用 "autoplay"，则忽略该属性。
src	url	要播放的视频的 URL。
width	pixels	设置视频播放器的宽度。

## video 支持的方法.  

HTML5 <video> 元素同样拥有方法、属性和事件。其方法用于播放、暂停以及加载等。其属性（比如时长、音量等）可以被读取或设置。其 DOM 事件能够通知您，比方说，<video> 元素开始播放、已暂停，已停止，等等。  

下面列出了大多数浏览器支持的视频方法、属性和事件：
方法	属性	事件
play()	currentSrc	play
pause()	currentTime	pause
load()	videoWidth	progress
canPlayType	videoHeight	error
 	duration	timeupdate
 	ended	ended
 	error	abort
 	paused	empty
 	muted	emptied
 	seeking	waiting
 	volume	loadedmetadata
 	height	 
 	width	

## 实例  

### 1. 使用 <video> 标签   

~~~html
<!DOCTYPE HTML>
<html>
<body>

<video src="movie.ogg"  width="320" height="240" autoplay="autoplay" loop="loop" controls="controls">
Your browser does not support the video tag.
</video>

</body>
</html>
~~~

### 2. 使用 <video> 元素的方法

~~~html
<!DOCTYPE html> 
<html> 
<body> 

    <div style="text-align:center;">
        <button onclick="playPause()" type="button"> play/pause </button> 
        <button onclick="makeBig()" type="button"> big Window </button>
        <button onclick="makeNormal()" type="button"> middle </button>
        <button onclick="makeSmall()" type="button"> small </button>
        <br /> 
        <video id="video_1" width="420" style="margin-top:15px;" 
            <source src="mov_bbb.mp4" type="video/mp4" />
            <source src="mov_bbb.ogg" type="video/ogg" />
            Your browser does not support HTML5 video.
        </video>
    </div> 

    <script type="text/javascript">
    var myVideo=document.getElementById("video_1");

        // 上面的例子调用了两个方法：play() 和 pause()。
        // 它同时使用了两个属性：paused 和 width。
        function playPause() { 
            if (myVideo.paused) 
                myVideo.play();    
            else 
                myVideo.pause(); 
        } 

        function makeBig() { 
            myVideo.width=560; 
        } 

        function makeSmall() { 
            myVideo.width=320; 
        } 

        function makeNormal() { 
            myVideo.width=420; 
        } 
    </script> 
</body> 
</html>
~~~