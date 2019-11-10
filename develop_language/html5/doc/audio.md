# HTML 5 音频   

HTML5 规定了一种通过 audio 元素来包含音频的标准方法。audio 元素能够播放声音文件或者音频流。  

## 音频格式

当前, audio 元素支持三种音频格式：   

            IE 9	Firefox 3.5	Opera 10.5	Chrome 3.0	Safari 3.0
Ogg Vorbis  	 	√	√	√	 
MP3	√	 	 	√	√
Wav	 	√	√	 	√

## <audio> 标签的属性

属性	值	描述
autoplay	autoplay	如果出现该属性，则音频在就绪后马上播放。
controls	controls	如果出现该属性，则向用户显示控件，比如播放按钮。
loop	loop	如果出现该属性，则每当音频结束时重新开始播放。
preload	preload	
如果出现该属性，则音频在页面加载时进行加载，并预备播放。
如果使用 "autoplay"，则忽略该属性。
src	url	要播放的音频的 URL。


## 实例  

control 属性供添加播放、暂停和音量控件。   

audio 元素允许多个 source 元素。source 元素可以链接不同的音频文件。浏览器将使用第一个可识别的格式.   

~~~html
<audio controls="controls">
  <source src="song.ogg" type="audio/ogg">
  <source src="song.mp3" type="audio/mpeg">
Your browser does not support the audio tag. // 之间插入的内容是供不支持 audio 元素的浏览器显示的
</audio>
~~~