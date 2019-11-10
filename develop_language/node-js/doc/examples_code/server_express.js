//引入 express 模块
var express = require("express")
var app = express()
var bodyParser = require("body-parser"); 
app.use(bodyParser.urlencoded({ extended: false }));  

//设置主机名
var hostName = '127.0.0.1';
//设置端口
var port = 8080;

//创建服务
app.all('*', function(req, res, next){
	res.header("Access-Control-Allow-Origin", "*");
	res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");
	res.header("Access-Control-Allow-Methods", "PUT, POST, GET, GET, DELETE, OPTIONS");
	res.header("X-Powered-By", "3.2.1");
	// res.header("Content-Type", "json/app.json;charset=utf-8")
    res.header('Content-Type','text/plain');

	// next();
    res.end("hello js");
});

app.get("/get",function(req, res){
	console.log("请求 url: ", req.path);
	console.log("请求参数: ", req.query);
	res.send("这个是 get 请求");
})

app.post("/post",function(req,res){
    console.log("请求参数：",req.body);
    var result = {code:200,msg:"post请求成功"};
    res.send(result);
});


app.listen(port, hostName, function(){
	console.log("server running...");
    // console.log("服务器运行在http://" + hostName+":"+port);

});