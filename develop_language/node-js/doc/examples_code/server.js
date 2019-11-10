//引入http模块
var http = require("http");
var express = require("express")


//设置主机名
var hostName = '127.0.0.1';
//设置端口
var port = 8080;
//创建服务
var server = http.createServer(function(req,res){
    res.setHeader('Content-Type','text/plain');
    res.setHeader('Access-Control-Allow-Origin',"*")
    res.setHeader("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");  
    res.end("hello nodejs");

});
server.listen(port,hostName,function(){
    console.log("服务器运行在http://" + hostName+":"+port);

});