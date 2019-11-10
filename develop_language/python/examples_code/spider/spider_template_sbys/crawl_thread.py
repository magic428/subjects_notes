# -*- coding: utf-8 -*-

import threading
import queue
import urllib.request
import urllib.error
import logging
from bs4 import BeautifulSoup
from bs4.element import ContentMetaAttributeValue
import url_table
import config_load

lock = threading.Lock()
LOG_FILENAME = "log.txt"
logging.basicConfig(filename=LOG_FILENAME, level=logging.NOTSET)


class CrawlClass(threading.Thread):
    """
    爬虫线程
    """
    def __init__(self, queue, u_table, conf, web_save, web_parse):
        threading.Thread.__init__(self)
        self.queue = queue
        self.u_table = u_table
        self.config = conf
        self.web_save = web_save
        self.web_parse = web_parse

    def run(self):
        """
        URL打开, 保存, 分析
        """
        while True:
            # 从队列中获取一个URL
            host = self.queue.get()
            try:
                url = urllib.request.urlopen(host, data=None, timeout=self.config.crawl_timeout)
                # 分析字符编码, 并以该编码读取网页
                charset = ContentMetaAttributeValue.CHARSET_RE.search(url.headers['content-type'])
                charset = charset and charset.group(3) or None
                response = BeautifulSoup(url.read(), "html.parser", from_encoding=charset)
            except urllib.error.HTTPError as e:
                logging.debug("Exception: %s" % e.code)
                continue
            except urllib.error.URLError as e:
                logging.debug("Exception: %s" % e.reason)
                continue
            except Exception as e:
                logging.debug("Exception: %s" % e)
                continue
            finally:
                # 标记队列工作已完成
                self.queue.task_done()

            # 保存网页
            self.web_save.save(host, response, threading.current_thread().getName())

            # 如果当前网页不是最大深度, 则分析该网页, 提取urls
            if self.web_parse.cur_depth < self.config.max_depth:
                ans_list = self.web_parse.parse(host, response)
                for ans in ans_list:
                    self.add_url(ans)

    def add_url(self, ans):
        """
        如果地址不与已有的重复, 则添加到todo_list
        """
        if lock.acquire():
            if ans not in self.u_table.all_urls:
                self.u_table.all_urls[ans] = 0
                self.u_table.add_todo_list(ans)
            else:
                logging.debug("Duplicated url: %s" % ans)
            lock.release()
        else:
            logging.debug("Lock error")

if __name__ == '__main__':
    conf = config_load.SpiderConfig()
    conf.load_conf()
    queue = queue.Queue()
    u_table = url_table.UrlTable()

    th = CrawlClass(queue)
    th.u_table = u_table
    th.config = conf
    th.setDaemon(True)
    th.start()

    queue.put(conf.urls[0])
    queue.join()
    print(th.u_table.todo_list)
