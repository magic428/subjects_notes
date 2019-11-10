# -*- coding: utf-8 -*-

import time
import queue
import copy
from argparse import ArgumentParser
import crawl_thread
import url_table
import config_load
import webpage_parse
import webpage_save


def main():
    """
    爬虫入口
    """
    p = ArgumentParser()
    p.add_argument('-v', action='version', version='1.0', help='version')
    p.add_argument('-c', default='spider.conf', help='config name')
    args = p.parse_args()

    conf = config_load.SpiderConfig()
    conf.load_conf(args.c)
    hosts = copy.deepcopy(conf.urls)
    hosts = list(set(hosts))
    u_table = url_table.UrlTable(hosts)
    web_save = webpage_save.WebSave(conf.output_directory)
    web_parse = webpage_parse.WebParse(conf.target_url)

    # 创建队列实例
    url_queue = queue.Queue()
    # 生成一个线程池
    for i in range(conf.thread_count):
        t = crawl_thread.CrawlClass(url_queue, u_table, conf, web_save, web_parse)
        # 主程序退出时，子线程也立即退出
        t.setDaemon(True)
        # 启动线程
        t.start()

    # 向队列中填充URLs
    cur_depth = 0
    depth = conf.max_depth
    while cur_depth <= depth:
        for host in hosts:
            url_queue.put(host)
            time.sleep(conf.crawl_interval)
        cur_depth += 1
        web_parse.cur_depth = cur_depth
        url_queue.join()
        hosts = copy.deepcopy(u_table.todo_list)
        u_table.todo_list = []

if __name__ == '__main__':
    st = time.time()
    main()
    print('%f' % (time.time() - st))
