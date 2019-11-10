# -*- coding: utf-8 -*-

from configparser import ConfigParser


class SpiderConfig(object):
    """
    配置文件
    """
    def __init__(self):
        self.url_list_file = []
        self.output_directory = []
        self.max_depth = 0
        self.crawl_interval = 0
        self.crawl_timeout = 0.0
        self.target_url = []
        self.thread_count = 0
        self.urls = []

    def load_conf(self, conf_name='spider.conf'):
        """
        读取配置文件
        """
        config = ConfigParser()
        config.read(conf_name, encoding='utf-8')

        self.url_list_file = config.get('spider', 'url_list_file')
        self.output_directory = config.get('spider', 'output_directory')
        self.max_depth = int(config.get('spider', 'max_depth'))
        self.crawl_interval = int(config.get('spider', 'crawl_interval'))
        self.crawl_timeout = float(config.get('spider', 'crawl_timeout'))
        self.target_url = config.get('spider', 'target_url')
        self.thread_count = int(config.get('spider', 'thread_count'))

        f = open(self.url_list_file, 'r')
        for line in f:
            self.urls.append(line.split('\n')[0])
        f.close()

        print('url_list_file: ' + self.url_list_file)
        print('output_directory: ' + self.output_directory)
        print('max_depth: %d' % self.max_depth)
        print('crawl_interval: %d' % self.crawl_interval)
        print('crawl_timeout: %f' % self.crawl_timeout)
        print('target_url: ' + self.target_url)
        print('thread_count: %d' % self.thread_count)
        print('urls:')
        print(self.urls)

if __name__ == '__main__':
    conf = SpiderConfig()
    conf.load_conf()
