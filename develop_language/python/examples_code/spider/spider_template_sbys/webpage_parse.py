# -*- coding: utf-8 -*-

import re


class WebParse:
    """
    网页分析
    """
    def __init__(self, target_url):
        self.target_url = target_url
        self.cur_depth = 0

    def parse(self, host, response):
        """
        根据正则表达式提取URL
        """
        ans_list = []
        content = response.findAll('a')
        pat = re.compile(r'href="(%s)"' % self.target_url)
        pat2 = re.compile(r'http')
        for item in content:
            h = pat.search(str(item))

            if h is None:
                continue
            href = h.group(1)
            if pat2.search(href):
                ans = href
            else:
                if '/' in str(host[7:]):
                    ans = str(host[0:str(host).rindex('/')]) + '/' + href
                else:
                    ans = str(host) + '/' + href
            ans_list.append(ans)
        return ans_list
