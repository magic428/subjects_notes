# -*- coding: utf-8 -*-


class UrlTable:
    def __init__(self, urls):
        self.todo_list = []
        self.all_urls = {}
        for url in urls:
            self.all_urls[url] = 0

    def add_todo_list(self, ans):
        return self.todo_list.append(ans)

