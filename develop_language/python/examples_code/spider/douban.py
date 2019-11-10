#coding=utf-8

import os
import sys
import urllib
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.keys import Keys
from lxml import etree
import requests
import time

home_dir = os.getenv("HOME")
save_dir = "snapshots/" 
print(save_dir)

classes = ["名著", "历史", "经济学", "理财"]

headers = {
        "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_12_5) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.104 Safari/537.36",
        }

if not os.path.exists(save_dir):
    os.makedirs(save_dir)

def check_list_length(xpath_list):
    if len(xpath_list) > 0:
        return xpath_list[0]

def get_book_introduce(introduce_url):
    
    intr_content = requests.get(introduce_url, headers=headers).text
    selector = etree.HTML(intr_content)
    intros = selector.xpath('//div[@class="indent"]//div[@class]/div[@class="intro"]/p')
    author_infos = selector.xpath('//div[@class="indent "]/span[@class="all hidden "]/div[@class="intro"]/p')
    text = []
    
    intro_text = ''
    for intro in intros:
        # print (intro.text)
        intro_text += intro.text
        intro_text += "\n"
    text.append(intro_text)
    
    author_info_text = ''
    for author_info in author_infos:
        author_info_text += author_info.text
        author_info_text += '\n'
    text.append(author_info_text)

    return text


def save_img(image_url):
    image_name = "{}".format(image_url[-9:])
    with open(save_dir + image_name, "wb") as f:
        content = requests.get(image_url, headers=headers).content
        f.write(content)


# headers = {'User-Agent': "Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/66.0.3359.139 Safari/537.36"}



chrome_options = Options()
chrome_options.add_argument('--headless')
chrome_options.add_argument('--disable-gpu')
driver = webdriver.Chrome(chrome_options=chrome_options)

url = "https://book.douban.com/tag/"

with open("booklist.md", "a") as f:
    f.write("# 豆瓣书单  \n\n")


for c in classes:
    book_num = 1
    with open("booklist.md", "a") as f:
        f.write("## {}  \n\n".format(c))
    kw = urllib.parse.quote(c)

    for i in range(40):
        
        start = i*20
        driver.get(url + kw + "?start={}".format(start))
        time.sleep(3)

        selector = etree.HTML(driver.page_source)
        subject_item = selector.xpath('//li[contains(@class,"subject-item")]')
        # subject_item = selector.xpath('//li[@class="subject-item"]')
        for item in subject_item:
            try:
                book_rating = check_list_length(item.xpath('.//span[@class="rating_nums"]')).text

                if float(book_rating) > 8.9:

                    book_introduce = check_list_length(item.xpath('.//a[@class="nbg"]/@href'))
                    book_cover_url = check_list_length(item.xpath('.//img/@src'))
                    save_img(book_cover_url)
                    image_name = "{}".format(book_cover_url[-9:])

                    book_name = check_list_length(item.xpath('.//a/@title'))
                    # print("{}: {}, {}".format(book_name, book_rating, get_book_introduce(book_introduce)))
                    
                    with open("booklist.md", "a") as f:
                        f.write("\n{}) {}: {}  \n".format(book_num, book_name, book_rating))
                        f.write("\n![](./snapshots/{})  \n".format(image_name))
                        f.write("\n**内容简介**:  \n\n{}".format(get_book_introduce(book_introduce)[0]))
                        f.write("\n**作者简介**:  \n\n{}".format(get_book_introduce(book_introduce)[1]))
                    
                    print("{}: 第{:>04}本书下载完毕...".format(c, book_num))
                    book_num +=1 
            except:
                book_num +=1
                pass