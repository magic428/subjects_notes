#coding=utf-8

from urllib.request import urlopen
from bs4 import BeautifulSoup
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
import csv

url = 'http://music.163.com/#/discover/playlist'

chrome_options = Options()
chrome_options.add_argument('--headless')
chrome_options.add_argument('--disable-gpu')
chrome_options.add_argument('--headless')

# driver = webdriver.PhantomJS()
driver = webdriver.Chrome(chrome_options=chrome_options)
# print (driver.title)

# Save Result to a .csv file
csv_file = open("/home/klm/playlist.csv", "w", newline = '')
writer = csv.writer(csv_file)
writer.writerow(['歌单名', '播放数', '链接'])

while url != "javascript:void(0)":
    driver.get(url)
    driver.switch_to.frame("contentFrame")
    data = driver.find_element_by_id("m-pl-container").\
        find_element_by_tag_name("li")

    # find the keyword of playtimes
    nb = data.find_element_by_class_name("nb").text 
    
    # pick the favorous playlists (> 500 万)
    if '万' in nb and int(nb.split("万")[0]) > 500:
        msk = data.find_element_by_css_selector("a.msk")
        writer.writerow([msk.get_attribute("title"), nb, msk.get_attribute("href")])

    url = driver.find_element_by_css_selector("a.zbtn.znxt").\
        get_attribute("href")
csv_file.close()