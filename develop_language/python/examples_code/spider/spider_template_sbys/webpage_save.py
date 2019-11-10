# -*- coding: utf-8 -*-

import logging
import time

ISO_TIME_FORMAT = '%Y-%m-%d %X'
LOG_FILENAME = "log.txt"
logging.basicConfig(filename=LOG_FILENAME, level=logging.NOTSET)


class WebSave:
    """
    网页保存
    """
    def __init__(self, output_dir):
        self.output_dir = output_dir

    def save(self, host, response, thread_name):
        """
        保存指定的网页
        """
        url_name = str(host).split('//')[1]
        url_name = url_name.replace('/', '-')
        filename = self.output_dir + '/' + url_name
        logging.debug("%s - %s - File saved: %s" %
                      (time.strftime(ISO_TIME_FORMAT, time.localtime()), thread_name, filename))

        try:
            saved_file = open(filename, 'w', encoding='utf-8')
            saved_file.write(str(response))
            saved_file.close()
        except IOError as e:
            logging.debug("IOError: %s" % e)

