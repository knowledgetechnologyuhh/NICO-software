import atexit
import logging
import Queue
import threading

import cv2

import time


class ImageWriter:
    """Multithreaded image writer for high resolution image processing"""

    def __init__(self, workers=2,write_enabled=True):
        self._queue = Queue.Queue()
        self._write_enabled=write_enabled
        for _ in range(workers):
                worker = threading.Thread(target=self._worker_thread)
                worker.daemon = True
                worker.start()
        atexit.register(self._close)

    def write_image(self, path, image):
        self._queue.put((path, image))

    def _worker_thread(self):
        while True:
            if self._write_enabled:
                cv2.imwrite(*self._queue.get())

                # fn,dummy=self._queue.get()
                # import os
                # open(fn, 'a').close()

                self._queue.task_done()
            else:
                time.sleep(0.1)

    def enable_write(self,state=True):
        self._write_enabled=state

    def _close(self):
        logging.info("Waiting for image writing tasks to finish")
        self._queue.join()
