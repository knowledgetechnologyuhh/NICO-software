import logging
import Queue
import threading
import time

import cv2


class ImageWriter:
    """Multithreaded image writer for high resolution image processing"""

    def __init__(self, workers=2, write_enabled=True):
        self._logger = logging.getLogger(__name__)
        self._logger.debug("Initializing {}".format(__name__))
        self._queue = Queue.Queue()
        self._write_enabled = write_enabled
        self._open = False
        self._worker_threads = [None] * workers
        self.open()

    def open(self):
        """
        Initializes and starts all worker threads
        """
        if self._open:
            self._logger.warning("Writer is already open")
        else:
            self._logger.debug("Initializing worker threads")
            self._open = True
            for i in range(len(self._worker_threads)):
                worker = threading.Thread(target=self._worker_thread)
                worker.daemon = True
                worker.start()
                self._worker_threads[i] = worker

    def write_image(self, path, image):
        """
        Adds image to queue to be written to path by a worker thread
        """
        if not self._open:
            self._logger.warning(
                ("Image inserted while writer is closed - image will only " +
                 "be processed if the writer is reopened"))
        self._queue.put((path, image))

    def _worker_thread(self):
        while self._open:
            if self._write_enabled:
                try:
                    cv2.imwrite(*self._queue.get(timeout=1.))
                    self._queue.task_done()
                except Queue.Empty:
                    self._logger.debug("Image writing Queue empty")
                    continue

                # fn,dummy=self._queue.get()
                # import os
                # open(fn, 'a').close()

            else:
                time.sleep(0.1)

    def enable_write(self, state=True):
        self._write_enabled = state

    def close(self):
        """
        Finishes writing process and stops all worker threads
        """
        self._logger.info("Waiting for image writing tasks to finish")
        self._queue.join()
        self._open = False
        self._logger.debug("Waiting for workers to return")
        for worker in self._worker_threads:
            worker.join()
