import logging
import Queue
import cv2
import atexit
import threading


class ImageWriter:
    """Multithreaded image writer for high resolution image processing"""
    def __init__(self, workers=2):
        self._queue = Queue.Queue()
        self._workers = workers
        worker = threading.Thread(target=self._worker_thread)
        worker.daemon = True
        worker.start()
        atexit.register(self._close)

    def write_image(self, path, image):
        self._queue.put((path, image))

    def _worker_thread(self):
        while True:
            cv2.imwrite(*self._queue.get())
            self._queue.task_done()

    def _close(self):
        logging.info("Waiting for image writing tasks to finish")
        self._queue.join()
