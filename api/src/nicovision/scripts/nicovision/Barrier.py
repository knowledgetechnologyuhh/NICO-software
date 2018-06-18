import threading


class BrokenBarrierError(RuntimeError):
    pass


class Barrier:
    """Barrier for thread synchronization in python 2 similar to
    https://docs.python.org/3/library/threading.html#barrier-objects partially
    taken from https://greenteapress.com/wp/semaphores/ p.44."""

    def __init__(self, parties):
        self.parties = parties
        self.n_waiting = 0
        self._mutex = threading.Semaphore()
        self._barrier1 = threading.Semaphore(0)
        self._barrier2 = threading.Semaphore(0)
        self.broken = False

    def phase1(self):
        self._mutex.acquire()
        if self.broken:
            self._mutex.release()
            raise BrokenBarrierError()
        self.n_waiting += 1
        if self.n_waiting == self.parties:
            for _ in range(self.parties):
                self._barrier1.release()
        self._mutex.release()

        self._barrier1.acquire()

    def phase2(self):
        self._mutex.acquire()
        if self.broken:
            self._mutex.release()
            raise BrokenBarrierError()
        self.n_waiting -= 1
        if self.n_waiting == 0:
            for _ in range(self.parties):
                self._barrier2.release()
        self._mutex.release()

        self._barrier2.acquire()

    def wait(self):
        self.phase1()
        self.phase2()

    def abort(self):
        self._mutex.acquire()
        for _ in range(self.parties):
            self._barrier1.release()
            self._barrier2.release()
        self.broken = True
        self._mutex.release()

    def reset(self):
        self.abort()
        self._mutex.acquire()
        self.n_waiting = 0
        self._barrier1 = threading.Semaphore(0)
        self._barrier2 = threading.Semaphore(0)
        self.broken = False
        self._mutex.release()
