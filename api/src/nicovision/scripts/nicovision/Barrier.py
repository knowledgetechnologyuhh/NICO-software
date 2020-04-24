import threading


class BrokenBarrierError(RuntimeError):
    pass


class Barrier:
    """Barrier for thread synchronization in python 2 similar to
    https://docs.python.org/3/library/threading.html#barrier-objects partially
    taken from https://greenteapress.com/wp/semaphores/ p.44."""

    def __init__(self, parties):
        """
        Initializes barrier that has to be called by given number of threads in
        order for all of them to proceed.
        """
        self.parties = parties
        self.n_waiting = 0
        self._mutex = threading.Semaphore()
        self._barrier1 = threading.Semaphore(0)
        self._barrier2 = threading.Semaphore(0)
        self.broken = False

    def phase1(self):
        """
        First phase of the barrier which blocks incoming calls until all threads
        have arrived.
        """
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
        """
        Second barrier to ensure that no thread can reenter phase 1 until all
        other threads have left it.
        """
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
        """
        Blocks until all threads have called this method.
        """
        self.phase1()
        self.phase2()

    def abort(self):
        """
        Breaks the barrier and resumes all waiting threads. This leaves the
        barrier in a broken state, future calls will raise a BrokenBarrierError.
        """
        self._mutex.acquire()
        for _ in range(self.parties):
            self._barrier1.release()
            self._barrier2.release()
        self.broken = True
        self._mutex.release()

    def reset(self):
        """
        Aborts and resets the barrier to its intitial state. This will resume
        all waiting threads.
        """
        self.abort()
        self._mutex.acquire()
        self.n_waiting = 0
        self._barrier1 = threading.Semaphore(0)
        self._barrier2 = threading.Semaphore(0)
        self.broken = False
        self._mutex.release()
