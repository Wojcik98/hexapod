from dataclasses import dataclass
from typing import List

from numpy import ndarray

MAX_INTERVAL = 0.1


@dataclass
class HistoryRecord:
    timestamp: float
    point: ndarray


class PathProxy:
    def __init__(self, path_buffer: List[ndarray], logger):
        self.path_buffer = path_buffer
        self.history: List[HistoryRecord] = []
        self.logger = logger

    def get_point(self, time: float) -> ndarray:
        candidates = list(filter(
            lambda x: abs(x.timestamp - time) < MAX_INTERVAL,
            self.history))

        if len(candidates) > 0:
            if len(candidates) > 1:
                # TODO user logger
                print("[WARN] More than one point with timestamp found")
            return candidates[0].point
        else:
            if len(self.path_buffer) > 0:
                new_point = self.path_buffer.pop(0)
                new_record = HistoryRecord(time, new_point)
            else:
                prev_point = self.history[-1].point
                new_record = HistoryRecord(time, prev_point)

            self.history.append(new_record)
            return new_record.point

    def first_unused(self) -> ndarray:
        print('Taking from proxy')
        if len(self.path_buffer) > 0:
            return self.path_buffer[0]
        else:
            return self.history[-1].point
