import csv
from datetime import datetime
import time
from os.path import join

from .state import create_state_random, serialize_state_to_dict


def time_delta(first_value=None):
    def time_delta_decorator(function):
        last_execution = None

        def decorated(*args, **kwargs):
            nonlocal last_execution

            time_now = time.time()
            try:
                delta = time_now - last_execution
            except TypeError:
                delta = first_value
            kwargs['time_delta'] = delta
            last_execution = time_now
            return function(*args, **kwargs)
        return decorated
    return time_delta_decorator


def time_throttled(min_delta):
    def time_throttled_decorator(function):
        last_execution = 0

        def decorated(*args, **kwargs):
            nonlocal last_execution

            time_now = time.time()

            if time_now - last_execution >= min_delta:
                last_execution = time_now
                return function(*args, **kwargs)
            else:
                return None
        return decorated
    return time_throttled_decorator


class UpdateRate:
    def __init__(self):
        self.counter = 0

    def update(self):
        self.counter += 1

    def pop_rate(self):
        c = self.counter
        self.counter = 0
        return c


class LogWriter:
    def __init__(self, log_file_path):
        self._log_file_path = log_file_path
        self._log_file = None

    def new_mission(self, params):
        if self._log_file_path is None:
            return

        filename = '{}-{}.sas_mission.csv'.format(
            datetime.now().isoformat(),
            '_'.join(["{}={:.4g}".format(k, v)
                      for k, v in params.items()]),

        )
        self._log_file = open(
            join(self._log_file_path, filename),
            'a', newline='', buffering=100
        )
        csv_fields = [
            'id',
            'timestamp',
        ] + list(serialize_state_to_dict(create_state_random()).keys())
        self._log_writer = csv.DictWriter(
            self._log_file, fieldnames=csv_fields
        )
        self._log_writer.writeheader()

    def stop(self):
        if self._log_file is not None:
            self._log_file.close()
        self._log_file = None

    def add_record(self, data):
        if self._log_file is None:
            return
        self._log_writer.writerow(data)
