import os
import logging
import time
from datetime import datetime

import os
import sys

class FileLogger:
    def __init__(self, log_file_path):
        self.log_file_path = log_file_path
        # Create the named FIFO pipe
        try:
            os.mkfifo(self.log_file_path)
        except FileExistsError:
            pass

    def log(self, message):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_message = f'[{timestamp}] {message}'
        with open(self.log_file_path, 'a') as fifo:
            fifo.write(log_message + '\n')

    def read_log(self):
        with open(self.log_file_path, 'r') as fifo:
            while True:
                line = fifo.readline()
                if len(line) == 0:
                    break
                print(line.strip())

    def close(self):
        if os.path.exists(self.log_file_path):
            os.remove(self.log_file_path)

# # Example usage:
# if __name__ == "__main__":
#     logger = FileLogger('./my_log_pipe.txt')
#     logger.close()
    
#     logger.log('This is a log message.')
#     logger.log('Another log message.')

#     # Read the log
#     print('Reading from the log pipe:')
#     logger.read_log()

#     # Clean up
#     # logger.close()
