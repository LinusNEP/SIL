import csv
import os
from datetime import datetime

class LogEntry:
    def __init__(self):
        self.timestamp = datetime.now().isoformat()
        self.input_command = ''  
        self.input_timestamp = ''  
        self.response_timestamp = ''  
        self.true_action_sequence = ''  
        self.predicted_action_sequence = '' 
        #self.action_feedback = ''  
        self.execution_success = '' 

class DataLogger:
    def __init__(self, log_file='robot_log.csv'):
        self.log_file = log_file
        file_exists = os.path.isfile(self.log_file)
        self.file = open(self.log_file, 'a', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.file)
        if not file_exists:
            self.csv_writer.writerow([
                'Timestamp',
                'Input Command',
                'Input Timestamp',
                'Response Timestamp',
                'True Action Sequence',
                'Predicted Action Sequence',
                #'Action Feedback',
                'Execution Success'
            ])
            self.file.flush()

    def log_entry(self, log_entry):
        self.csv_writer.writerow([
            log_entry.timestamp,
            log_entry.input_command,
            log_entry.input_timestamp,
            log_entry.response_timestamp,
            log_entry.true_action_sequence,
            log_entry.predicted_action_sequence,
            #log_entry.action_feedback,
            log_entry.execution_success
        ])
        self.file.flush()

    def close(self):
        self.file.close()
