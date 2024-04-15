import csv

class MetricGenerator():
    def __init__(self, file_name, field_names):
        self.file_name = file_name
        self.fieldnames = field_names # ex: fieldnames = ['timestamp', 'path_length', 'path_coverage', 'time_to_goal', 'battery_level', 'obstacle_collisions']


        with open(self.file_name, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
            writer.writeheader()
            # writer.writerows(data)
    

    def generate_metrics(self, data):
        # Assuming data is a list of dictionaries where each dictionary represents a row of data
        fieldnames = data[0].keys() if data else []
        
        with open(self.file_name, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data)


    def append_info(self, additional_data):
        # ex: additional_info = {'timestamp': '2024-04-20 09:30:00', 'path_length': 12, 'path_coverage': 0.85, 'time_to_goal': 18, 'battery_level': 80, 'obstacle_collisions': 0}

        with open(self.file_name, mode='a', newline='') as csv_file:
            writer = csv.DictWriter(csv_file)
            writer.writerow(additional_data)