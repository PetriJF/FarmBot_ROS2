import rclpy
from rclpy.node import Node
import socket
from datetime import datetime
import json
import threading
from flask import Flask, render_template_string
from ament_index_python.packages import get_package_share_directory
import os, yaml

# Specify the path where the data should be saved
file_path = '/home/farmbotdev/Random_Scripts/received_data.txt'  # Adjust the file path as needed

# Global variables to store the latest sensor data from Arduino 1 and Arduino 2
latest_data_arduino_1 = ''
latest_data_arduino_2 = ''

# Flask app for serving the webpage
app = Flask(__name__)

@app.route('/')
def index():
    global latest_data_arduino_1, latest_data_arduino_2
    # HTML template to display the sensor data and auto-refresh the page every 5 seconds
    html_template = '''
    <html>
        <head>
            <title>Sensor Data</title>
            <meta http-equiv="refresh" content="5"> <!-- Auto-refresh every 5 seconds -->
        </head>
        <body>
            <h1>Latest Sensor Data</h1>
            <h2>Arduino 1 (Sensors 1-16)</h2>
            <pre>{{ sensor_data_arduino_1 }}</pre>
            <h2>Arduino 2 (Sensors 17-32)</h2>
            <pre>{{ sensor_data_arduino_2 }}</pre>
        </body>
    </html>
    '''
    return render_template_string(html_template, sensor_data_arduino_1=latest_data_arduino_1, sensor_data_arduino_2=latest_data_arduino_2)

class TCPServerNode(Node):
    def __init__(self):
        super().__init__('array_server')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 5000))  # Bind to all interfaces on port 5000
        self.server_socket.listen(1)  # Listen for 1 client connection
        self.get_logger().info('Server started, waiting for connections...')
        
        # Active map location and file name
        directory = os.path.join(
            get_package_share_directory('map_handler'),
            'config'
        )
        active_map_file = 'active_map.yaml'

        self.map_instance = self.retrieve_map(directory, active_map_file)
        self.current_plant_index = 0
        self.current_water_reading = 0.0

        self.server_timer_ = self.create_timer(0.2, self.server_callback)
        self.watering_timer_ = self.create_timer(0.5, self.form_water_sequence_callback)

    def server_callback(self):
        client_socket, addr = self.server_socket.accept()
        self.get_logger().info(f"Connection from {addr}")

        data = client_socket.recv(1024).decode('utf-8')  # Receive data from Arduino
        if data:
            self.get_logger().info(f"Received data: {data}")
            self.process_and_save_data(data)

        client_socket.close()

    def process_and_save_data(self, data):
        try:
            global latest_data_arduino_1, latest_data_arduino_2

            # Parse the received JSON data
            sensor_data = json.loads(data)

            # Create a timestamp for the entry
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Check which device is sending the data
            device_id = sensor_data.get('device_id', 'unknown')

            # Parse the sensor data
            sensors = sensor_data['sensors']
            if len(sensors) != 16:  # We expect 16 sensors from each Arduino
                self.get_logger().error(f"Expected 16 sensor readings, got {len(sensors)}")   
                return

            # Determine sensor labels based on device ID
            if device_id == 'arduino_1':
                start_label = 1
                # Format the data for Arduino 1
                formatted_data_arduino_1 = f"{timestamp} - "
                for i, sensor_value in enumerate(sensors):
                    sensor_label = start_label + i
                    formatted_data_arduino_1 += f"Sensor {sensor_label}: {sensor_value}%, "
                formatted_data_arduino_1 = formatted_data_arduino_1.rstrip(', ') + '\n'

                # Update the latest data for Arduino 1
                latest_data_arduino_1 = formatted_data_arduino_1

            elif device_id == 'arduino_2':
                start_label = 17
                # Format the data for Arduino 2
                formatted_data_arduino_2 = f"{timestamp} - "
                for i, sensor_value in enumerate(sensors):
                    sensor_label = start_label + i
                    formatted_data_arduino_2 += f"Sensor {sensor_label}: {sensor_value}%, "
                formatted_data_arduino_2 = formatted_data_arduino_2.rstrip(', ') + '\n'

                # Update the latest data for Arduino 2
                latest_data_arduino_2 = formatted_data_arduino_2

            else:
                self.get_logger().error('Unknown device ID')
                return

            # Save the data to a file
            formatted_data = formatted_data_arduino_1 if device_id == 'arduino_1' else formatted_data_arduino_2
            with open(file_path, 'a') as file:
                file.write(formatted_data)
            self.get_logger().info(f"Data saved to {file_path}: {formatted_data}")

        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")

    def form_water_sequence_callback(self, target_water_amount = 0.5):
        def water_plant(x, y, z, pulse_len) -> str:
            sub_sequence = ''
            
            sub_sequence += 'CC_soil_array\n'
            sub_sequence += f'{x} {y} {z}\n'

            sub_sequence += 'DC_soil_array\n'
            sub_sequence += f'WaterPulses {pulse_len}\n' #ms
            return sub_sequence

  

        # get all plants that need to be watered
        # CC to the coordinate of a plant
        # DC a starting water pulse

        # TODO: in future, feedback     

        if self.current_water_reading >= target_water_amount:
            self.get_logger().info('Watering on current plant finished')
            self.current_water_reading = 0.0

    def retrieve_map(self, directory = '', file_name = ''):
        '''
        Attempts to retrieve the map configuration file from memory.
        If it fails, it either means that the file was deleted or the
        current run is a fresh run. This node will not work on a fresh
        run unless plants are added in manually.

        Args:
            directory {String}: The share directory the yaml files are located at
            file_name1 {String}: The active config (i.e. the one from memory)
        '''
        active_config = os.path.join(directory, file_name)
        if os.path.exists(active_config):
            self.get_logger().info('Initialized map from previous run!')
            return self.load_from_yaml(directory, file_name)
        else:
            self.get_logger().error('Previous map info could not be found! Unless you have a back-up, previous items need to be re-added')
            return

def start_flask():
    # Start Flask web server on a different thread
    app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False)

def main(args=None):
    # Start the Flask app in a separate thread
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.start()

    # Initialize ROS 2 node
    rclpy.init(args=args)
    tcp_server_node = TCPServerNode()
    rclpy.spin(tcp_server_node)
    tcp_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
