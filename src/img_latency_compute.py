#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
import csv
import os

class HeaderToCSV:
    def __init__(self):
        # Initialize the node
        rospy.init_node('header_to_csv_node', anonymous=True)

        # Set up subscriber to the 'image_header' topic
        rospy.Subscriber('/img_latency', Header, self.callback)

        # Set up CSV file
        self.csv_file = '20240925_img_latency_test.csv'
        self.setup_csv()

        rospy.loginfo("HeaderToCSV Node Started")
    
    def setup_csv(self):
        # Check if the CSV file exists; if not, create it and add headers
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='wb') as file:  # 'wb' mode for binary writing
                writer = csv.writer(file)
                writer.writerow(['Seq', 'Stamp', 'Frame_id', 'Latency (ms)'])  # Add headers

    def callback(self, header):
        # Calculate the timestamp from the header in seconds
        header_time = header.stamp.secs + header.stamp.nsecs * 1e-9
        
        # Get the current time in seconds
        current_time = rospy.Time.now().secs + rospy.Time.now().nsecs * 1e-9
        
        # Calculate the latency in milliseconds
        latency = (current_time - header_time) * 1000.0  # Convert to milliseconds
        
        # Write header data and latency to CSV
        with open(self.csv_file, mode='ab') as file:  # 'ab' mode for appending in binary
            writer = csv.writer(file)
            writer.writerow([header.seq, header_time, header.frame_id, latency])

        rospy.loginfo("Header data written to CSV: Seq={}, Stamp={}, Frame_id={}, Latency={} ms".format(
            header.seq, header_time, header.frame_id, latency))

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        header_to_csv = HeaderToCSV()
        header_to_csv.run()
    except rospy.ROSInterruptException:
        pass
