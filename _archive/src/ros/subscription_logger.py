#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import re

class SubscriptionLogger:
    def __init__(self):
        rospy.init_node('subscription_logger', anonymous=True)
        
        # Create a custom topic for subscription logs
        self.sub_log_pub = rospy.Publisher('/robot/subscription_logs', String, queue_size=100)
        
        # Subscribe to rosout to filter subscription messages
        rospy.Subscriber('/rosout', String, self.rosout_callback)
        
        rospy.loginfo("Subscription Logger initialized")

    def rosout_callback(self, msg):
        # Check if the message is a subscription/unsubscription log
        if '[Client' in msg.data and ('Subscribed to' in msg.data or 'Unsubscribed from' in msg.data):
            # Extract the relevant information
            client_id = re.search(r'\[Client (.*?)\]', msg.data).group(1)
            action = 'Subscribed to' if 'Subscribed to' in msg.data else 'Unsubscribed from'
            topic = msg.data.split(action)[1].strip()
            
            # Create a cleaner log message
            log_msg = f"{action} {topic} [Client: {client_id}]"
            
            # Publish to our custom topic
            self.sub_log_pub.publish(log_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = SubscriptionLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass 