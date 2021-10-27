#!/usr/bin/env python
import rospy
import asyncio
from bleak import BleakClient
from orientation.msg import HapticMsg

class BLEConnector:
    def __init__(self, address, uuid):
        self.address = address
        self.uuid = uuid

    def send_message(self, message):
        async with BleakClient(address) as client:
            model_number = await client.read_gatt_char(MODEL_NBR_UUID)
            print("Model Number: {0}".format("".join(map(chr, model_number))))
            await client.write_gatt_char(MODEL_NBR_UUID, motor2bytes)
            await asyncio.sleep(0.005)

class HapticFeedbackConnector(BLEConnector):
    def __init__(self, address, uuid, obstacle_topic, vibration_motor_count):
        super().__init__(address, uuid)
        rospy.init_node('haptic_feedback',anonymous=True)
        self.obstacle_topic = obstacle_topic
        self.vibration_motor_count = vibration_motor_count
        rospy.Subscriber(self.obstacle_topic, HapticMsg,callback=self.haptic_feedback_callback, queue_size=1)

    def distance_to_duty_cycle(obstacle_distance, min_threshold=3, max_threshold=1):
        if obstacle_distance > min_threshold:
            duty_cycle = 0
        elif obstacle_distance > max_threshold:
            duty_cycle = (obstacle_distance-max_threshold)/(min_threshold-max_threshold)
        else:
            duty_cycle = 100
        return duty_cycle

    def send_motor_message(self, obstacle_distance, motor_index):
        duty_cycle = HapticFeedbackConnector.distance_to_duty_cycle(obstacle_distance)
        print((duty_cycle, motor_index))
        message_str = '{0}M{1}'.format(duty_cycle, motor_index)
        message_bytes = b'%b' % message_str.encode('utf8')
        self.send_message(message_bytes)

    def haptic_feedback_callback(self, obstacle_data):
        for motor_index in range(len(obstacle_data)):
            self.send_motor_message(obstacle_data[motor_index], motor_index)


if __name__ == '__main__':
    try:
        address = "EC:94:CB:4D:7F:7A"
        MODEL_NBR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
        vibration_motor_count = 3
        connector = HapticFeedbackConnector(address, MODEL_NBR_UUID,vibration_motor_count)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
