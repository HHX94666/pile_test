'''
Date: 2022-03-10 16:28:38
LastEditors: houhuixiang
'''
import _thread
import time
import sys
import glog as log
import logging
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from chassis_interfaces.srv import GetString


def initLogging(logFilename, e):

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s-%(levelname)s-%(message)s',
        datefmt='%y-%m-%d %H:%M',
        filename=logFilename,
        filemode='a')

    filehandler = logging.FileHandler(logFilename, encoding='utf-8')
    logging.getLogger().addHandler(filehandler)
    log = logging.exception(e)
    return log


class PileTest(Node):
    charge_state = 0

    def __init__(self):
        super().__init__('pile_test')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        log.setLevel("INFO")

        self.msg = rclpy.create_node('timer')

        self.charge_cancel = self.create_publisher(
            Empty, '/auto_charge/cancel', qos)
        self.charge_start = self.create_publisher(
            String, '/auto_charge/start', qos)

        self.charge_event = self.create_subscription(
            String,
            '/auto_charge/event',
            self.charge_event_callback,
            qos)
        
        self.charge_state = self.create_subscription(
            String,
            '/auto_charge/state',
            self.charge_state_callback,
            qos)

        self.charge_feedback = self.create_subscription(
            UInt8,
            '/auto_charge/charge_feedback',
            self.charge_feedback_callback,
            qos)

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.success_count = 0
        self.failed_count = 0
        self.btconnect_failed_count = 0
        self.btconnect_success_count = 0
        self.out_count = 0
        self.in_count = 0
        self.bt_state = 0
        self.auto_sign = False
        self.thread = threading.Thread(
            target=self.startThread, args=[])
        self.thread.start()


        msg = String()
        msg.data = "NO"
        self.charge_start.publish(msg)
        log.info('发送对桩命令')

    def charge_feedback_callback(self, msg):
        feedback = msg.data
        log.info('[sub] feedback:%d' % feedback)
        if feedback == 0:
            self.auto_sign = False
            self.out_count += 1
            log.info('机器人弹桩  累计次数 %d 次' % self.out_count)
        if feedback == 1 and self.auto_sign == False:
            self.auto_sign = True
            self.in_count += 1
            log.info('机器人压桩  累计次数 %d 次' % self.in_count)

    def charge_event_callback(self, msg):
        event = msg.data
        log.info('[sub] %s' % event)
        if event == "ChargeStart":
            self.success_count += 1
            log.info('充电成功  累计次数 %d 次' % self.success_count)
            self.charge_state = 1

        if event == "ChargeCancelSuccess":
            log.info('脱桩成功')
            self.charge_state = 2

        if event == "ChargeFailed":
            self.failed_count += 1
            log.error('充电失败！！！ 累计次数 %d 次' % self.failed_count)
            self.charge_state = 1

        if event == "BtConnectFailed":
            self.btconnect_failed_count += 1
            log.error('蓝牙连接失败！！！ 累计次数 %d 次' % self.btconnect_failed_count)
            
    def charge_state_callback(self, msg):
        state = msg.data
        log.info('[sub] %s' % state)
        if state == "WaitHeartbeat" and self.bt_state == 0:
            self.bt_state  = 1
            self.btconnect_success_count += 1
            log.info('蓝牙连接成功  累计次数 %d 次' % self.btconnect_success_count)
            
        if state != "WaitHeartbeat":
            self.bt_state  = 0


    def startThread(self):
        while True:
            msg = String()
            msg.data = "NO"
            if self.charge_state == 2:
                self.charge_start.publish(msg)
                log.info('发送对桩命令')
                self.charge_state = 0
                
            msg_m= Empty()
            if self.charge_state == 1:
                log.info('开始延时10分钟')
                time.sleep(60*10)
                log.info('延时结束')
                self.charge_cancel.publish(msg_m)
                log.info('发送脱桩命令')
                self.charge_state = 0

    def pub_start(self):
        msg = String()
        msg.data = "NO"
        self.charge_start.publish(msg)
        log.info('发送对桩命令')


def main(args=None):
    rclpy.init(args=args)
    initLogging("pile_test.log","info")

    pile_test = PileTest()

    pile_test.pub_start()
    pile_test.pub_start()
    pile_test.pub_start()
    pile_test.pub_start()

    try:
        rclpy.spin(pile_test)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        pile_test.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
