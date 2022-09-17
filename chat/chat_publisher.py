import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

import sys
import threading
from PyQt5.QtWidgets import *


class ChatNode(Node):

    def __init__(self):
        super().__init__('chat')



class CWidget(QWidget):
    def __init__(self):
        super().__init__()
        qos_profile = QoSProfile(depth=10)

        chat_node = ChatNode()
        self.nickname = '닉네임 미정'
        self.is_connected = False
        self.chat_publisher = chat_node.create_publisher(String, 'enjoychat', qos_profile)
        self.chat_subscriber = chat_node.create_subscription(
            String,
            'enjoychat',
            self.subscribe_topic_message,
            qos_profile)


        def node_execute(node):
            executor = MultiThreadedExecutor(num_threads=4)
            executor.add_node(node)

            try:
                executor.spin()
            except KeyboardInterrupt:
                node.get_logger().info('Keyboard Interrupt (SIGINT)')
            finally:
                node.destroy_node()
                rclpy.shutdown()

        t1 = threading.Thread(target=node_execute, args=[chat_node])
        t1.start()


        self.initUI()

    def subscribe_topic_message(self, msg):
        if self.is_connected:
            self.recvmsg.addItem(QListWidgetItem(msg.data))

    def initUI(self):
        self.setWindowTitle('클라이언트')

        # 클라이언트 설정 부분
        ipbox = QHBoxLayout()

        gb = QGroupBox('닉네임 설정')
        ipbox.addWidget(gb)

        box = QHBoxLayout()

        label = QLabel('닉네임')
        self.nickname_line_edit = QLineEdit()
        # self.ip.setInputMask('닉네임 입력')
        self.nickname_line_edit.returnPressed.connect(self.connectBtnClicked)
        box.addWidget(label)
        box.addWidget(self.nickname_line_edit)

        # label = QLabel('Server Port')
        # box.addWidget(label)

        self.btn = QPushButton('접속')
        self.btn.clicked.connect(self.connectBtnClicked)
        box.addWidget(self.btn)

        gb.setLayout(box)

        # 채팅창 부분
        infobox = QHBoxLayout()
        gb = QGroupBox('메시지')
        infobox.addWidget(gb)

        box = QVBoxLayout()

        label = QLabel('받은 메시지')
        box.addWidget(label)

        self.recvmsg = QListWidget()
        box.addWidget(self.recvmsg)

        label = QLabel('보낼 메시지')
        box.addWidget(label)

        # self.sendmsg = QTextEdit()
        self.sendmsg = QLineEdit()
        self.sendmsg.returnPressed.connect(self.sendMsg)
        self.sendmsg.setFixedHeight(50)
        box.addWidget(self.sendmsg)

        hbox = QHBoxLayout()

        box.addLayout(hbox)
        self.sendbtn = QPushButton('보내기')
        self.sendbtn.setAutoDefault(True)
        self.sendbtn.clicked.connect(self.sendMsg)

        self.clearbtn = QPushButton('채팅창 지움')
        self.clearbtn.clicked.connect(self.clearMsg)

        hbox.addWidget(self.sendbtn)
        hbox.addWidget(self.clearbtn)
        gb.setLayout(box)

        # 전체 배치
        vbox = QVBoxLayout()
        vbox.addLayout(ipbox)
        vbox.addLayout(infobox)
        self.setLayout(vbox)

        self.show()

    def connectBtnClicked(self):
        if self.is_connected == False:
            self.is_connected = True
            self.nickname = self.nickname_line_edit.text()
            msg = String()
            msg.data = f'{self.nickname}님 입장'
            self.chat_publisher.publish(msg)

    def updateMsg(self, msg):
        self.recvmsg.addItem(QListWidgetItem(msg))

    def updateDisconnect(self):
        self.btn.setText('접속')

    def sendMsg(self):
        if self.is_connected == True:
            # sendmsg = self.sendmsg.toPlainText()
            sendmsg = self.sendmsg.text()
            print('보낼메시지', sendmsg)
            msg = String()
            msg.data = f'{self.nickname} : {sendmsg}'
            self.chat_publisher.publish(msg)
            self.sendmsg.clear()
        else:
            QMessageBox.information(self,'Information Title','닉네임을 입력하고 접속해주세요.')

    def clearMsg(self):
        self.recvmsg.clear()



def main(argv=sys.argv[1:]):
    rclpy.init(args=argv)
    app = QApplication(sys.argv)

    w = CWidget()
    sys.exit(app.exec_())



if __name__ == '__main__':
    main()
