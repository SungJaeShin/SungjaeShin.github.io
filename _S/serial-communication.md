---
title: "[Serial Communication] Communication between NUC and SBC"
excerpt: "Serial Communication btw NUC and SBC"
---
# [Serial Communication] Communication between NUC and SBC

## Ubuntu OS가 설치되어 있는 2대의 NUC와 SBC에서 메세지를 serial로 통신하여 주고 받는 방법

---

- **[Reference Site]**
    - [https://engcang.github.io/Ubuntu-PC-to-PC-USB-to-USB-serial-communication/](https://engcang.github.io/Ubuntu-PC-to-PC-USB-to-USB-serial-communication/)
                
    - [https://pyserial.readthedocs.io/en/latest/shortintro.html](https://pyserial.readthedocs.io/en/latest/shortintro.html)
        
    - [https://blog.naver.com/chandong83/221156763486](https://blog.naver.com/chandong83/221156763486)
        

---

- **[기본적으로 알아야 할 내용]**
    - Based on python code
    
    ```python
    import serial 
    
    ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
    
    # [First Case] 
    if ser.is_open:
    	for x in range(10):
    		data = data + x
    	ser.write(data)
    ser.close()
    
    # [Second Case]
    if ser.is_open:
    	data = ser.read(100)
    	print(data)
    ser.close()
    ```
    
    - (1) import serial
        - serial communication을 하려면 serial 이라는 library를 import해주어야 한다.
    - (2) ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
        - 통신을 위해서 먼저 serial을 정의해준다.
            - [1] /dev/ttyUSB0:
                - Terminal에 "$ cd /dev" 를 진행하고 목록을 보면 USB의 port name을 알 수 있는데 이 부분이 USB가 연결되어 있는 이름을 작성하는 부분이다.
            - [2] 19200:
                - Baud Rate의 입력 값으로 정확하지는 않지만 아마 19200이 maximum Baud Rate일 것이다. 이는 초당 얼마나 많은 symbol(의미 있는 데이터 묶음)을 전송할 수 있는지를 나타내는 말이다.
                    - Serial communication에서는 1개의 symbol은 8개의 bit를 담고 있다.
                    - Reference Site: [https://m.blog.naver.com/gmqgufrn/220862079486](https://m.blog.naver.com/gmqgufrn/220862079486)
                        
            - [3] timeout=1:
                - timeout은 데이터를 읽을 때 사용되는 변수로 3가지 case로 볼 수 있다.
                    - timeout=None
                        - 요청된 baudrate가 모두 받아질 때까지 계속 기다린다.
                    - timeout=0
                        - blocking이 없는 방식으로 정보가 올때마다 계속 전달한다.
                    - timeout=x
                        - x초이내에는 해당 baudrate만큼 다 받았다면 바로 반환하고 이 경우가 아니라면 x초까지 계속 정보를 받다가 반환한다.
    - (3) ser.is_open
        - boolean type으로 serial port가 열려서 데이터를 받을 수 있다는 flag와 같다고 생각하면 된다.
    - (4) ser.write(data)
        - serial port를 통해서 전달할 때 사용하는 것으로 data를 다른 연결된 device로 전송한다.
    - (5) ser.read(value)
        - 다른 device로부터 timeout 이내의 value만큼의 byte로 수신을 하게 된다.
        - 예시에서 작성된 value는 100이므로 100byte 만큼의 정보를 다른  device로부터 얻을 수 있다.
    - (6) ser.close()
        - serial port를 열어주고 device끼리 충분한 데이터를 주고 받았다면 이제는 닫아주는 역할을 하게 된다.

---

- **[ROS message를 serial communication하는 경우]**
    - **(1) NUC to SBC**
    
    ```python
    import serial
    import time
    
    import rospy
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Twist
    
    import sys
    import signal
    
    # ctrl + c -> exit program
    def signal_handler(signal, frame): 
      print('You pressed Ctrl+C!')
      sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    
    def cmd_callback(msg):
      print("cmd_callback!")
      cmd = Twist()
      cmd.linear.x = msg.linear.x 
      cmd.linear.y = msg.linear.y 
      cmd.linear.z = msg.linear.z 
      cmd.angular.x = msg.angular.x 
      cmd.angular.y = msg.angular.y 
      cmd.angular.z = msg.angular.z 
    
      ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
    
      if ser.is_open:
        print("ser is open")
        data = 's,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,'%(cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z)
        while len(data) < 80: 
          data = data + 'x'
        ser.write(data)
        **time.sleep(0.5) ***important*****
      else:
        ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
    
    if __name__ == '__main__':
      ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
      rospy.init_node("NUC_Node", anonymous=True)
      received_odom_pub = rospy.Publisher("/received_pose", Pose, queue_size=1)
      send_to_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
    
      while 1:
        try:
          if ser.is_open:
            d=ser.read(80)
            b=d.split(',')
            if b[0]!='s':
              ser.close()
            else:
              pmsg = Pose()
              pmsg.position.x = float(b[1])
              pmsg.position.y = float(b[2])
              pmsg.position.z = float(b[3])
              pmsg.orientation.x = float(b[4])
              pmsg.orientation.y = float(b[5])
              pmsg.orientation.z = float(b[6])
              pmsg.orientation.w = float(b[7])
              received_odom_pub.publish(pmsg)
          else:
            ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1) 
        except (SystemExit, KeyboardInterrupt) :
          ser.close()             # close port
          sys.exit(0)
    ```
    
    - **(2) SBC to NUC**
        - 여기서, **rospy.Rate(5) → rospy.Rate(30)** 으로 변경을 해주면 한번 클릭만 해주어도 계속 로봇이 돌거나 이동하는 현상으로 변경할 수 있다. 이는 메세지 Hz를 5에서 30으로 늘려주는 것이라서 그만큼 빠르게 publish하기 때문에 마치 그렇게 보이기 때문이다.
    
    ```python
    """
    Created on Thu Jan 21 04:52:04 2021
    @author: EungChang Mason Lee
    @based on EungChang Mason Lee codes 
    """
    
    import serial
    import time
    
    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    
    import sys
    import signal
    def signal_handler(signal, frame):
        print('Pressed Ctrl+C')
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    
    ''' class '''
    class serial_to_NUC():
        def __init__(self):
            rospy.init_node('serial_connector', anonymous=True)
            self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
    
            self.rate = rospy.Rate(30)
            self.ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
    
        def odom_callback(self, msg):
            if self.ser.is_open:
                data = 's,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,'%(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                while len(data)<80:
                    data= data + 'x'
                self.ser.write(data)
    						**time.sleep(0.5) ***important*****
       
        def twist_pub(self,msg):
            self.cmd_pub.publish(msg)
                    
    if __name__== '__main__':
        ser_nuc = serial_to_NUC()
        twist_msg= Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0
    
        while 1:
            try:
                if ser_nuc.ser.is_open:
                    d=ser_nuc.ser.read(80)
                    if not(len(d)<80):
                        if(d): 
                            try:		    
                                x =[i for i in d.split(',')]
                                print(x)
                                twist_msg.linear.x = float(x[1])
                                twist_msg.linear.y = float(x[2])
                                twist_msg.linear.z = float(x[3])
                                twist_msg.angular.x = float(x[4])
                                twist_msg.angular.y = float(x[5])
                                twist_msg.angular.z = float(x[6])
                            except:
                                print("corrupted message")
                        else:
                            print("empty message")
                ser_nuc.twist_pub(twist_msg)
                ser_nuc.rate.sleep()
    
            except (SystemExit, KeyboardInterrupt):
                ser_nuc.ser.close()
                sys.exit(0)
    ```
    

---

- **[Error 및 느낀점]**
    - (Error 1) serial.serialutil.SerialException: device reports readiness to read but returned no data (device disconnected or multiple access on port?)
        - 이이는 같은 포트에서 닫히고 열리는 과정이 반복적으로 진행이 되어 다시 포트를 열 시간이 부족하기 때문에 생기는 문제라고 한다.
        - 해결 방법은 write하는 부분에 time.sleep(0.5)를 추가로 넣어 0.5초씩 delay를 줌으로서 해결을 한다. (******important****** 라고 했던 부분!)
        - Reference Site: [https://github.com/pyserial/pyserial/issues/86](https://github.com/pyserial/pyserial/issues/86)
            
    ---
    
    - (Error 2) serial 통신이다보니 데이터의 순서가 변경이 되거나 개수가 충분치 않게 들어오는 경우가 있다.
        - 해결 방법은 중간에 **if(len(data) < value)** 를 삽입하여 원하는 데이터의 개수가 누락이 되지 않았는지 필터링을 하는 방법이 있다.
        - 또는 **if(data)** 를 설정하여 데이터가 아얘 들어오지 않았는지 확인해보는 경우도 있다.
        - 마지막으로는 **try** 문을 사용하여 데이터가 잘못들어왔을 경우에는 이 statement를 그냥 넘기도록 만들어준다.
    
    ---
    
    - **(느낀점)**
        - Serial communication을 하려는 것은 가장 직관적으로 할 수 있는 방법이나, 상당히 비효율적인 방법이라고 생각한다.
        - 예를 들면, 메세지 자체의 손실이 많이 일어나게 된다.
            - Error에서와 같이 정보가 전부 들어오지 않는 경우, 정보의 순서가 뒤바뀐 경우, 연속적으로 붙어 있는 데이터가 같은 공간에 합쳐져서 들어오는 경우(1.2, 3.4 → 1.23.4)
        - 메세지의 delay도 고려를 해야하기 때문에 앞서 전송했던 앞으로 가는 명령에 좌로 도는 명령이 overlap이 되어 앞으로 움직이면서 좌도 도는 현상도 보이기도 하였다.

---

- **[결론]**
    - **되도록이면 Serial communication을 쓰지 말자~!!!**