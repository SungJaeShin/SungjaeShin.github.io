---
title: "[Visualized Pose Tracking]"
classes: wide
---

## Github Code 
[![Github_Page]({{ site.url }}{{ site.baseurl }}/assets/images/github_logo.png)](https://github.com/SungJaeShin/pose_movement.git)*Click Image!*

## pose의 정보가 담겨있는 .csv file을 이용하여 rviz상으로 움직임 파악하기
### csv file
[example]
- 498.00488,-436.80432,-4.7894874,0.55949587,-0.46982133,0.4388751,-0.52308786 <br>
- 차례대로 다음과 같은 값들을 의미한다.
  * position x = 498.00488
  * position y = -436.80432
  * position z = -4.7894874
  * orientation w = 0.55949587
  * orientation x = -0.46982133
  * orientation y = 0.4388751
  * orientation z = -0.52308786


### Node & Topic explanation
[Node]
- "pose_movement" Node

[Topic]
- "pose" topic
  * message type : geometry_msgs::PoseStamped <br>
- "tracking" topic
  * message type : nav_msgs::Path <br>
- "odom" topic
  * message type : nav_msgs::Odometry <br>

[Information]
- header.stamp &#8658; ros::Time::now()
- header.frame_id &#8658; base_link



### Parameter explanation
- std::ifstream file &#8658; 나의 Computer에 있는 CSV file을 받을 변수 <br>
- std::vector<double> result &#8658; CSV file을 comma(,) 제거 후 double type을 가진 std::vector로 넣어주기 위한 변수 <br>
- int index &#8658; 1set에 7개의 value들이 있기 때문에 구분하기 위해 설정 <br>
- double array[7] &#8658; 7개의 value들을 담기 위한 임시 배열이고 7개가 모두 채워진다면 이 값들을 publish하기 위해서 설정 <br>
- nav_msgs::Path path &#8658; pose의 이동경로를 출력하기 위해서 설정 <br>
- auto result_address &#8658; result의 첫 번째 주소값으로 움직이기 위해 설정 <br>
- geometry_msgs::PoseStamped pose_track &#8658; 현재 위치해있는 pose를 나타내기 위해 설정 <br>
- nav_msgs::Odometry odom &#8658; 시작점부터 끝날때까지의 경로를 출력하기 위해서 설정 <br>



### overall code explanation
- function 1 &#8658; std::vector<double> parseCSV(std::istream &file)
  * CSV file을 받아서 각 방들이 double type을 가지는 std::vector로 넣어준다.
  * 이 경우, comma(,)의 값을 빼고 Value들이 들어가도록 설정되어 있다.

<br>

- function 2 &#8658; geometry_msgs::PoseStamped get_pose(double x, double y, double z, double q_w, double q_x, double q_y, double q_z)
  * double type의 방을 가진 std::vector의 모든 값을 geometry_msgs::PoseStamped로 바꿔준다.
  * 이 경우, 한 줄씩 받아서 while문 안의 __pose_track__ 에 넣어준다.


<br>