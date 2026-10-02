# ROS 2 launch 관련 모듈 불러오기
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# launch 파일을 생성하는 함수
def generate_launch_description():


    # 1. main.cpp 실행 (다이나믹셀 모션 실행 제어)
    main_node = Node(
        package='forward_walk',      # 모션 실행 로직이 있는 패키지명
        executable='main_node',      # main.cpp에서 빌드된 실행파일 이름
        name='main_node',
        output='screen',
    ) 

    # # 2. xsens 실행
    xsens_mti_node = TimerAction(
        period=9.0,
        actions=[    
            Node(
                package='bluespace_ai_xsens_mti_driver',
                executable='xsens_mti_node',
                name='xsens_mti_node',
                output='screen',
                arguments=[]
            )
        ]
    )

    # 3. RealSense 카메라 실행 (별도의 launch 포함)
    realsense_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch',
                        'dual_realsense_launch.py'
                    )
                )
            )
        ]
    )

    # 4. 12초 후 실행: 나머지 노드들
    delayed_nodes = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='my_cv',
                executable='yolo_cpp',
                name='line_publisher1',
                output='screen'
            ),
            Node(
                package='my_cv',
                executable='yolo_line_subscriber.py',
                name='line_subscriber',
                output='screen',
                arguments=['--ros-args', '--log-level', 'line_subscriber:=error']
            ),
            Node(
                package='my_cv',
                executable='ball_and_hoop.py',
                name='ball_and_hoop',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'] 
            ),
            Node(
                package='decision',
                executable='fall_detect',
                name='fall_detect',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'] 
            ),
            Node(
                package='decision',
                executable='motion_decision',
                name='motion_decision',
                output='screen',
                remappings=[
                    ('line_result', '/line_result'),
                    ('motion_command', '/motion_command'),
                ],
                arguments=['--ros-args', '--log-level', 'debug'] 
            ),
        ]
    )



    # LaunchDescription 객체에 위에서 정의한 모든 노드 포함
    return LaunchDescription([
        main_node,
        xsens_mti_node,
        realsense_launch,
        delayed_nodes
    ])







# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     # 토글 인자 (둘 다 기본 ON)
#     start_yolo_cpp = DeclareLaunchArgument('start_yolo_cpp', default_value='true')
#     start_yolo_sub = DeclareLaunchArgument('start_yolo_sub', default_value='true')

#     # 1) 메인 노드 (이름 충돌 가독성 위해 임시로 이름 변경 권장)
#     main_node = Node(
#         package='forward_walk',
#         executable='main_node',
#         name='main_node_fw',                     # <-- 임시로 변경. 원하면 원복 가능
#         output='screen',
#         remappings=[
#             ('motion_cmd', '/motion_command'),
#             ('command', '/motion_command'),
#         ],
#     )

#     # 2) motion_decision 은 항상 띄움 (YOLO와 무관하게 존재 보장)
#     decision_node = Node(
#         package='decision',
#         executable='motion_decision',
#         name='line_result_subscribe',
#         output='screen',
#         respawn=True, respawn_delay=2.0,        # 크래시 자동 재기동
#         remappings=[
#             ('line_result', '/line_result'),    # 구독
#             ('motion_command', '/motion_command'),  # 발행
#         ],
#         arguments=['--ros-args', '--log-level', 'motion_decision:=debug'],
#     )

#     # 3) RealSense (조금 늦게)
#     realsense_launch = TimerAction(
#         period=10.0,
#         actions=[
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(
#                     os.path.join(
#                         get_package_share_directory('realsense2_camera'),
#                         'launch',
#                         'rs_launch.py'
#                     )
#                 )
#             )
#         ]
#     )

#     # 4) YOLO 노드들 (각각 따로 토글, 에러 보기 위해 screen)
#     yolo_cpp_node = TimerAction(
#         period=12.0,
#         actions=[Node(
#             condition=IfCondition(LaunchConfiguration('start_yolo_cpp')),
#             package='my_cv',
#             executable='yolo_cpp',
#             name='line_publisher1',
#             output='screen', emulate_tty=True,   # ← 디버그 위해 콘솔에 직접 출력
#             remappings=[('candidates', '/candidates')],
#         )]
#     )

#     yolo_sub_node = TimerAction(
#         period=14.0,
#         actions=[Node(
#             package='my_cv',
#             executable='yolo_line_subscriber.py',
#             name='line_subscriber',
#             output='screen',           # 에러를 콘솔로
#             emulate_tty=True,          # 버퍼링 없이 바로 출력
#             respawn=True, respawn_delay=2.0,  # 죽어도 재기동 (원인 찾기 쉬움)
#             remappings=[
#                 ('candidates', '/candidates'),    # 입력
#                 ('line_points', '/line_result'),  # 출력: decision이 구독하는 이름으로 고정
#             ],
#             arguments=['--ros-args','--log-level','line_subscriber:=debug'],
#         )]
#     )

#     return LaunchDescription([
#         start_yolo_cpp, start_yolo_sub,
#         main_node, decision_node, realsense_launch,
#         yolo_cpp_node, yolo_sub_node
#     ])
