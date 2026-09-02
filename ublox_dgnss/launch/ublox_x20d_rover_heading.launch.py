""" Launch ublox_dgnss_node publishing high precision position and dual-antenna heading"""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  """Generate launch description for ublox_dgnss ZED-X20D heading components."""

  namespace = LaunchConfiguration('namespace')
  device_family = LaunchConfiguration("device_family")
  device_serial_string = LaunchConfiguration('device_serial_string')
  frame_id = LaunchConfiguration('frame_id')
  daheading_offset = LaunchConfiguration('daheading_offset')
  min_carr_soln = LaunchConfiguration('min_carr_soln')

  log_level_arg = DeclareLaunchArgument(
    "log_level", default_value=TextSubstitution(text="INFO")
  )
  namespace_arg = DeclareLaunchArgument(
    "namespace", default_value=""
  )
  device_family_arg = DeclareLaunchArgument(
    "device_family", default_value=TextSubstitution(text="X20D")
  )
  device_serial_string_arg = DeclareLaunchArgument(
    "device_serial_string",
    default_value="",
    description="Serial string of the device to use"
  )
  frame_id_arg = DeclareLaunchArgument(
    "frame_id",
    default_value="ubx",
    description="The frame_id to use in header of published messages"
  )
  daheading_offset_arg = DeclareLaunchArgument(
    "daheading_offset",
    default_value="0",
    description="CFG_NAVSPG_DAHEADING_OFFSET in raw 0.01 degree units (9000 = +90 deg)"
                " between the antenna 1 to antenna 2 baseline and the vehicle"
                " forward direction"
  )
  min_carr_soln_arg = DeclareLaunchArgument(
    "min_carr_soln",
    default_value="2",
    description="Minimum carrier solution to publish heading/imu:"
                " 0 = any, 1 = RTK float, 2 = RTK fixed"
  )

  params = [{"DEVICE_FAMILY": device_family},
            {'DEVICE_SERIAL_STRING': device_serial_string},
            {'FRAME_ID': frame_id},
            {'CFG_USBOUTPROT_NMEA': False},
            {'CFG_RATE_MEAS': 100},
            {'CFG_RATE_NAV': 1},
            # USB output (0x01ab CDC-ACM interface)
            {'CFG_MSGOUT_UBX_NAV_DAHEADING_USB': 1},
            {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
            {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 5},
            {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
            {'CFG_NAVSPG_DAHEADING_OFFSET': ParameterValue(daheading_offset, value_type=int)},
            ]

  container1 = ComposableNodeContainer(
    name='ublox_dgnss_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        plugin='ublox_dgnss::UbloxDGNSSNode',
        name='ublox_dgnss',
        namespace=namespace,
        parameters=params
      )
    ]
  )

  container2 = ComposableNodeContainer(
    name='ublox_nav_sat_fix_hp_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_nav_sat_fix_hp_node',
        plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
        name='ublox_nav_sat_fix_hp',
        namespace=namespace
      )
    ]
  )

  container3 = ComposableNodeContainer(
    name='ublox_heading_imu_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_heading_imu_node',
        plugin='ublox_heading_imu::UbloxHeadingImuNode',
        name='ublox_heading_imu',
        namespace=namespace,
        parameters=[{'min_carr_soln': ParameterValue(min_carr_soln, value_type=int)}]
      )
    ]
  )

  return launch.LaunchDescription([
    log_level_arg,
    device_family_arg,
    namespace_arg,
    device_serial_string_arg,
    frame_id_arg,
    daheading_offset_arg,
    min_carr_soln_arg,
    container1,
    container2,
    container3,
    ])
