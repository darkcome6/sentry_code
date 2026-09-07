# 聚合 PID 诊断：一个控制器内所有 PID 的状态折叠成一个话题
# （方案 B：单话题，如 /sentry/chassis_controller/pid_states）
std_msgs/Header header
PidState[] states
