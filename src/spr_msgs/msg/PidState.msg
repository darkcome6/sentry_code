# 单个 PID 的实时诊断快照（自研通用 PID，替代 control_toolbox::PidROS）
# 字段名自解释：target=目标值, feedback=反馈值, feedforward=前馈值
# 用于调试：可同时看到 P/I/D 各项输出、前馈大小、目标/反馈/误差、增益与饱和状态。

string joint        # 控制器内关节/轴标识，如 left_wheel_front / pitch

float64 kp          # 当前生效比例增益
float64 ki          # 当前生效积分增益
float64 kd          # 当前生效微分增益

float64 target      # 目标值
float64 feedback    # 当前反馈值
float64 error       # 误差 = target - feedback

float64 p_term      # P 项输出 = kp * error
float64 i_term      # I 项输出（积分累积，已含 ki，抗饱和后）
float64 d_term      # D 项输出（低通后微分 * kd）
float64 feedforward_term  # 前馈项（外部注入，如重力补偿力矩）

float64 output      # 总输出 = (P+I+D 限幅后) + feedforward（整体限幅后）
float64 integral    # 积分累积值（抗饱和回退后）
bool saturated      # 是否处于输出饱和（PID 顶格或含前馈后顶格）
