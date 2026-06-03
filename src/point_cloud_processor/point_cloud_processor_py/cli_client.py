#!/usr/bin/env python3
"""
cli_client.py
=============
命令行客户端，用于手动触发节点提供的服务和动作。

用法示例
--------
# 1. 裁剪点云（服务调用，同步）
python3 cli_client.py crop \
    --input data/filtered.pcd \
    --output data/cropped_cloud.pcd \
    --min 0.1 -1.0 0.5 \
    --max 0.5 2.0 0.8

# 2. 预处理 + 泊松重建（动作调用，异步 + 进度）
python3 cli_client.py process \
    --input data/cropped_cloud.pcd \
    --output data/processed_mesh.ply

# 3. 全覆盖路径规划（动作调用）
python3 cli_client.py plan \
    --ply  data/processed_mesh.ply \
    --csv  outputs/coverage_path.csv \
    --gcode outputs/coverage_path.gcode

# 4. 完整流水线（依次调用上述三步）
python3 cli_client.py pipeline \
    --input data/filtered.pcd
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from point_cloud_processor.srv import CropPointCloud
from point_cloud_processor.action import ProcessPointCloud, PlanCoveragePath


# ─────────────────────────────────────────────────────────────
#  辅助：动作客户端同步调用
# ─────────────────────────────────────────────────────────────
def call_action(node: Node, client: ActionClient, goal, action_name: str):
    """发送 Goal，打印 Feedback，等待 Result，返回 Result 对象。"""
    node.get_logger().info(f"[Client] 发送动作 Goal → {action_name}")

    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error(f"动作服务器 {action_name} 不可用！")
        return None

    send_future = client.send_goal_async(
        goal,
        feedback_callback=lambda fb: _print_feedback(node, fb)
    )
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()

    if not goal_handle.accepted:
        node.get_logger().error("Goal 被拒绝！")
        return None

    node.get_logger().info("Goal 已接受，等待结果...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    return result_future.result().result


def _print_feedback(node, feedback_msg):
    fb = feedback_msg.feedback
    bar_len = 30
    filled = int(fb.progress / 100.0 * bar_len)
    bar = "█" * filled + "░" * (bar_len - filled)
    node.get_logger().info(
        f"  [{bar}] {fb.progress:5.1f}%  {fb.stage_description}"
    )


# ─────────────────────────────────────────────────────────────
#  子命令：crop
# ─────────────────────────────────────────────────────────────
def cmd_crop(node, args):
    client = node.create_client(CropPointCloud, 'crop_point_cloud')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("服务 /crop_point_cloud 不可用！")
        return 1

    req = CropPointCloud.Request()
    req.input_pcd_path  = args.input
    req.output_pcd_path = args.output
    req.min_bound       = list(args.min)
    req.max_bound       = list(args.max)

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()

    if resp.success:
        node.get_logger().info(
            f"✅ 裁剪成功！点数: {resp.point_count}，"
            f"输出: {resp.output_pcd_path}"
        )
        return 0
    else:
        node.get_logger().error(f"❌ 裁剪失败: {resp.message}")
        return 1


# ─────────────────────────────────────────────────────────────
#  子命令：process
# ─────────────────────────────────────────────────────────────
def cmd_process(node, args):
    client = ActionClient(node, ProcessPointCloud, 'process_point_cloud')
    goal = ProcessPointCloud.Goal()
    goal.input_pcd_path  = args.input
    goal.output_ply_path = args.output

    result = call_action(node, client, goal, 'process_point_cloud')
    if result is None:
        return 1

    if result.success:
        node.get_logger().info(
            f"✅ 预处理完成！点数: {result.point_count}，"
            f"面数: {result.face_count}，输出: {result.output_ply_path}"
        )
        return 0
    else:
        node.get_logger().error(f"❌ 预处理失败: {result.message}")
        return 1


# ─────────────────────────────────────────────────────────────
#  子命令：plan
# ─────────────────────────────────────────────────────────────
def cmd_plan(node, args):
    client = ActionClient(node, PlanCoveragePath, 'plan_coverage_path')
    goal = PlanCoveragePath.Goal()
    goal.input_ply_path    = args.ply
    goal.output_csv_path   = args.csv   or ''
    goal.output_gcode_path = args.gcode or ''

    result = call_action(node, client, goal, 'plan_coverage_path')
    if result is None:
        return 1

    if result.success:
        node.get_logger().info(
            f"✅ 路径规划完成！\n"
            f"   路径点数  : {result.point_count}\n"
            f"   总长度    : {result.total_length_m:.3f} m\n"
            f"   理论覆盖率: {result.coverage_percent:.1f}%\n"
            f"   CSV       : {result.output_csv_path}\n"
            f"   G-code    : {result.output_gcode_path}"
        )
        return 0
    else:
        node.get_logger().error(f"❌ 路径规划失败: {result.message}")
        return 1


# ─────────────────────────────────────────────────────────────
#  子命令：pipeline（依次执行全部三步）
# ─────────────────────────────────────────────────────────────
def cmd_pipeline(node, args):
    crop_args = argparse.Namespace(
        input=args.input,
        output=args.cropped or 'data/cropped_cloud.pcd',
        min=args.min or [0.1, -1.0, 0.5],
        max=args.max or [0.5,  2.0, 0.8]
    )
    if cmd_crop(node, crop_args) != 0:
        return 1

    proc_args = argparse.Namespace(
        input=crop_args.output,
        output=args.ply or 'data/processed_mesh.ply'
    )
    if cmd_process(node, proc_args) != 0:
        return 1

    plan_args = argparse.Namespace(
        ply=proc_args.output,
        csv=args.csv   or 'outputs/coverage_path.csv',
        gcode=args.gcode or 'outputs/coverage_path.gcode'
    )
    return cmd_plan(node, plan_args)


# ─────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='point_cloud_processor CLI 客户端')
    sub = parser.add_subparsers(dest='command', required=True)

    # crop
    p_crop = sub.add_parser('crop', help='裁剪点云（服务）')
    p_crop.add_argument('--input',  required=True)
    p_crop.add_argument('--output', required=True)
    p_crop.add_argument('--min', nargs=3, type=float, default=[0.1, -1.0, 0.5])
    p_crop.add_argument('--max', nargs=3, type=float, default=[0.5,  2.0, 0.8])

    # process
    p_proc = sub.add_parser('process', help='预处理 + 重建（动作）')
    p_proc.add_argument('--input',  required=True)
    p_proc.add_argument('--output', required=True)

    # plan
    p_plan = sub.add_parser('plan', help='全覆盖路径规划（动作）')
    p_plan.add_argument('--ply',   required=True)
    p_plan.add_argument('--csv',   default='')
    p_plan.add_argument('--gcode', default='')

    # pipeline
    p_pipe = sub.add_parser('pipeline', help='完整流水线（三步连续执行）')
    p_pipe.add_argument('--input',   required=True, help='原始 PCD 文件路径')
    p_pipe.add_argument('--cropped', default='data/cropped_cloud.pcd')
    p_pipe.add_argument('--ply',     default='data/processed_mesh.ply')
    p_pipe.add_argument('--csv',     default='outputs/coverage_path.csv')
    p_pipe.add_argument('--gcode',   default='outputs/coverage_path.gcode')
    p_pipe.add_argument('--min', nargs=3, type=float, default=None)
    p_pipe.add_argument('--max', nargs=3, type=float, default=None)

    args = parser.parse_args()

    rclpy.init()
    node = Node('point_cloud_processor_cli_client')

    dispatch = {
        'crop':     cmd_crop,
        'process':  cmd_process,
        'plan':     cmd_plan,
        'pipeline': cmd_pipeline,
    }

    exit_code = dispatch[args.command](node, args)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
