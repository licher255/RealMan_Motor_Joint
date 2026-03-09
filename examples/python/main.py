"""
RealMan Motor Joint - Python Control Package

电机控制Python包主入口。

Usage:
    python main.py check             # 检查环境配置
    python main.py whj               # 运行WHJ示例
    python main.py kinco             # 运行Kinco示例
    python main.py dual              # 运行双电机示例
    python main.py interactive       # 交互式控制
    python main.py test [whj|kinco|dual|all]  # 运行测试

Examples:
    # 检查环境配置 (推荐先运行)
    python main.py check
    
    # WHJ基础控制
    python main.py whj
    
    # Kinco基础控制
    python main.py kinco
    
    # 双电机同步控制
    python main.py dual
    
    # 交互式控制
    python main.py interactive
    
    # 运行所有测试
    python main.py test
    
    # 运行WHJ测试
    python main.py test whj
"""

import sys
import os

# 确保可以导入本地模块
sys.path.insert(0, os.path.dirname(__file__))


def print_help():
    """打印帮助信息"""
    print(__doc__)
    print("\nModules:")
    print("  core/      - CAN driver and protocols")
    print("  drivers/   - Motor driver classes")
    print("  utils/     - Utility tools")
    print("  examples/  - Example programs")
    print("  tests/     - Test suites")


def run_whj():
    """运行WHJ示例"""
    # 直接运行 motion_controller 风格的 WHJ 控制
    from drivers.motion_controller import main as whj_main
    import sys
    # 默认使用 ID=7
    if len(sys.argv) <= 2:
        sys.argv = [sys.argv[0], "7"]
    whj_main()


def run_kinco():
    """运行Kinco示例"""
    from examples.basic_kinco import main
    main()


def run_dual():
    """运行双电机示例"""
    from examples.dual_motor_basic import main
    main()


def run_interactive():
    """运行交互式控制"""
    from examples.interactive_control import main
    main()


def run_check():
    """运行环境检查"""
    from check_setup import main as check_main
    check_main()


def run_test(target: str = "all"):
    """运行测试"""
    if target in ["all", "whj"]:
        print("\n" + "=" * 60)
        print("Running WHJ Tests")
        print("=" * 60)
        from tests.test_whj import main as whj_test
        whj_test()
    
    if target in ["all", "kinco"]:
        print("\n" + "=" * 60)
        print("Running Kinco Tests")
        print("=" * 60)
        from tests.test_kinco import main as kinco_test
        kinco_test()
    
    if target in ["all", "dual"]:
        print("\n" + "=" * 60)
        print("Running Dual Motor Tests")
        print("=" * 60)
        from tests.test_dual import main as dual_test
        dual_test()


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print_help()
        return
    
    command = sys.argv[1].lower()
    
    commands = {
        'check': run_check,
        'whj': run_whj,
        'kinco': run_kinco,
        'dual': run_dual,
        'interactive': run_interactive,
        'help': print_help,
        '-h': print_help,
        '--help': print_help,
    }
    
    if command == 'test':
        target = sys.argv[2] if len(sys.argv) > 2 else 'all'
        run_test(target)
    elif command in commands:
        commands[command]()
    else:
        print(f"Unknown command: {command}")
        print_help()


if __name__ == "__main__":
    main()
