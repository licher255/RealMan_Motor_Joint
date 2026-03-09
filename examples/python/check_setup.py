"""
Setup Check Script

检查Python控制包的配置是否正确，包括DLL路径检测。

Usage:
    python check_setup.py
"""

import os
import sys
import platform


def check_path(name, path):
    """检查路径是否存在"""
    exists = os.path.exists(path)
    status = "[OK]" if exists else "[MISSING]"
    print(f"  {status} {name}")
    print(f"      Path: {path}")
    return exists


def main():
    print("=" * 60)
    print("RealMan Motor Joint - Setup Check")
    print("=" * 60)
    
    # 系统信息
    print("\n[1] System Information:")
    print(f"  Platform: {platform.system()}")
    print(f"  Architecture: {platform.architecture()[0]}")
    print(f"  Python: {platform.python_version()}")
    print(f"  Script Directory: {os.path.dirname(os.path.abspath(__file__))}")
    
    # 检查关键目录
    print("\n[2] Directory Structure:")
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    dirs_to_check = [
        ("core/", os.path.join(base_dir, "core")),
        ("core/protocol/", os.path.join(base_dir, "core", "protocol")),
        ("drivers/", os.path.join(base_dir, "drivers")),
        ("utils/", os.path.join(base_dir, "utils")),
        ("examples/", os.path.join(base_dir, "examples")),
        ("tests/", os.path.join(base_dir, "tests")),
    ]
    
    all_dirs_ok = True
    for name, path in dirs_to_check:
        if not check_path(name, path):
            all_dirs_ok = False
    
    # 检查关键文件
    print("\n[3] Key Files:")
    files_to_check = [
        ("core/zlgcan_driver.py", os.path.join(base_dir, "core", "zlgcan_driver.py")),
        ("core/protocol/whj_protocol.py", os.path.join(base_dir, "core", "protocol", "whj_protocol.py")),
        ("core/protocol/kinco_protocol.py", os.path.join(base_dir, "core", "protocol", "kinco_protocol.py")),
        ("drivers/whj_driver.py", os.path.join(base_dir, "drivers", "whj_driver.py")),
        ("drivers/kinco_driver.py", os.path.join(base_dir, "drivers", "kinco_driver.py")),
        ("utils/dual_motor_manager.py", os.path.join(base_dir, "utils", "dual_motor_manager.py")),
    ]
    
    all_files_ok = True
    for name, path in files_to_check:
        if not check_path(name, path):
            all_files_ok = False
    
    # 检查DLL路径
    print("\n[4] DLL Path Detection:")
    is_64bit = platform.architecture()[0] == "64bit"
    arch = "x64" if is_64bit else "x86"
    print(f"  Architecture detected: {arch}")
    
    # 计算DLL路径 (从examples/python/到third_party/)
    project_root = os.path.dirname(os.path.dirname(base_dir))
    dll_path = os.path.join(project_root, "third_party", "zlgcan", arch, "zlgcan.dll")
    
    print(f"  Project Root: {project_root}")
    print(f"  Expected DLL: {dll_path}")
    
    if check_path("zlgcan.dll", dll_path):
        print("  [OK] DLL found!")
    else:
        print("  [MISSING] DLL not found!")
        # 尝试查找其他可能的位置
        alternative_paths = [
            os.path.join(project_root, "third_party", "zlgcan", "zlgcan.dll"),
            os.path.join(base_dir, "third_party", "zlgcan", arch, "zlgcan.dll"),
            r"C:\Program Files (x86)\ZCANPRO\zlgcan.dll",
        ]
        print("\n  Searching alternative paths:")
        for alt_path in alternative_paths:
            if os.path.exists(alt_path):
                print(f"    [FOUND] at: {alt_path}")
                break
        else:
            print("    [NOT FOUND] in alternative paths either")
    
    # 测试导入
    print("\n[5] Module Import Test:")
    modules_to_test = [
        "core",
        "core.protocol",
        "drivers",
        "utils",
    ]
    
    for module in modules_to_test:
        try:
            __import__(module)
            print(f"  [OK] {module}")
        except Exception as e:
            print(f"  [FAIL] {module}: {e}")
    
    # 总结
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    
    if all_dirs_ok and all_files_ok:
        print("[OK] All checks passed!")
        print("\nYou can now run:")
        print("  python main.py whj")
        print("  python main.py kinco")
        print("  python main.py dual")
    else:
        print("[FAIL] Some checks failed. Please fix the issues above.")
    
    print("=" * 60)


if __name__ == "__main__":
    main()
