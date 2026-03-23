#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制成熟蓝莓的拉力分布和尺寸区间分布图
"""

import json
import matplotlib.pyplot as plt
import numpy as np
import os

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def load_data():
    """加载 JSON 数据文件"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    with open(os.path.join(script_dir, 'size.json'), 'r') as f:
        sizes = json.load(f)
    
    with open(os.path.join(script_dir, 'force.json'), 'r') as f:
        forces = json.load(f)
    
    return sizes, forces

def plot_force_distribution(forces):
    """绘制拉力分布直方图和箱线图"""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # 直方图
    ax1 = axes[0]
    counts, bins, patches = ax1.hist(forces, bins=8, edgecolor='black', alpha=0.7, color='steelblue')
    ax1.set_xlabel('Force (N)', fontsize=12)
    ax1.set_ylabel('Frequency', fontsize=12)
    ax1.set_title('Blueberry Picking Force Distribution', fontsize=14)
    ax1.grid(axis='y', alpha=0.3)
    
    # 在柱子上显示数值
    for count, x in zip(counts, bins[:-1]):
        if count > 0:
            ax1.text(x + (bins[1]-bins[0])/2, count + 0.05, str(int(count)), 
                    ha='center', va='bottom', fontsize=10)
    
    # 箱线图
    ax2 = axes[1]
    bp = ax2.boxplot(forces, vert=True, patch_artist=True,
                     labels=['Force (N)'])
    bp['boxes'][0].set_facecolor('lightblue')
    bp['medians'][0].set_color('red')
    bp['medians'][0].set_linewidth(2)
    ax2.set_ylabel('Force (N)', fontsize=12)
    ax2.set_title('Blueberry Picking Force Box Plot', fontsize=14)
    ax2.grid(axis='y', alpha=0.3)
    
    # 添加统计信息
    mean_val = np.mean(forces)
    std_val = np.std(forces)
    stats_text = f'Mean: {mean_val:.3f} N\nStd: {std_val:.3f} N\nN: {len(forces)}'
    ax2.text(1.25, mean_val, stats_text, fontsize=10, 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'force_distribution.png'), 
                dpi=150, bbox_inches='tight')
    plt.close()
    print("Force distribution plot saved: force_distribution.png")

def plot_size_distribution(sizes):
    """绘制尺寸区间分布图"""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # 计算区间
    min_size = min(sizes)
    max_size = max(sizes)
    bin_width = 2  # 每2mm一个区间
    bins = np.arange(int(min_size/2)*2, int(max_size/2)*2 + bin_width + 0.1, bin_width)
    
    # 直方图
    ax1 = axes[0]
    counts, bin_edges, patches = ax1.hist(sizes, bins=bins, edgecolor='black', alpha=0.7, color='forestgreen')
    ax1.set_xlabel('Size (mm)', fontsize=12)
    ax1.set_ylabel('Frequency', fontsize=12)
    ax1.set_title('Blueberry Size Distribution', fontsize=14)
    ax1.grid(axis='y', alpha=0.3)
    
    # 在柱子上显示数值
    for count, x in zip(counts, bin_edges[:-1]):
        if count > 0:
            ax1.text(x + bin_width/2, count + 0.05, str(int(count)), 
                    ha='center', va='bottom', fontsize=10)
    
    # 添加区间标签
    ax1.set_xticks(bin_edges[:-1] + bin_width/2)
    ax1.set_xticklabels([f'{int(x)}-{int(x+bin_width)}' for x in bin_edges[:-1]], rotation=45)
    
    # 饼图显示各区间占比
    ax2 = axes[1]
    non_zero_counts = counts[counts > 0]
    non_zero_labels = [f'{int(bin_edges[i])}-{int(bin_edges[i]+bin_width)} mm' 
                       for i in range(len(counts)) if counts[i] > 0]
    colors = plt.cm.YlGn(np.linspace(0.3, 0.9, len(non_zero_counts)))
    
    wedges, texts, autotexts = ax2.pie(non_zero_counts, labels=non_zero_labels, autopct='%1.1f%%',
                                        colors=colors, startangle=90)
    ax2.set_title('Blueberry Size Range Proportion', fontsize=14)
    
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'size_distribution.png'), 
                dpi=150, bbox_inches='tight')
    plt.close()
    print("Size distribution plot saved: size_distribution.png")

def print_statistics(sizes, forces):
    """打印统计数据"""
    print("\n" + "="*50)
    print("Statistics Summary")
    print("="*50)
    
    print(f"\nSize (mm):")
    print(f"  Count: {len(sizes)}")
    print(f"  Mean: {np.mean(sizes):.2f}")
    print(f"  Std: {np.std(sizes):.2f}")
    print(f"  Min: {min(sizes):.1f}")
    print(f"  Max: {max(sizes):.1f}")
    
    print(f"\nForce (N):")
    print(f"  Count: {len(forces)}")
    print(f"  Mean: {np.mean(forces):.3f}")
    print(f"  Std: {np.std(forces):.3f}")
    print(f"  Min: {min(forces):.3f}")
    print(f"  Max: {max(forces):.3f}")
    print("="*50)

def main():
    try:
        sizes, forces = load_data()
        plot_force_distribution(forces)
        plot_size_distribution(sizes)
        print_statistics(sizes, forces)
        print("\nAll plots generated successfully!")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
