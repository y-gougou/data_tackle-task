#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
create_sliding_windows.py - 滑动窗口分割模块

功能：
1. 加载预处理后的CSV数据
2. 使用滑动窗口分割时间序列
3. 划分训练集和测试集（时序划分：80%训练，20%测试）
4. 保存为numpy数组

滑动窗口参数（@ 20Hz）：
  窗口长度：100点 = 5秒
  步长：50点 = 2.5秒
  重叠率：50%

使用方法：
  python create_sliding_windows.py --data_path /path/to/processed_data.csv
"""

import pandas as pd
import numpy as np
import os
import pickle
from datetime import datetime
import argparse


class SlidingWindowCreator:
    """滑动窗口创建器"""

    # 特征列名（与preprocess_data.py保持一致）
    FEATURE_COLUMNS = [
        'x', 'y', 'z',
        'vx', 'vy', 'vz',
        'ax', 'ay', 'az',
        'gx', 'gy', 'gz',
        'voltage', 'current0', 'current1', 'current2'
    ]

    # 滑动窗口参数 @ 20Hz
    WINDOW_SIZE = 100  # 100点 = 5秒 @ 20Hz
    STEP_SIZE = 50     # 步长50点 = 2.5秒

    def __init__(self, data_path):
        """
        初始化滑动窗口创建器

        Args:
            data_path: 预处理后的CSV文件路径
        """
        self.data_path = data_path
        self.data_dir = os.path.dirname(data_path)

        # 加载的数据
        self.data = None
        self.windows_X = None
        self.windows_y = None

        # 划分后的数据
        self.X_train = None
        self.X_test = None
        self.y_train = None
        self.y_test = None

    def load_data(self):
        """加载预处理后的CSV数据"""
        print("加载数据...")

        if not os.path.exists(self.data_path):
            raise FileNotFoundError(f"数据文件不存在: {self.data_path}")

        self.data = pd.read_csv(self.data_path)
        print(f"  加载 {len(self.data)} 行数据")
        print(f"  列: {list(self.data.columns)}")

        # 检查必需列
        for col in self.FEATURE_COLUMNS + ['fault_label']:
            if col not in self.data.columns:
                raise ValueError(f"缺少必需列: {col}")

        # 标签分布
        if 'fault_label' in self.data.columns:
            print("\n标签分布:")
            label_counts = self.data['fault_label'].value_counts().sort_index()
            for label, count in label_counts.items():
                print(f"  标签 {int(label)}: {count} 样本")

        return self.data

    def load_normalization_params(self):
        """
        加载归一化参数

        Returns:
            norm_params: 归一化参数字典
        """
        # 查找归一化参数文件
        base_path = self.data_path.replace('.csv', '')
        norm_path = base_path + '_norm_params.pkl'

        if not os.path.exists(norm_path):
            raise FileNotFoundError(f"归一化参数文件不存在: {norm_path}")

        with open(norm_path, 'rb') as f:
            norm_params = pickle.load(f)

        print(f"加载归一化参数: {norm_path}")
        return norm_params

    def create_windows(self):
        """
        创建滑动窗口

        窗口形状: (window_size, n_features)
        标签: 取窗口最后一个点的标签
        """
        print(f"\n创建滑动窗口...")
        print(f"  窗口大小: {self.WINDOW_SIZE}")
        print(f"  步长: {self.STEP_SIZE}")
        print(f"  重叠率: {(self.WINDOW_SIZE - self.STEP_SIZE) / self.WINDOW_SIZE * 100:.1f}%")

        n_samples = len(self.data)
        n_features = len(self.FEATURE_COLUMNS)

        windows_X = []
        windows_y = []

        # 滑动窗口遍历
        start_idx = 0
        window_count = 0
        while start_idx + self.WINDOW_SIZE <= n_samples:
            # 提取窗口数据
            end_idx = start_idx + self.WINDOW_SIZE
            window_data = self.data[self.FEATURE_COLUMNS].iloc[start_idx:end_idx].values

            # 提取标签（使用窗口最后一个点的标签）
            window_labels = self.data['fault_label'].iloc[start_idx:end_idx]
            window_label = int(window_labels.mode()[0])

            # 检查标签一致性（同一窗口内应有相同标签）
            unique_labels = window_labels.unique()
            if len(unique_labels) > 1:
                print(f"  警告: 窗口跨越标签边界 [{start_idx}:{end_idx}], 标签: {unique_labels} -> 使用众数: {window_label}")

            windows_X.append(window_data)
            windows_y.append(window_label)

            window_count += 1
            start_idx += self.STEP_SIZE

        self.windows_X = np.array(windows_X)
        self.windows_y = np.array(windows_y)

        print(f"  创建了 {window_count} 个窗口")
        print(f"  X shape: {self.windows_X.shape}")
        print(f"  y shape: {self.windows_y.shape}")

        # 标签分布
        print("\n窗口标签分布:")
        unique, counts = np.unique(self.windows_y, return_counts=True)
        for label, count in zip(unique, counts):
            print(f"  标签 {label}: {count} 窗口 ({count/len(self.windows_y)*100:.1f}%)")

        return self.windows_X, self.windows_y

    def temporal_split(self, test_ratio=0.2):
        """
        时序划分训练集和测试集

        Args:
            test_ratio: 测试集比例（默认20%）

        Returns:
            X_train, X_test, y_train, y_test
        """
        print(f"\n时序划分（训练集 {(1-test_ratio)*100:.0f}%, 测试集 {test_ratio*100:.0f}%）...")

        n_windows = len(self.windows_X)
        split_idx = int(n_windows * (1 - test_ratio))

        # 训练集：前80%
        self.X_train = self.windows_X[:split_idx]
        self.y_train = self.windows_y[:split_idx]

        # 测试集：后20%
        self.X_test = self.windows_X[split_idx:]
        self.y_test = self.windows_y[split_idx:]

        print(f"  训练集: {len(self.X_train)} 窗口, shape {self.X_train.shape}")
        print(f"  测试集: {len(self.X_test)} 窗口, shape {self.X_test.shape}")

        # 各标签在训练集和测试集的分布
        print("\n训练集标签分布:")
        unique, counts = np.unique(self.y_train, return_counts=True)
        for label, count in zip(unique, counts):
            print(f"  标签 {label}: {count} ({count/len(self.y_train)*100:.1f}%)")

        print("\n测试集标签分布:")
        unique, counts = np.unique(self.y_test, return_counts=True)
        for label, count in zip(unique, counts):
            print(f"  标签 {label}: {count} ({count/len(self.y_test)*100:.1f}%)")

        return self.X_train, self.X_test, self.y_train, self.y_test

    def save_dataset(self, output_dir=None):
        """
        保存数据集为numpy数组

        Args:
            output_dir: 输出目录（默认为数据所在目录）

        Returns:
            output_files: 保存的文件路径列表
        """
        if output_dir is None:
            output_dir = self.data_dir

        print(f"\n保存数据集到: {output_dir}")

        # 生成输出文件名
        base_name = os.path.splitext(os.path.basename(self.data_path))[0]
        prefix = f"{base_name}_w{self.WINDOW_SIZE}_s{self.STEP_SIZE}"

        # 保存文件
        output_files = {}

        X_train_path = os.path.join(output_dir, f"X_train.npy")
        np.save(X_train_path, self.X_train)
        output_files['X_train'] = X_train_path
        print(f"  保存: {X_train_path}")

        X_test_path = os.path.join(output_dir, f"X_test.npy")
        np.save(X_test_path, self.X_test)
        output_files['X_test'] = X_test_path
        print(f"  保存: {X_test_path}")

        y_train_path = os.path.join(output_dir, f"y_train.npy")
        np.save(y_train_path, self.y_train)
        output_files['y_train'] = y_train_path
        print(f"  保存: {y_train_path}")

        y_test_path = os.path.join(output_dir, f"y_test.npy")
        np.save(y_test_path, self.y_test)
        output_files['y_test'] = y_test_path
        print(f"  保存: {y_test_path}")

        # 保存数据集信息
        info = {
            'window_size': self.WINDOW_SIZE,
            'step_size': self.STEP_SIZE,
            'n_features': len(self.FEATURE_COLUMNS),
            'feature_columns': self.FEATURE_COLUMNS,
            'X_train_shape': self.X_train.shape,
            'X_test_shape': self.X_test.shape,
            'y_train_shape': self.y_train.shape,
            'y_test_shape': self.y_test.shape,
            'train_label_dist': dict(zip(*np.unique(self.y_train, return_counts=True))),
            'test_label_dist': dict(zip(*np.unique(self.y_test, return_counts=True))),
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        info_path = os.path.join(output_dir, f"{prefix}_info.pkl")
        with open(info_path, 'wb') as f:
            pickle.dump(info, f)
        output_files['info'] = info_path
        print(f"  保存: {info_path}")

        print("\n数据集保存完成！")
        return output_files

    def process_pipeline(self):
        """
        完整处理流程

        Returns:
            X_train, X_test, y_train, y_test
        """
        print("\n" + "=" * 50)
        print("滑动窗口创建流程")
        print("=" * 50)

        # Step 1: 加载数据
        self.load_data()

        # Step 2: 创建滑动窗口
        self.create_windows()

        # Step 3: 时序划分
        self.temporal_split(test_ratio=0.2)

        # Step 4: 保存数据集
        self.save_dataset()

        print("\n" + "=" * 50)
        print("滑动窗口创建完成！")
        print("=" * 50)

        return self.X_train, self.X_test, self.y_train, self.y_test


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='滑动窗口分割')
    parser.add_argument('--data_path', type=str,
                        default=r"C:\Users\ypp\Desktop\数据集集合\0418\processed_20260419_163400.csv",
                       help='预处理后的CSV文件路径')
    parser.add_argument('--output_dir', type=str, default=None,
                       help='输出目录（默认为CSV所在目录）')
    parser.add_argument('--test_ratio', type=float, default=0.2,
                       help='测试集比例（默认0.2）')

    args = parser.parse_args()

    # 创建滑动窗口
    creator = SlidingWindowCreator(args.data_path)
    creator.process_pipeline()

    # 打印数据集摘要
    print("\n" + "=" * 50)
    print("数据集摘要")
    print("=" * 50)
    print(f"训练集 X shape: {creator.X_train.shape}")
    print(f"训练集 y shape: {creator.y_train.shape}")
    print(f"测试集 X shape: {creator.X_test.shape}")
    print(f"测试集 y shape: {creator.y_test.shape}")
    print(f"每个样本: {creator.WINDOW_SIZE} 时间步 × {len(creator.FEATURE_COLUMNS)} 特征")


if __name__ == '__main__':
    main()