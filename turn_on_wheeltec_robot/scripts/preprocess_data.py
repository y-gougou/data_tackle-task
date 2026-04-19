#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
preprocess_data.py - 数据预处理模块

功能：
1. 加载CSV数据（支持多文件合并）
2. 时间戳同步（对 low-frequency 通道如 current 进行插值对齐）
3. 缺失值处理（线性插值）
4. 异常值剔除（3σ原则）
5. 数据归一化（Min-Max）
6. 保存处理后数据及归一化参数

采样率：20Hz（由底盘控制器决定）
滑动窗口参数：
  窗口长度：100点 = 5秒 @ 20Hz
  步长：50点 = 2.5秒
  重叠率：50%

时间同步说明：
  CSV 中包含独立时间戳字段：odom_time, imu_time, voltage_time, current_time
  由于 /current_data 频率较低（约5-6Hz），需要对其他通道进行插值对齐

使用方法：
  python preprocess_data.py --data_dir /path/to/log --pattern '*.csv'
  python preprocess_data.py --data_dir /path/to/log --pattern '*.csv' --no_sync  # 跳过时间同步（兼容旧数据）
"""

import pandas as pd
import numpy as np
import os
import glob
import pickle
from datetime import datetime
import argparse


class MultiChannelNormalizer:
    """
    多通道归一化器 - 针对不同类型通道使用不同归一化方法

    电压（单极性）→ Min-Max [0,1]
    位置/速度/加速度/角速度/电流（有正有负）→ 对称 [-1,1]
    """

    # 通道分组
    CHANNEL_GROUPS = {
        'position': ['x', 'y', 'z'],
        'velocity': ['vx', 'vy', 'vz'],
        'acceleration': ['ax', 'ay', 'az'],
        'gyro': ['gx', 'gy', 'gz'],
        'voltage': ['voltage'],
        'current': ['current0', 'current1', 'current2']
    }

    # 单极性通道（始终为正）
    UNIPOLAR_CHANNELS = ['voltage']

    def __init__(self):
        self.normalizers = {}
        self.feature_columns = None
        self.method = None

    def fit(self, df, method='symmetric'):
        """
        计算归一化参数

        Args:
            df: 原始数据DataFrame
            method: 'symmetric'（全用[-1,1]）或 'mixed'（电压用[0,1]，其他用[-1,1]）
        """
        self.method = method
        self.feature_columns = [
            'x', 'y', 'z',
            'vx', 'vy', 'vz',
            'ax', 'ay', 'az',
            'gx', 'gy', 'gz',
            'voltage', 'current0', 'current1', 'current2'
        ]

        print(f"\n多通道归一化配置:")
        print(f"  方法: {method}")

        for col in self.feature_columns:
            if col in df.columns:
                data = df[col].values
                max_abs = np.abs(data).max()

                if method == 'mixed' and col in self.UNIPOLAR_CHANNELS:
                    # 电压：Min-Max [0,1]
                    min_val = data.min()
                    max_val = data.max()
                    self.normalizers[col] = {
                        'type': 'minmax',
                        'min': min_val,
                        'max': max_val
                    }
                    print(f"  {col:12s}: Min-Max [0,1] (范围: {min_val:.2f}~{max_val:.2f})")
                else:
                    # 其他通道：对称 [-1,1]
                    self.normalizers[col] = {
                        'type': 'symmetric',
                        'max_abs': max_abs if max_abs > 1e-8 else 1.0
                    }
                    print(f"  {col:12s}: 对称 [-1,1] (max_abs: {max_abs:.4f})")

    def transform(self, df):
        """应用归一化"""
        df_norm = df.copy()

        for col in self.feature_columns:
            if col not in df.columns or col not in self.normalizers:
                continue

            params = self.normalizers[col]

            if params['type'] == 'minmax':
                val_range = params['max'] - params['min']
                if val_range > 1e-8:
                    df_norm[col] = (df[col] - params['min']) / val_range
                else:
                    df_norm[col] = 0.0
            else:
                max_abs = params['max_abs']
                if max_abs > 1e-8:
                    df_norm[col] = df[col] / max_abs
                    df_norm[col] = np.clip(df_norm[col], -1.0, 1.0)
                else:
                    df_norm[col] = 0.0

        return df_norm

    def fit_transform(self, df, method='symmetric'):
        """拟合并转换"""
        self.fit(df, method)
        return self.transform(df)

    def inverse_transform(self, df_norm):
        """反归一化（用于恢复原始尺度）"""
        df = df_norm.copy()

        for col in self.feature_columns:
            if col not in df.columns or col not in self.normalizers:
                continue

            params = self.normalizers[col]

            if params['type'] == 'minmax':
                val_range = params['max'] - params['min']
                df[col] = df_norm[col] * val_range + params['min']
            else:
                df[col] = df_norm[col] * params['max_abs']

        return df

    def get_params(self):
        """获取归一化参数字典（用于保存）"""
        return self.normalizers.copy()


class DataPreprocessor:
    """数据预处理器"""

    # 特征列名（不含时间戳字段）
    FEATURE_COLUMNS = [
        'x', 'y', 'z',
        'vx', 'vy', 'vz',
        'ax', 'ay', 'az',
        'gx', 'gy', 'gz',
        'voltage', 'current0', 'current1', 'current2'
    ]

    # 时间戳字段
    TIMESTAMP_COLUMNS = ['timestamp', 'odom_time', 'imu_time', 'voltage_time', 'current_time']

    # 需要同步的通道（低频率通道）
    SYNC_COLUMNS = ['current0', 'current1', 'current2']

    # 故障标签名称映射
    LABEL_NAMES = {
        0: 'normal',
        1: 'drive_fault',
        2: 'hub_loss',
        3: 'shaft_eccentric',
        4: 'voltage_low'
    }

    def __init__(self, data_dir):
        """
        初始化预处理器

        Args:
            data_dir: CSV数据目录路径
        """
        self.data_dir = data_dir
        self.raw_data = None
        self.processed_data = None

        # 归一化器
        self.multi_normalizer = MultiChannelNormalizer()

    def load_data(self, filename_pattern='*.csv'):
        """
        加载所有CSV文件

        Args:
            filename_pattern: 文件名模式，如 '*.csv' 或 'normal_*.csv'

        Returns:
            raw_data: 合并后的原始DataFrame
        """
        pattern = os.path.join(self.data_dir, filename_pattern)
        files = glob.glob(pattern)

        if not files:
            raise FileNotFoundError(f"未找到匹配的文件: {pattern}")

        print(f"找到 {len(files)} 个文件:")
        for f in files:
            print(f"  - {os.path.basename(f)}")

        # 读取并合并所有CSV
        dfs = []
        for filepath in files:
            df = pd.read_csv(filepath)
            print(f"  加载 {os.path.basename(filepath)}: {len(df)} 行")
            dfs.append(df)

        self.raw_data = pd.concat(dfs, ignore_index=True)
        print(f"\n总共加载 {len(self.raw_data)} 行数据")

        return self.raw_data

    def sync_timestamps(self):
        """
        时间戳同步：将 low-frequency 通道（current）插值到 odom 时间轴

        由于 /current_data 频率较低（约5-6Hz），需要对电流数据进行插值，
        使其与 /odom（20Hz）的时间轴对齐
        """
        print("时间戳同步处理...")

        # 检查是否有独立时间戳字段
        has_timestamp_cols = all(col in self.raw_data.columns for col in self.TIMESTAMP_COLUMNS)

        if not has_timestamp_cols:
            print("  警告: 未找到独立时间戳字段，跳过时间同步")
            print("  （可能是旧格式数据，当前采集的数据已包含时间戳字段）")
            return

        # 检查 current 数据的时间戳分布
        current_times = self.raw_data['current_time'].astype(float).values
        odom_times = self.raw_data['timestamp'].astype(float).values

        # 计算平均间隔
        current_intervals = np.diff(current_times)
        current_intervals = current_intervals[current_intervals > 0]  # 过滤异常值
        if len(current_intervals) > 0:
            avg_interval = np.mean(current_intervals)
            print(f"  /current_data 平均间隔: {avg_interval*1000:.1f} ms")
            print(f"  /odom 间隔: 50 ms (20Hz)")

            # 如果间隔差异不大，说明已经同步过了
            if avg_interval < 0.06:  # 60ms 以下认为频率接近
                print("  电流数据频率较高，跳过插值同步")
                return

        # 对电流通道进行插值
        print("  对电流通道进行线性插值...")

        for col in self.SYNC_COLUMNS:
            if col in self.raw_data.columns:
                # 创建临时 DataFrame 用于插值
                temp_df = self.raw_data[['timestamp', 'current_time', col]].copy()
                temp_df = temp_df.dropna(subset=['current_time', col])
                temp_df = temp_df.sort_values('current_time')

                if len(temp_df) < 2:
                    print(f"    {col}: 数据不足，跳过插值")
                    continue

                # 使用 numpy 插值
                current_time_vals = temp_df['current_time'].values.astype(float)
                current_val_vals = temp_df[col].values.astype(float)

                # 对 odom 时间进行插值
                interpolated_vals = np.interp(
                    odom_times,
                    current_time_vals,
                    current_val_vals
                )

                # 更新原始数据
                self.raw_data[col] = interpolated_vals

                print(f"    {col}: 插值完成")

        print("  ✓ 时间戳同步完成")

    def check_data_quality(self):
        """
        检查数据质量
        """
        print("\n" + "=" * 50)
        print("数据质量检查")
        print("=" * 50)

        # 1. 检查缺失值
        missing = self.raw_data.isnull().sum()
        if missing.sum() > 0:
            print("\n缺失值统计:")
            print(missing[missing > 0])
        else:
            print("\n✓ 无缺失值")

        # 2. 检查时间戳连续性（基于 odom 时间戳）
        if 'timestamp' in self.raw_data.columns:
            timestamps = self.raw_data['timestamp'].astype(float).values
            dt = np.diff(timestamps)
            expected_dt = 0.05  # 20Hz期望间隔50ms

            dt_anomalies = np.sum(np.abs(dt - expected_dt) > 0.01)  # 允许10ms误差
            print(f"\n时间戳连续性检查:")
            print(f"  期望间隔: {expected_dt*1000:.1f} ms")
            print(f"  实际间隔: {np.mean(dt)*1000:.2f} ± {np.std(dt)*1000:.2f} ms")
            print(f"  异常点数: {dt_anomalies} ({dt_anomalies/len(dt)*100:.2f}%)")

            if len(dt) > 0 and dt_anomalies / len(dt) > 0.1:
                print("⚠ 警告: 超过10%的数据点时间间隔异常")
            else:
                print("✓ 时间戳连续性良好")

        # 3. 故障标签分布
        print(f"\n故障标签分布:")
        if 'fault_label' in self.raw_data.columns:
            label_counts = self.raw_data['fault_label'].value_counts().sort_index()
            for label, count in label_counts.items():
                name = self.LABEL_NAMES.get(int(label), f"未知({label})")
                print(f"  {int(label)} ({name}): {count} 行")
        else:
            print("  (无fault_label列)")

        # 4. 数据范围统计
        print(f"\n数据范围统计:")
        for col in self.FEATURE_COLUMNS[:6]:
            if col in self.raw_data.columns:
                min_val = self.raw_data[col].min()
                max_val = self.raw_data[col].max()
                print(f"  {col}: [{min_val:.4f}, {max_val:.4f}]")

        print("=" * 50 + "\n")

        return {
            'missing_count': missing.sum(),
            'dt_anomaly_ratio': dt_anomalies / len(dt) if len(dt) > 0 else 0,
            'total_rows': len(self.raw_data)
        }

    def handle_missing_values(self):
        """
        处理缺失值：线性插值
        """
        print("处理缺失值...")

        if self.processed_data is None:
            self.processed_data = self.raw_data.copy()

        # 对每个特征列进行线性插值
        for col in self.FEATURE_COLUMNS:
            if col in self.processed_data.columns:
                if self.processed_data[col].isnull().sum() > 0:
                    self.processed_data[col] = self.processed_data[col].interpolate(
                        method='linear',
                        limit_direction='both'
                    )

        # 再次检查并删除仍有缺失值的行
        remaining_missing = 0
        for col in self.FEATURE_COLUMNS:
            if col in self.processed_data.columns:
                remaining_missing += self.processed_data[col].isnull().sum()

        if remaining_missing > 0:
            self.processed_data = self.processed_data.dropna(subset=self.FEATURE_COLUMNS)
            print(f"  删除了 {remaining_missing} 个仍有缺失的样本")
        else:
            print("  ✓ 所有缺失值已通过插值填充")

        print(f"  处理后数据量: {len(self.processed_data)} 行\n")

    def remove_outliers(self, sigma_threshold=3):
        """
        剔除异常值（3σ原则）

        Args:
            sigma_threshold: 标准差倍数阈值
        """
        print(f"剔除异常值（{sigma_threshold}σ原则）...")

        # 创建掩码，标记正常数据
        mask = pd.Series([True] * len(self.processed_data))

        for col in self.FEATURE_COLUMNS:
            if col not in self.processed_data.columns:
                continue

            mean = self.processed_data[col].mean()
            std = self.processed_data[col].std()

            if std < 1e-8:  # 避免除零
                continue

            # 计算异常值掩码
            col_mask = (
                (self.processed_data[col] >= mean - sigma_threshold * std) &
                (self.processed_data[col] <= mean + sigma_threshold * std)
            )

            outliers_count = (~col_mask).sum()
            if outliers_count > 0:
                print(f"  {col}: 发现 {outliers_count} 个异常值 ({outliers_count/len(self.processed_data)*100:.2f}%)")

            mask &= col_mask

        removed_count = (~mask).sum()
        self.processed_data = self.processed_data[mask].reset_index(drop=True)

        print(f"  共移除 {removed_count} 个异常样本")
        print(f"  处理后数据量: {len(self.processed_data)} 行\n")

    def normalize(self, method='symmetric', fit_only=False):
        """
        数据归一化

        Args:
            method: 'symmetric'（[-1,1]推荐）或 'mixed'（电压[0,1]其他[-1,1]）
            fit_only: True则只计算参数不进行归一化（用于测试集）
        """
        print(f"数据归一化（{method}）...")

        if fit_only:
            # 只计算参数
            self.multi_normalizer.fit(self.processed_data, method=method)
            print("  已计算归一化参数（待应用于测试集）")
        else:
            # 完整归一化
            self.processed_data = self.multi_normalizer.fit_transform(
                self.processed_data, method=method
            )

            # 打印归一化后统计
            print("\n归一化后数据统计:")
            for col in self.FEATURE_COLUMNS[:8]:
                if col in self.processed_data.columns:
                    data = self.processed_data[col]
                    print(f"  {col:12s}: min={data.min():7.4f}, max={data.max():7.4f}")

            print("  ✓ 归一化完成")

        print()

    def process_pipeline(self, remove_outliers_flag=True, sync_flag=True, skip_first=0):
        """
        完整预处理流程

        Args:
            remove_outliers_flag: 是否剔除异常值
            sync_flag: 是否进行时间戳同步
            skip_first: 跳过前N秒数据（用于丢弃启动阶段的0值），默认0

        Returns:
            processed_data: 处理后的DataFrame
        """
        print("\n" + "=" * 50)
        print("开始数据预处理流程")
        print("=" * 50)

        # Step 1: 质量检查
        self.check_data_quality()

        # Step 2: 初始化处理数据
        self.processed_data = self.raw_data.copy()

        # Step 2.5: 跳过前N秒数据（20Hz采样率）
        if skip_first > 0:
            rows_to_skip = int(skip_first * 20)  # 20Hz = 20行/秒
            original_len = len(self.processed_data)
            self.processed_data = self.processed_data.iloc[rows_to_skip:].reset_index(drop=True)
            print(f"\n跳过前 {skip_first} 秒数据 ({rows_to_skip} 行)")
            print(f"  原始: {original_len} 行 → 跳过: {rows_to_skip} 行 → 剩余: {len(self.processed_data)} 行")

        # Step 3: 时间戳同步（可选）
        if sync_flag:
            self.sync_timestamps()
        else:
            print("跳过时间戳同步步骤\n")

        # Step 4: 处理缺失值
        self.handle_missing_values()

        # Step 5: 剔除异常值
        if remove_outliers_flag:
            self.remove_outliers(sigma_threshold=3)
        else:
            print("跳过异常值剔除步骤\n")

        # Step 6: 归一化（默认使用对称归一化[-1,1]，保留正负信息）
        self.normalize(method='symmetric')

        print("=" * 50)
        print("预处理完成！")
        print("=" * 50 + "\n")

        return self.processed_data

    def save_processed_data(self, output_path=None):
        """
        保存处理后的数据和归一化参数

        Args:
            output_path: 输出CSV路径（不含后缀）

        Returns:
            csv_path: 处理后数据的路径
        """
        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(self.data_dir, f"processed_{timestamp}")

        # 保存处理后数据
        csv_path = output_path + '.csv'
        self.processed_data.to_csv(csv_path, index=False)
        print(f"处理后数据已保存: {csv_path}")

        # 保存归一化参数
        norm_params_path = output_path + '_norm_params.pkl'
        norm_params = {
            'normalizers': self.multi_normalizer.get_params(),
            'feature_columns': self.FEATURE_COLUMNS,
            'label_names': self.LABEL_NAMES,
            'method': self.multi_normalizer.method
        }
        with open(norm_params_path, 'wb') as f:
            pickle.dump(norm_params, f)
        print(f"归一化参数已保存: {norm_params_path}")

        # 保存统计信息
        stats = {
            'total_rows': len(self.processed_data),
            'feature_columns': self.FEATURE_COLUMNS,
            'n_features': len(self.FEATURE_COLUMNS),
            'label_distribution': self.processed_data['fault_label'].value_counts().to_dict() if 'fault_label' in self.processed_data.columns else {},
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        stats_path = output_path + '_stats.pkl'
        with open(stats_path, 'wb') as f:
            pickle.dump(stats, f)
        print(f"统计信息已保存: {stats_path}")

        return csv_path


def main():
    """测试预处理模块"""
    parser = argparse.ArgumentParser(description='数据预处理')
    parser.add_argument('--data_dir', type=str,
                       default=r"C:\Users\ypp\Desktop\数据集集合\0418",
                       help='CSV数据目录')
    parser.add_argument('--pattern', type=str, default='*.csv',
                       help='文件名模式')
    parser.add_argument('--output', type=str, default=None,
                       help='输出路径（不含后缀）')
    parser.add_argument('--no_outlier_removal', action='store_true',
                       help='跳过异常值剔除步骤')
    parser.add_argument('--no_sync', action='store_true',
                       help='跳过时间戳同步步骤（兼容旧数据格式）')
    parser.add_argument('--normalize', type=str, default='symmetric',
                       choices=['symmetric', 'mixed'],
                       help='归一化方法: symmetric=[-1,1]推荐, mixed=电压[0,1]其他[-1,1]')
    parser.add_argument('--skip_first', type=float, default=0,
                       help='跳过前N秒数据（用于丢弃启动阶段的0值，默认0）')

    args = parser.parse_args()

    # 创建预处理器
    preprocessor = DataPreprocessor(args.data_dir)

    # 加载数据
    preprocessor.load_data(args.pattern)

    # 执行预处理流程
    preprocessor.process_pipeline(
        remove_outliers_flag=not args.no_outlier_removal,
        sync_flag=not args.no_sync,
        skip_first=args.skip_first
    )

    # 保存处理后数据
    output_path = args.output
    preprocessor.save_processed_data(output_path)


if __name__ == '__main__':
    main()
