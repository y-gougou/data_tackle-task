#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
validate_dataset.py - 数据集验证与可视化模块

功能：
1. 数据集完整性检查（shape, dtype, 缺失值）
2. 标签分布统计
3. 特征值范围验证
4. 时序样本可视化（每类故障抽取样本）
5. 特征分布直方图
6. PCA降维可视化
7. 生成验证报告

使用方法：
  python validate_dataset.py --data_dir /path/to/log
"""

import os
import numpy as np
import pickle
from datetime import datetime
import argparse
import sys

try:
    import matplotlib
    matplotlib.use('Agg')  # 无GUI后端
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    from sklearn.decomposition import PCA
    from sklearn.manifold import TSNE
    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False


class DatasetValidator:
    """数据集验证器"""

    # 特征列名
    FEATURE_COLUMNS = [
        'x', 'y', 'z',
        'vx', 'vy', 'vz',
        'ax', 'ay', 'az',
        'gx', 'gy', 'gz',
        'voltage', 'current0', 'current1', 'current2'
    ]

    # 故障标签名称
    LABEL_NAMES = {
        0: 'normal',
        1: 'drive_fault',
        2: 'hub_loss',
        3: 'shaft_eccentric',
        4: 'voltage_low'
    }

    # 特征类型分组（用于可视化）
    FEATURE_GROUPS = {
        'position': ['x', 'y', 'z'],
        'velocity': ['vx', 'vy', 'vz'],
        'acceleration': ['ax', 'ay', 'az'],
        'gyro': ['gx', 'gy', 'gz'],
        'voltage': ['voltage'],
        'current': ['current0', 'current1', 'current2']
    }

    def __init__(self, data_dir):
        """
        初始化验证器

        Args:
            data_dir: 数据集目录（包含X_train.npy等文件）
        """
        self.data_dir = data_dir

        # 加载的数据
        self.X_train = None
        self.X_test = None
        self.y_train = None
        self.y_test = None
        self.info = None

        # 验证结果
        self.validation_results = {}

    def load_dataset(self):
        """加载数据集"""
        print("加载数据集...")

        files = {
            'X_train': 'X_train.npy',
            'X_test': 'X_test.npy',
            'y_train': 'y_train.npy',
            'y_test': 'y_test.npy'
        }

        for attr, filename in files.items():
            filepath = os.path.join(self.data_dir, filename)
            if not os.path.exists(filepath):
                raise FileNotFoundError(f"文件不存在: {filepath}")
            data = np.load(filepath)
            setattr(self, attr, data)
            print(f"  {filename}: shape={data.shape}, dtype={data.dtype}")

        # 加载info文件
        info_files = [
            os.path.join(self.data_dir, f) for f in os.listdir(self.data_dir)
            if f.endswith('_info.pkl')
        ]
        if info_files:
            with open(info_files[0], 'rb') as f:
                self.info = pickle.load(f)
            print(f"\n数据集信息:")
            print(f"  窗口大小: {self.info.get('window_size')}")
            print(f"  步长: {self.info.get('step_size')}")
            print(f"  特征数: {self.info.get('n_features')}")
        else:
            print("\n警告: 未找到info文件")

        print()

        return self.X_train, self.X_test, self.y_train, self.y_test

    def check_integrity(self):
        """
        检查数据集完整性

        Returns:
            results: 验证结果字典
        """
        print("=" * 50)
        print("数据集完整性检查")
        print("=" * 50)

        results = {}

        # 1. Shape检查
        print("\n1. Shape检查:")
        print(f"   X_train: {self.X_train.shape}")
        print(f"   X_test:  {self.X_test.shape}")
        print(f"   y_train: {self.y_train.shape}")
        print(f"   y_test:  {self.y_test.shape}")

        expected_window_size = 100
        expected_n_features = 15

        shape_ok = (
            self.X_train.shape[1] == expected_window_size and
            self.X_train.shape[2] == expected_n_features and
            self.X_test.shape[1] == expected_window_size and
            self.X_test.shape[2] == expected_n_features
        )
        results['shape_ok'] = shape_ok
        print(f"   {'✓' if shape_ok else '✗'} Shape检查: {'通过' if shape_ok else '失败'}")

        # 2. Dtype检查
        print("\n2. Dtype检查:")
        dtype_ok = (
            np.issubdtype(self.X_train.dtype, np.floating) or
            np.issubdtype(self.X_train.dtype, np.float32) or
            np.issubdtype(self.X_train.dtype, np.float64)
        )
        results['dtype_ok'] = dtype_ok
        print(f"   X_train dtype: {self.X_train.dtype}")
        print(f"   {'✓' if dtype_ok else '✗'} Dtype检查: {'通过' if dtype_ok else '警告（非float类型）'}")

        # 3. 缺失值检查
        print("\n3. 缺失值检查:")
        X_train_nan = np.isnan(self.X_train).sum()
        X_test_nan = np.isnan(self.X_test).sum()
        y_train_nan = np.isnan(self.y_train).sum()
        y_test_nan = np.isnan(self.y_test).sum()

        nan_ok = (X_train_nan == 0 and X_test_nan == 0 and
                  y_train_nan == 0 and y_test_nan == 0)
        results['nan_ok'] = nan_ok

        print(f"   X_train NaN: {X_train_nan}")
        print(f"   X_test  NaN: {X_test_nan}")
        print(f"   y_train NaN: {y_train_nan}")
        print(f"   y_test  NaN: {y_test_nan}")
        print(f"   {'✓' if nan_ok else '✗'} 缺失值检查: {'通过' if nan_ok else '失败'}")

        # 4. 无穷值检查
        print("\n4. 无穷值检查:")
        X_train_inf = np.isinf(self.X_train).sum()
        X_test_inf = np.isinf(self.X_test).sum()

        inf_ok = (X_train_inf == 0 and X_test_inf == 0)
        results['inf_ok'] = inf_ok
        print(f"   X_train Inf: {X_train_inf}")
        print(f"   X_test  Inf: {X_test_inf}")
        print(f"   {'✓' if inf_ok else '✗'} 无穷值检查: {'通过' if inf_ok else '失败'}")

        # 5. 标签范围检查
        print("\n5. 标签范围检查:")
        unique_labels = np.unique(np.concatenate([self.y_train, self.y_test]))
        expected_labels = set(range(5))
        actual_labels = set(unique_labels.astype(int))

        labels_ok = actual_labels.issubset(expected_labels)
        results['labels_ok'] = labels_ok
        print(f"   唯一标签: {sorted(actual_labels)}")
        print(f"   期望标签: {sorted(expected_labels)}")
        print(f"   {'✓' if labels_ok else '✗'} 标签范围检查: {'通过' if labels_ok else '警告'}")
        for label in unique_labels:
            name = self.LABEL_NAMES.get(int(label), f"未知({label})")
            print(f"      标签 {int(label)}: {name}")

        # 6. 数值范围检查
        print("\n6. 数值范围检查（归一化后应都在[-1, 1]或[0, 1]）:")
        X_min = self.X_train.min()
        X_max = self.X_train.max()
        range_ok = (X_min >= -1.05 and X_max <= 1.05)
        results['range_ok'] = range_ok
        print(f"   X_train min: {X_min:.4f}")
        print(f"   X_train max: {X_max:.4f}")
        print(f"   {'✓' if range_ok else '⚠'} 数值范围检查: {'通过' if range_ok else '超出[-1,1]范围'}")

        self.validation_results['integrity'] = results
        return results

    def check_label_distribution(self):
        """
        检查标签分布

        Returns:
            distributions: 标签分布统计
        """
        print("\n" + "=" * 50)
        print("标签分布统计")
        print("=" * 50)

        distributions = {}

        # 训练集分布
        print("\n训练集:")
        train_unique, train_counts = np.unique(self.y_train, return_counts=True)
        train_dist = {}
        for label, count in zip(train_unique, train_counts):
            label_int = int(label)
            name = self.LABEL_NAMES.get(label_int, f"未知({label_int})")
            pct = count / len(self.y_train) * 100
            train_dist[label_int] = {'count': int(count), 'pct': pct}
            print(f"  标签 {label_int} ({name:15s}): {count:6d} 样本 ({pct:5.2f}%)")
        distributions['train'] = train_dist

        # 测试集分布
        print("\n测试集:")
        test_unique, test_counts = np.unique(self.y_test, return_counts=True)
        test_dist = {}
        for label, count in zip(test_unique, test_counts):
            label_int = int(label)
            name = self.LABEL_NAMES.get(label_int, f"未知({label_int})")
            pct = count / len(self.y_test) * 100
            test_dist[label_int] = {'count': int(count), 'pct': pct}
            print(f"  标签 {label_int} ({name:15s}): {count:6d} 样本 ({pct:5.2f}%)")
        distributions['test'] = test_dist

        # 训练/测试比例
        print(f"\n训练集: {len(self.y_train)} 样本 ({len(self.y_train)/(len(self.y_train)+len(self.y_test))*100:.1f}%)")
        print(f"测试集: {len(self.y_test)} 样本 ({len(self.y_test)/(len(self.y_train)+len(self.y_test))*100:.1f}%)")

        self.validation_results['distribution'] = distributions
        return distributions

    def plot_label_distribution(self, output_path=None):
        """绘制标签分布图"""
        if not HAS_MATPLOTLIB:
            print("\n警告: matplotlib未安装，跳过标签分布图绘制")
            return

        print("\n绘制标签分布图...")

        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        # 训练集
        train_unique, train_counts = np.unique(self.y_train, return_counts=True)
        labels = [self.LABEL_NAMES.get(int(l), f"未知({l})") for l in train_unique]
        colors = plt.cm.Set2(np.linspace(0, 1, len(train_unique)))

        axes[0].bar(labels, train_counts, color=colors)
        axes[0].set_title('Training Set Label Distribution', fontsize=12)
        axes[0].set_xlabel('Fault Type')
        axes[0].set_ylabel('Sample Count')
        axes[0].tick_params(axis='x', rotation=45)
        for i, (label, count) in enumerate(zip(labels, train_counts)):
            axes[0].text(i, count + 50, str(count), ha='center', va='bottom', fontsize=9)

        # 测试集
        test_unique, test_counts = np.unique(self.y_test, return_counts=True)
        labels = [self.LABEL_NAMES.get(int(l), f"未知({l})") for l in test_unique]

        axes[1].bar(labels, test_counts, color=colors)
        axes[1].set_title('Test Set Label Distribution', fontsize=12)
        axes[1].set_xlabel('Fault Type')
        axes[1].set_ylabel('Sample Count')
        axes[1].tick_params(axis='x', rotation=45)
        for i, (label, count) in enumerate(zip(labels, test_counts)):
            axes[1].text(i, count + 10, str(count), ha='center', va='bottom', fontsize=9)

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"  保存: {output_path}")
        else:
            plt.savefig(os.path.join(self.data_dir, 'label_distribution.png'),
                       dpi=150, bbox_inches='tight')
            print(f"  保存: label_distribution.png")

        plt.close()

    def plot_sample_timeseries(self, output_path=None):
        """绘制每类故障的时序样本"""
        if not HAS_MATPLOTLIB:
            print("\n警告: matplotlib未安装，跳过时序样本图绘制")
            return

        print("\n绘制时序样本图...")

        n_labels = len(self.LABEL_NAMES)
        fig, axes = plt.subplots(n_labels, 4, figsize=(16, 3 * n_labels))

        if n_labels == 1:
            axes = axes.reshape(1, -1)

        for label_idx in range(n_labels):
            # 找到该标签的一个样本
            mask = self.y_train == label_idx
            if not mask.any():
                continue

            sample_indices = np.where(mask)[0]
            sample_idx = sample_indices[0] if len(sample_indices) > 0 else None

            if sample_idx is None:
                continue

            sample = self.X_train[sample_idx]  # shape: (100, 15)

            label_name = self.LABEL_NAMES.get(label_idx, f"未知({label_idx})")

            # 绘制速度、加速度、角速度、电压的代表性通道
            plot_channels = [
                ('vx', 'Velocity X'),
                ('ax', 'Acceleration X'),
                ('gx', 'Gyro X'),
                ('voltage', 'Voltage')
            ]

            for col_idx, (channel, title) in enumerate(plot_channels):
                channel_idx = self.FEATURE_COLUMNS.index(channel)
                data = sample[:, channel_idx]

                axes[label_idx, col_idx].plot(data, linewidth=0.8)
                axes[label_idx, col_idx].set_title(f'{label_name} - {title}', fontsize=9)
                axes[label_idx, col_idx].set_ylim([-1.1, 1.1])
                axes[label_idx, col_idx].grid(True, alpha=0.3)

                if label_idx == n_labels - 1:
                    axes[label_idx, col_idx].set_xlabel('Time (10ms)')

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"  保存: {output_path}")
        else:
            plt.savefig(os.path.join(self.data_dir, 'sample_timeseries.png'),
                       dpi=150, bbox_inches='tight')
            print(f"  保存: sample_timeseries.png")

        plt.close()

    def plot_feature_distributions(self, output_path=None):
        """绘制特征分布直方图"""
        if not HAS_MATPLOTLIB:
            print("\n警告: matplotlib未安装，跳过特征分布图绘制")
            return

        print("\n绘制特征分布图...")

        n_features = len(self.FEATURE_COLUMNS)
        fig, axes = plt.subplots(4, 4, figsize=(16, 12))
        axes = axes.flatten()

        for i, col in enumerate(self.FEATURE_COLUMNS):
            if i >= len(axes):
                break

            channel_idx = i
            data = self.X_train[:, :, channel_idx].flatten()

            axes[i].hist(data, bins=50, alpha=0.7, edgecolor='black', linewidth=0.5)
            axes[i].set_title(f'{col}', fontsize=10)
            axes[i].set_xlabel('Value')
            axes[i].set_ylabel('Frequency')
            axes[i].grid(True, alpha=0.3)

        # 隐藏多余的子图
        for i in range(n_features, len(axes)):
            axes[i].set_visible(False)

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"  保存: {output_path}")
        else:
            plt.savefig(os.path.join(self.data_dir, 'feature_distributions.png'),
                       dpi=150, bbox_inches='tight')
            print(f"  保存: feature_distributions.png")

        plt.close()

    def plot_pca_visualization(self, output_path=None):
        """绘制PCA降维可视化"""
        if not HAS_SKLEARN or not HAS_MATPLOTLIB:
            print("\n警告: sklearn或matplotlib未安装，跳过PCA可视化")
            return

        print("\n绘制PCA可视化...")

        # 将3D数据展平为2D进行可视化
        n_samples = min(5000, len(self.X_train))  # 限制样本数加速
        X_flat = self.X_train[:n_samples].reshape(n_samples, -1)
        y_labels = self.y_train[:n_samples]

        # PCA降维到2D
        pca = PCA(n_components=2)
        X_pca = pca.fit_transform(X_flat)

        print(f"  PCA解释方差比: {pca.explained_variance_ratio_}")
        print(f"  总解释方差: {sum(pca.explained_variance_ratio_):.4f}")

        # 绘图
        fig, ax = plt.subplots(figsize=(10, 8))

        colors = plt.cm.Set1(np.linspace(0, 1, len(self.LABEL_NAMES)))
        for label_idx in range(len(self.LABEL_NAMES)):
            mask = y_labels == label_idx
            label_name = self.LABEL_NAMES.get(label_idx, f"未知({label_idx})")
            ax.scatter(X_pca[mask, 0], X_pca[mask, 1],
                      c=[colors[label_idx]], label=f'{label_idx}: {label_name}',
                      alpha=0.6, s=10)

        ax.set_xlabel(f'PC1 ({pca.explained_variance_ratio_[0]*100:.1f}%)')
        ax.set_ylabel(f'PC2 ({pca.explained_variance_ratio_[1]*100:.1f}%)')
        ax.set_title('PCA Visualization of Fault Types')
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"  保存: {output_path}")
        else:
            plt.savefig(os.path.join(self.data_dir, 'pca_visualization.png'),
                       dpi=150, bbox_inches='tight')
            print(f"  保存: pca_visualization.png")

        plt.close()

    def generate_report(self, output_path=None):
        """
        生成验证报告

        Returns:
            report: 报告文本
        """
        print("\n" + "=" * 50)
        print("生成验证报告")
        print("=" * 50)

        # 汇总检查结果
        integrity = self.validation_results.get('integrity', {})
        distribution = self.validation_results.get('distribution', {})

        report_lines = [
            "=" * 60,
            "数据集验证报告",
            "=" * 60,
            f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            "",
            "1. 数据集基本信息",
            "-" * 40,
            f"  训练集: {len(self.y_train)} 样本",
            f"  测试集: {len(self.y_test)} 样本",
            f"  总计:   {len(self.y_train) + len(self.y_test)} 样本",
            f"  窗口大小: {self.X_train.shape[1]} 时间步",
            f"  特征数量: {self.X_train.shape[2]} 特征",
            "",
            "2. 完整性检查结果",
            "-" * 40,
        ]

        check_items = [
            ('shape_ok', 'Shape检查'),
            ('dtype_ok', 'Dtype检查'),
            ('nan_ok', '缺失值检查'),
            ('inf_ok', '无穷值检查'),
            ('labels_ok', '标签范围检查'),
            ('range_ok', '数值范围检查'),
        ]

        all_passed = True
        for key, name in check_items:
            status = integrity.get(key, False)
            all_passed = all_passed and status
            report_lines.append(f"  [{'✓' if status else '✗'}] {name}: {'通过' if status else '失败'}")

        report_lines.extend([
            "",
            "3. 标签分布",
            "-" * 40,
        ])

        for split in ['train', 'test']:
            if split not in distribution:
                continue
            report_lines.append(f"  {split.capitalize()} set:")
            for label, info in sorted(distribution[split].items()):
                name = self.LABEL_NAMES.get(label, f"未知({label})")
                report_lines.append(
                    f"    标签 {label} ({name:15s}): "
                    f"{info['count']:6d} ({info['pct']:5.2f}%)"
                )
            report_lines.append("")

        report_lines.extend([
            "4. 结论",
            "-" * 40,
        ])

        if all_passed:
            report_lines.append("  ✓ 数据集验证通过，可以用于模型训练")
        else:
            report_lines.append("  ⚠ 数据集存在问题，请检查上述失败项")

        report_lines.append("=" * 60)

        report_text = "\n".join(report_lines)
        print(report_text)

        # 保存报告
        if output_path:
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(report_text)
            print(f"\n报告已保存: {output_path}")
        else:
            report_path = os.path.join(self.data_dir, 'validation_report.txt')
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write(report_text)
            print(f"\n报告已保存: {report_path}")

        return report_text

    def validate(self, generate_plots=True):
        """
        执行完整验证流程

        Args:
            generate_plots: 是否生成可视化图表

        Returns:
            validation_results: 验证结果
        """
        print("\n" + "=" * 50)
        print("开始数据集验证")
        print("=" * 50)

        # Step 1: 加载数据
        self.load_dataset()

        # Step 2: 完整性检查
        self.check_integrity()

        # Step 3: 标签分布统计
        self.check_label_distribution()

        # Step 4: 生成可视化（可选）
        if generate_plots:
            self.plot_label_distribution()
            self.plot_sample_timeseries()
            self.plot_feature_distributions()
            self.plot_pca_visualization()

        # Step 5: 生成报告
        self.generate_report()

        print("\n" + "=" * 50)
        print("数据集验证完成！")
        print("=" * 50)

        return self.validation_results


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='数据集验证')
    parser.add_argument('--data_dir', type=str,
                       default='/home/wheeltec/R550PLUS_data_collect/log',
                       help='数据集目录')
    parser.add_argument('--no_plots', action='store_true',
                       help='跳过图表生成')

    args = parser.parse_args()

    # 创建验证器
    validator = DatasetValidator(args.data_dir)

    # 执行验证
    validator.validate(generate_plots=not args.no_plots)


if __name__ == '__main__':
    main()