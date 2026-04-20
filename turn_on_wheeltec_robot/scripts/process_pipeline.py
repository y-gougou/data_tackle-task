#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
process_pipeline.py - 数据处理完整流程

功能：
1. 自动创建数据集目录结构（按日期+故障类型分类）
2. 一键执行完整处理流程：预处理 → 滑动窗口 → 验证
3. 自动识别故障标签

目录结构：
  datasets/
  └── {date}_{fault_label}/
      ├── raw/                  # 原始CSV文件
      │   └── {采集的csv文件}.csv
      └── processed/            # 处理后的数据
          ├── processed_{timestamp}.csv
          ├── X_train.npy, X_test.npy, y_train.npy, y_test.npy
          ├── *_info.pkl, *_norm_params.pkl
          └── validation_report.txt

使用方法：
  # 单个文件处理
  python process_pipeline.py --csv "C:\数据集\0418\normal_20260419_154637.csv"

  # 批量处理目录
  python process_pipeline.py --csv_dir "C:\数据集集合" --fault_label 1

  # 指定输出目录
  python process_pipeline.py --csv "xxx.csv" --output "C:\datasets\2026-04-19_drive_fault"
"""

import os
import sys
import shutil
import argparse
from datetime import datetime


class DataPipeline:
    """数据处理流水线"""

    # 故障标签映射
    LABEL_MAP = {
        0: 'normal',
        1: 'drive_fault',
        2: 'wheel_slip',
        3: 'shaft_eccentric',
        4: 'voltage_low'
    }

    def __init__(self, csv_path, output_dir=None, fault_label=None, skip_first=5):
        """
        初始化数据流水线

        Args:
            csv_path: 原始CSV文件路径
            output_dir: 输出目录（默认自动生成）
            fault_label: 故障标签（默认从文件名自动识别）
            skip_first: 跳过前N秒数据
        """
        self.csv_path = csv_path
        self.csv_dir = os.path.dirname(csv_path)
        self.csv_filename = os.path.basename(csv_path)

        # 自动识别故障标签
        if fault_label is None:
            fault_label = self._detect_fault_label()
        self.fault_label = fault_label

        # 自动生成输出目录
        if output_dir is None:
            output_dir = self._generate_output_dir()
        self.output_dir = output_dir

        self.skip_first = skip_first

        # 子目录
        self.raw_dir = os.path.join(self.output_dir, 'raw')
        self.processed_dir = os.path.join(self.output_dir, 'processed')

        # 内部状态
        self.csv_output_path = None
        self.windows_output_dir = None

    def _detect_fault_label(self):
        """从文件名自动识别故障标签"""
        filename_lower = self.csv_filename.lower()

        # 检查文件名中的故障类型关键词
        if 'normal' in filename_lower:
            return 0
        elif 'drive_fault' in filename_lower or 'drive' in filename_lower:
            return 1
        elif 'wheel_slip' in filename_lower or 'hub' in filename_lower:
            return 2
        elif 'shaft' in filename_lower or 'eccentric' in filename_lower:
            return 3
        elif 'voltage_low' in filename_lower or 'low_voltage' in filename_lower or 'voltage' in filename_lower:
            return 4

        # 从文件名尝试提取数字
        for i in range(5):
            if f'_{i}_' in filename_lower or f'_{i}.' in filename_lower:
                return i

        print(f"警告: 无法从文件名 '{self.csv_filename}' 自动识别故障标签")
        print("  请使用 --fault_label 参数指定")
        return 0

    def _generate_output_dir(self):
        """自动生成输出目录"""
        # 从CSV文件名提取日期
        date_str = datetime.now().strftime("%Y-%m-%d")
        label_name = self.LABEL_MAP.get(self.fault_label, f"label_{self.fault_label}")

        # 尝试从CSV文件名提取日期
        if '2026' in self.csv_filename:
            # 从文件名提取日期部分
            parts = self.csv_filename.split('_')
            for i, part in enumerate(parts):
                if part.startswith('2026'):
                    date_str = part[:10]  # 取 YYYY-MM-DD 格式
                    break

        return os.path.join(self.csv_dir, f"{date_str}_{label_name}")

    def setup_directories(self):
        """创建目录结构"""
        print("\n创建目录结构:")
        print(f"  输出目录: {self.output_dir}")
        print(f"  原始数据: {self.raw_dir}")
        print(f"  处理后:   {self.processed_dir}")

        os.makedirs(self.raw_dir, exist_ok=True)
        os.makedirs(self.processed_dir, exist_ok=True)

        # 复制原始CSV到raw目录
        raw_csv_dest = os.path.join(self.raw_dir, self.csv_filename)
        if not os.path.exists(raw_csv_dest):
            shutil.copy2(self.csv_path, raw_csv_dest)
            print(f"  复制原始CSV: {self.csv_filename}")

        return self.raw_dir, self.processed_dir

    def step1_preprocess(self):
        """步骤1：预处理"""
        print("\n" + "=" * 60)
        print("步骤1: 数据预处理")
        print("=" * 60)

        # 导入预处理模块
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from preprocess_data import DataPreprocessor

        preprocessor = DataPreprocessor(self.raw_dir)
        preprocessor.load_data(os.path.basename(self.csv_path))

        preprocessor.process_pipeline(
            remove_outliers_flag=True,
            sync_flag=True,
            skip_first=self.skip_first
        )

        # 保存到processed目录
        base_name = f"processed_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        output_path = os.path.join(self.processed_dir, base_name)
        preprocessor.save_processed_data(output_path)

        self.csv_output_path = output_path + '.csv'
        print(f"\n预处理完成: {self.csv_output_path}")

        return self.csv_output_path

    def step2_sliding_windows(self):
        """步骤2：滑动窗口分割"""
        print("\n" + "=" * 60)
        print("步骤2: 滑动窗口分割")
        print("=" * 60)

        if self.csv_output_path is None:
            raise RuntimeError("步骤1未完成，请先运行预处理")

        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from create_sliding_windows import SlidingWindowCreator

        creator = SlidingWindowCreator(self.csv_output_path)
        creator.process_pipeline()

        # 保存到processed目录
        self.windows_output_dir = self.processed_dir
        print(f"\n滑动窗口完成")

        return creator.X_train, creator.X_test, creator.y_train, creator.y_test

    def step3_validate(self):
        """步骤3：数据集验证"""
        print("\n" + "=" * 60)
        print("步骤3: 数据集验证")
        print("=" * 60)

        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from validate_dataset import DatasetValidator

        validator = DatasetValidator(self.windows_output_dir)
        validator.run_validation()

        return validator.report

    def run(self):
        """运行完整流程"""
        print("\n" + "#" * 60)
        print(f"# 数据处理流水线")
        print(f"# 故障标签: {self.fault_label} ({self.LABEL_MAP.get(self.fault_label, 'unknown')})")
        print(f"# 跳过前 {self.skip_first} 秒数据")
        print("#" * 60)

        # Step 0: 创建目录
        self.setup_directories()

        # Step 1: 预处理
        self.step1_preprocess()

        # Step 2: 滑动窗口
        self.step2_sliding_windows()

        # Step 3: 验证
        report = self.step3_validate()

        # 最终总结
        print("\n" + "#" * 60)
        print("# 处理完成!")
        print("#" * 60)
        print(f"\n数据集目录: {self.output_dir}")
        print(f"处理后数据: {self.processed_dir}")
        print(f"\n包含文件:")
        for f in os.listdir(self.processed_dir):
            print(f"  - {f}")

        return report


def main():
    parser = argparse.ArgumentParser(description='数据处理完整流水线')
    parser.add_argument('--csv', type=str,
                       help='原始CSV文件路径（单个文件）')
    parser.add_argument('--csv_dir', type=str,
                       help='CSV目录路径（批量处理）')
    parser.add_argument('--fault_label', type=int, choices=[0,1,2,3,4],
                       help='故障标签（0=normal, 1=drive_fault, 2=wheel_slip, 3=shaft_eccentric, 4=voltage_low）')
    parser.add_argument('--output', type=str,
                       help='输出目录（默认自动生成）')
    parser.add_argument('--skip_first', type=float, default=5,
                       help='跳过前N秒数据（默认5秒，用于丢弃启动0值）')

    args = parser.parse_args()

    if args.csv:
        # 单文件处理
        pipeline = DataPipeline(
            csv_path=args.csv,
            output_dir=args.output,
            fault_label=args.fault_label,
            skip_first=args.skip_first
        )
        pipeline.run()

    elif args.csv_dir:
        # 批量处理目录
        import glob
        csv_files = glob.glob(os.path.join(args.csv_dir, '*.csv'))

        if not csv_files:
            print(f"未找到CSV文件: {args.csv_dir}")
            return

        print(f"找到 {len(csv_files)} 个CSV文件")

        for csv_path in sorted(csv_files):
            print(f"\n{'='*60}")
            print(f"处理文件: {os.path.basename(csv_path)}")
            print('='*60)

            pipeline = DataPipeline(
                csv_path=csv_path,
                output_dir=args.output,
                fault_label=args.fault_label,
                skip_first=args.skip_first
            )
            pipeline.run()

    else:
        parser.print_help()
        print("\n示例:")
        print("  python process_pipeline.py --csv \"C:\\数据集\\normal_20260419_154637.csv\"")
        print("  python process_pipeline.py --csv \"xxx.csv\" --fault_label 1 --skip_first 5")


if __name__ == '__main__':
    main()
