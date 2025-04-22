#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BIWAKO-8 サーボモードテスト
--------------------------
このスクリプトはBIWAKO-8ロボットの変形機構をテストします。
キーボード入力を使用して、キープモードと直進モードを切り替えることができます。
"""

import time
import os
import sys
import argparse
import signal

# 親ディレクトリをパスに追加して、モジュールをインポートできるようにする
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# 既存のクラスをインポート
from actuator_interface import ActuatorInterface

# 定数
DEFAULT_CONFIG_FILE = "../servo_pwm_config.csv"  # デフォルトの設定ファイルパス

def keyboard_control(actuator):
    """インタラクティブなキーボード制御ループ"""
    print("\n" + "="*50)
    print("BIWAKO-8 サーボモードテスト")
    print("="*50)
    print("コマンド:")
    print("  k - キープモードに切り替え")
    print("  s - 直進モードに切り替え")
    print("  i - 現在のモード情報を表示")
    print("  q - 終了")
    print("="*50 + "\n")
    
    current_mode = None
    
    while True:
        try:
            cmd = input("コマンド [k/s/i/q]: ").strip().lower()
            
            if cmd == 'k':
                print("[INFO] キープモードに変形します")
                actuator.set_keep_mode()
                current_mode = "keep"
            elif cmd == 's':
                print("[INFO] 直進モードに変形します")
                actuator.set_straight_mode()
                current_mode = "straight"
            elif cmd == 'i':
                if current_mode:
                    print(f"[INFO] 現在のモード: {current_mode}")
                else:
                    print("[INFO] モードがまだ設定されていません")
            elif cmd == 'q':
                break
            else:
                print(f"[ERROR] 不明なコマンド: {cmd}")
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[ERROR] {e}")

def signal_handler(sig, frame):
    """Ctrl+Cのハンドリング"""
    print("\n[INFO] 終了しています...")
    sys.exit(0)

def main():
    """メイン関数"""
    # コマンドライン引数のパース
    parser = argparse.ArgumentParser(description='BIWAKO-8 サーボモードテスト')
    parser.add_argument('--config', type=str, default=DEFAULT_CONFIG_FILE,
                      help=f'サーボ設定ファイルのパス (デフォルト: {DEFAULT_CONFIG_FILE})')
    args = parser.parse_args()
    
    # シグナルハンドラの設定（スムーズな終了のため）
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # アクチュエータインターフェースの初期化
        actuator = ActuatorInterface(pwm_config_file=args.config)
        
        # キーボード制御ループを開始
        keyboard_control(actuator)
        
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        # 終了時にスラスタを停止
        try:
            print("[INFO] スラスタを停止しています...")
            actuator.stop_thruster()
        except:
            pass
        print("[INFO] 終了しました")

if __name__ == "__main__":
    main()