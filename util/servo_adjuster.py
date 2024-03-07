import time
import Adafruit_PCA9685

# 初期化
pwm = Adafruit_PCA9685.PCA9685(0x41)
pwm.set_pwm_freq(60)

# 有効なサーボ番号の範囲とパルス範囲
VALID_CHANNELS = range(8)
PULSE_MIN = 150
PULSE_MAX = 600

def is_valid_channel(ch):
    return ch in VALID_CHANNELS

def is_valid_pulse(pulse):
    return PULSE_MIN <= pulse <= PULSE_MAX

print("サーボ制御プログラムを開始します。")
print("サーボ番号(0-7)とパルス値(150-600)を入力してください。")
print("例: 0 400（サーボ0を400の角度に設定）")
print("終了したい場合は 'exit' と入力してください。")

while True:
    user_input = input("入力 > ")

    if user_input.strip().lower() == "exit":
        print("終了します。")
        break

    parts = user_input.strip().split()

    if len(parts) != 2:
        print("入力形式が正しくありません。サーボ番号と角度を半角スペースで区切って入力してください。")
        continue

    try:
        channel = int(parts[0])
        pulse = int(parts[1])

        if not is_valid_channel(channel):
            print("サーボ番号は0〜7の範囲で入力してください。")
            continue

        if not is_valid_pulse(pulse):
            print("角度（パルス値）は150〜600の範囲で入力してください。")
            continue

        pwm.set_pwm(channel, 0, pulse)
        print(f"サーボ{channel}を{pulse}に設定しました。")

    except ValueError:
        print("数値として解釈できませんでした。もう一度確認してください。")
