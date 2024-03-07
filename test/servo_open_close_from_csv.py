import time
import csv
import Adafruit_PCA9685

# 初期化
pwm = Adafruit_PCA9685.PCA9685(0x41)
pwm.set_pwm_freq(60)

# CSVファイル名
CSV_FILE = "servo_params.csv"

# サーボのパラメータを保持する辞書 {ch: (close_val, open_val)}
servo_data = {}

# CSVから読み込み
def load_servo_params(filename):
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if len(row) != 3:
                continue  # 無効な行はスキップ
            try:
                ch = int(row[0])
                close_val = int(row[1])
                open_val = int(row[2])
                servo_data[ch] = (close_val, open_val)
            except ValueError:
                continue  # 数値変換できなければスキップ

# サーボを順に動かす関数
def move_servos(mode='open'):
    print(f"サーボを{'開く' if mode == 'open' else '閉じる'}動作にします。")
    for ch, (close_val, open_val) in servo_data.items():
        target = open_val if mode == 'open' else close_val
        pwm.set_pwm(ch, 0, target)
        print(f"サーボ{ch} → {target}")
        time.sleep(0.1)  # 各サーボごとに少し待つ

# メイン処理
if __name__ == "__main__":
    load_servo_params(CSV_FILE)

    if not servo_data:
        print("サーボ設定が読み込めませんでした。CSVファイルを確認してください。")
        exit(1)

    try:
        while True:
            move_servos('open')
            time.sleep(1.5)
            move_servos('close')
            time.sleep(1.5)
    except KeyboardInterrupt:
        print("動作を終了します。")
