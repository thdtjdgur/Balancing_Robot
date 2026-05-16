import tensorflow as tf#TensorFlow 라이브러리를 불러옴
import numpy as np#숫자 배열 계산용 Numpy를 불러옴
import pandas as pd#CSV 파일을 표 형태로 읽고 다루기 위한 Pandas를 불러옴
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import joblib

# 1. CSV 읽기
df = pd.read_csv("lidar_log.csv")#라이다 로그 파일을 엑셀 표처럼 메모리 안으로 불러오는 줄

# 2. 입력 열 이름 만들기
feature_cols = [#“이 열들은 AI 입력으로 쓸 거야”라고 먼저 체크하는 줄
    "scan_x", "scan_y", "scan_psi", "pred_x", "pred_y"
]

for i in range(24):#d0부터 d23까지 입력 열 목록에 추가
    feature_cols.append(f"d{i}")

for i in range(24):#a0부터 a23까지 입력 열 목록에 추가
    feature_cols.append(f"a{i}")

label_col = "label_cost"#“AI가 맞춰야 할 정답은 label_cost다”라고 정하는 줄

# 3. 입력 X, 정답 y 분리
X = df[feature_cols].values.astype("float32")#입력 열들만 뽑아서 배열로 만듦
y = df[[label_col]].values.astype("float32")#정답 열들만 뽑아서 배열로 만듦

# 4. 학습/검증 데이터 분리
X_train, X_val, y_train, y_val = train_test_split(#전체 데이터를 학습용(train) 80%, 검증용(val) 20%로 나눔
#“가중치 수정용 데이터”와 “진짜 잘 배웠는지 검사하는 데이터”를 나눔
    X, y, test_size=0.2, random_state=42
)

# 5. 정규화(데이터값 크기를 학습하기 좋게 맞춰주는 변환기 객체를 만드는 줄)
x_scaler = StandardScaler()#입력 데이터 정규화 도구 생성
y_scaler = StandardScaler()#정답 데이터 정규화 도구 생성

X_train_scaled = x_scaler.fit_transform(X_train).astype("float32")
#학습용 입력 데이터를 기준으로 “적당한 기준척도”를 만든 뒤 값 크기를 맞추는 줄
X_val_scaled = x_scaler.transform(X_val).astype("float32")
#검증용 입력에도 방금 만든 같은 기준으로 정규화 적용

y_train_scaled = y_scaler.fit_transform(y_train).astype("float32")
y_val_scaled = y_scaler.transform(y_val).astype("float32")

# 6. 스케일러 저장
joblib.dump(x_scaler, "x_scaler.pkl")
joblib.dump(y_scaler, "y_scaler.pkl")

# 7. 모델 만들기
inputs = tf.keras.Input(shape=(len(feature_cols),), name="input")
h1 = tf.keras.layers.Dense(32, activation="relu")(inputs)
h2 = tf.keras.layers.Dense(16, activation="relu")(h1)
outputs = tf.keras.layers.Dense(1, name="output")(h2)

model = tf.keras.Model(inputs=inputs, outputs=outputs)

# 8. 학습 설정
model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
    loss="mse",
    metrics=["mae"]
)

# 9. 학습
history = model.fit(
    X_train_scaled,
    y_train_scaled,
    validation_data=(X_val_scaled, y_val_scaled),
    epochs=50,
    batch_size=64
)

# 10. 검증 성능 확인
val_loss, val_mae = model.evaluate(X_val_scaled, y_val_scaled, verbose=0)
print("val_loss =", val_loss)
print("val_mae  =", val_mae)

# 11. float32 TFLite 변환
run_model = tf.function(lambda t: model(t))
concrete_func = run_model.get_concrete_function(
    tf.TensorSpec([1, len(feature_cols)], tf.float32)
)

converter = tf.lite.TFLiteConverter.from_concrete_functions(
    [concrete_func], model
)
tflite_model = converter.convert()

with open("model.tflite", "wb") as f:
    f.write(tflite_model)

print("model.tflite 저장 완료")
print("입력 개수 =", len(feature_cols))
