

import matplotlib.pyplot as plt
import json
with open("median.json", "r") as file:
    data = json.load(file)
data_exponential = {"left": [50.0, 0.0, 0.0, 50.0, 0.0, 0.0, 50.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 50.0, 0.0, 0.0, 50.0, 0.0, 0.0, 50.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 50.0, 0.0, 0.0, 50.0, 0.0, 0.0, 110.0, 0.0, 0.0, 100.0, 0.0, 0.0, 230.0, 0.0, 0.0, 240.0, 0.0, 0.0], "right": [50.0, 0.0, 0.0, 50.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0], "center": [170.0, 0.0, 0.0, 170.0, 0.0, 0.0, 160.0, 0.0, 0.0, 160.0, 0.0, 0.0, 160.0, 0.0, 0.0, 160.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 150.0, 0.0, 0.0, 160.0, 0.0, 0.0, 160.0, 0.0, 0.0]}
data_original = {"left": [50.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 30.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 30.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 0.0, 0.0, 50.0, 0.0, 0.0, 40.0, 0.0, 0.0, 60.0, 0.0, 0.0, 60.0, 0.0, 0.0, 430.0, 0.0, 0.0], "right": [90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 70.0, 0.0, 0.0, 70.0, 0.0, 0.0, 80.0, 0.0, 0.0, 70.0, 0.0, 0.0, 50.0, 0.0, 0.0, 60.0, 0.0, 0.0, 60.0, 0.0, 0.0, 0.0, 0.0, 0.0, 60.0, 0.0, 0.0, 60.0, 0.0, 0.0, 60.0, 0.0, 0.0, 60.0, 0.0, 0.0, 60.0, 0.0, 0.0, 60.0, 0.0, 0.0, 70.0, 0.0, 0.0, 70.0, 0.0, 0.0, 70.0, 0.0, 0.0, 80.0, 0.0, 0.0], "center": [160.0, 0.0, 0.0, 160.0, 0.0, 0.0, 140.0, 0.0, 0.0, 130.0, 0.0, 0.0, 150.0, 0.0, 0.0, 140.0, 0.0, 0.0, 130.0, 0.0, 0.0, 140.0, 0.0, 0.0, 140.0, 0.0, 0.0, 140.0, 0.0, 0.0, 140.0, 0.0, 0.0, 140.0, 0.0, 0.0, 140.0, 0.0, 0.0, 140.0, 0.0, 0.0, 140.0, 0.0, 0.0, 130.0, 0.0, 0.0, 140.0, 0.0, 0.0, 130.0, 0.0, 0.0, 140.0, 0.0, 0.0, 140.0, 0.0, 0.0]}
# Ось x - номера точек
x = range(1, len(data["left"]) + 1)

# Данные для каждого датчика
y_left = data["left"]
y_right = data["right"]
y_center = data["center"]

# Данные экспоненциального сглаживания
y_exp_left = data_exponential["left"]
y_exp_right = data_exponential["right"]
y_exp_center = data_exponential["center"]


y_original_left = data_original["left"]
y_original_center = data_original["center"]
y_original_right = data_original["right"]
# Построение графика
plt.figure(figsize=(10, 6))
plt.scatter(x, y_left, label='Left - Median', color='blue', marker="v")
plt.scatter(x, y_right, label='Right - Median', color='red', marker="v")
plt.scatter(x, y_center, label='Center - Median', color='green', marker="v")

# Добавление точек для экспоненциального сглаживания
plt.scatter(x, y_exp_left, label='Left - Exponential', color='blue', marker='x')
plt.scatter(x, y_exp_right, label='Right - Exponential', color='red', marker='x')
plt.scatter(x, y_exp_center, label='Center - Exponential', color='green', marker='x')

plt.scatter(x, y_original_left, label='Left - Original', color='blue', marker='o')
plt.scatter(x, y_original_right, label='Right - Original', color='red', marker='o')
plt.scatter(x, y_original_center, label='Center - Original', color='green', marker='o')

plt.xlabel('Point Number')
plt.ylabel('Value')
plt.title('Values for Each Point')
plt.legend()
plt.grid(True)
plt.show()