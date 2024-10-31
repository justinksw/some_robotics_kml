import pandas as pd
from sklearn.linear_model import LinearRegression

df = pd.read_csv('distance_log.txt')
y = df['distance'].to_numpy()
X = df.drop(columns=['distance']).to_numpy()

model = LinearRegression()

# X = [[26882], [25991], [25030], [23450], [22984],
#      [22345], [22315], [22144], [20418], [20400]]

# y = [330, 330, 340, 360, 367, 370, 370, 367, 373, 372]

model.fit(X, y)

# X_ = [[20400]]
# y_ = model.predict(X_)[0]

# print(y_)

# y = m * X + c
print('coef_', model.coef_)  # m
print('intercept_', model.intercept_)  # c

# area = 20400
# forward = round(-0.0076 * area + 534, 1)
# print(forward)
