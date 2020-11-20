import json


print("输入任务点序列:")
orders = list()
for i in range(6):
    k = int(input())
    orders.append(k)
print(orders)
json_data = dict()

with open("task_points.txt", "r") as file:
    result = [[] for _ in range(6)]
    lines = file.readlines()
    json_data["tasks"] = list()
    for line in lines[1:]:
        items = line.split()
        idx = int(items[0])-1
        result[idx].append(items[2:])

    for order in orders:
        if len(result[order - 1]) == 0:
            continue
        task = dict()
        task["task"] = dict()
        task["task"]["lon"] = float(result[order - 1][0][0])
        task["task"]["lat"] = float(result[order - 1][0][1])
        task["task"]["utm_x"] = float(result[order - 1][0][2])
        task["task"]["utm_y"] = float(result[order - 1][0][3])
        task["task"]["heading"] = float(result[order - 1][0][4])
        task["task_points"] = list()
        for task_points in result[order - 1]:
            point = dict()
            point["lon"] = float(task_points[0])
            point["lat"] = float(task_points[1])
            point["utm_x"] = float(task_points[2])
            point["utm_y"] = float(task_points[3])
            point["heading"] = float(task_points[4])
            task["task_points"].append(point)
        json_data["tasks"].append(task)
    json_str = json.dumps(json_data)

with open("任务点序列.json", "w") as json_file:
    json_file.write(json_str)
    print("生成任务点序列文件成功")


