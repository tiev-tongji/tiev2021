with open("task_points.txt", "r") as file:
    result = [[] for _ in range(6)] 
    lines = file.readlines()
    for line in lines[1:]:
        items = line.split()
        idx = int(items[0])-1
        result[idx].append(items[2:])
    s = '\n"utm_x": {},\n"utm_y": {},\n"lon": {},\n"lat": {},\n"heading": {}\n'
    #s = '{},{},{},{},{}'
    for j in range(len(result)):
        print(j + 1)
        for i in result[j]:
            print("{",s.format(i[2],i[3],i[0],i[1],i[4]),"},")


