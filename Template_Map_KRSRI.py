# ========= Map Robot KRSRI ========= [START]

    # start and goal position
    sx = 20.0  # [m]
    sy = 180.0  # [m]
    gx = 20.0  # [m]
    gy = 220.0  # [m]
    robot_size = 4.0  # [m]

    ox = []
    oy = []
    
    for i in range(0, 81):
        ox.append(i)
        oy.append(0)
    for i in range(0, 240):
        ox.append(0)
        oy.append(i)
        
    #Korban-1    
    for i in range(30, 140):
        ox.append(30)
        oy.append(i)
    for i in range(160, 190):
        ox.append(30)
        oy.append(i)
    for i in range(140, 160):
        ox.append(40)
        oy.append(i)
    for i in range(30, 40):
        ox.append(i)
        oy.append(140)
    for i in range(30, 40):
        ox.append(i)
        oy.append(160)
    
    for i in range(130, 191):
        ox.append(50)
        oy.append(i)
    for i in range(30, 50):
        ox.append(i)
        oy.append(130)
    for i in range(30, 50):
        ox.append(i)
        oy.append(40)
        
    for i in range(0, 51):
        ox.append(i)
        oy.append(190)
        
    #korban - 2
    for i in range(0, 61):
        ox.append(80)
        oy.append(i)
    for i in range(100, 215):
        ox.append(80)
        oy.append(i)
    for i in range(80, 91):
        ox.append(i)
        oy.append(60)
    for i in range(80, 91):
        ox.append(i)
        oy.append(100)
    for i in range(60, 100):
        ox.append(90)
        oy.append(i)
        
    #korban - 3
    for i in range(80, 95):
        ox.append(i)
        oy.append(215)
    for i in range(80, 95):
        ox.append(i)
        oy.append(240)
    for i in range(215, 240):
        ox.append(95)
        oy.append(i)
        
    #Topside
    for i in range(0, 30):
        ox.append(i)
        oy.append(240)
    for i in range(50, 81):
        ox.append(i)
        oy.append(240)
    for i in range(225, 240):
        ox.append(30)
        oy.append(i)
    for i in range(225, 240):
        ox.append(50)
        oy.append(i)
    for i in range(30, 50):
        ox.append(i)
        oy.append(225)
    
    # ========= Map Robot KRSRI ========= [END]