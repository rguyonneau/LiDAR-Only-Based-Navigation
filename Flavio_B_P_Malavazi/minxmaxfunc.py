def extrem_lists(list):
    aux = float('inf')

    for i in range (len(list)):
        if list[i][0] < aux:
            aux = list[i][0]

    aux2 = float('-inf')

    for i in range(len(list)):
        if list[i][0] > aux2:
            aux2 = list[i][0]

    aux3 = float('inf')

    for i in range(len(list)):
        if list[i][1] < aux3:
            aux3 = list[i][1]

    aux4 = float('-inf')

    for i in range(len(list)):
        if list[i][1] > aux4:
            aux4 = list[i][1]

    return aux,aux2,aux3,aux4