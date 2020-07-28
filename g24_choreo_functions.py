from motor import MOTOR

motor = MOTOR()

def readlist(filename):
    char_list = [ch for ch in open(filename).read() if ch not in ['\r', '\n']]
    return char_list

def readmove(moves, i, u):
    move = moves[i]
    if move == 'l':
        print('left 45')
        motor.A_forward(abs(u+50)*3)
        motor.B_forward(u+50)
    elif move == 'r':
        print('right 45')
        motor.A_forward(u+50)
        motor.B_forward((u+50)*3)
    elif move == 'b':
        print('backwards')
        motor.A_back((u+50)*3)
        motor.B_back((u+50)*3)
    elif move == 'f':
        print('forwards')
        motor.A_forward((u+50)*3)
        motor.B_forward((u+50)*3)
    elif move == 'm':
        print('left 90')
        motor.A_forward(u+50)
        motor.B_forward((u+50)*6)
    elif move == 'q':
        print('right 90')
        motor.B_forward(u+50)
        motor.A_forward((u+50)*6)
    elif move == 'L':
        print('Left 45 backwards')
        motor.A_back((u+50)*3)
        motor.B_back(u+50)
    elif move == 'R':
        print('right 45 backwards')
        motor.A_back(u+50)
        motor.B_back((u+50)*3)
    elif move == 'o':
        print('centre pivot: both wheels same speed, one forwards one backwards')
        motor.A_back((u+50)*3)
        motor.B_forward((u+50)*3)
    elif move == 'O':
        print('centre pivot: both wheels same speed, one forwards one backwards, reversed')
        motor.B_back((u+50)*3)
        motor.A_forward((u+50)*3)
    elif move == 'p':
        print('Pivot: one wheel stationary, one forwards 15 degrees')
        motor.A_stop()
        motor.B_forward((u+50)*3)
    elif move == 's':
        print('stop')
        motor.A_stop()
        motor.B_stop()
    elif move == 'x':
        # if the previous move was also PC, do not send the signal again
        print('PC drawing')
    else:
        print('Potential error')
