from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
#Cñ ̧óü1±üû~¿2ú1øwÛÿ
notes = [Note.C3, Note.D3, Note.E3, Note.F3, Note.G3, Note.A3, Note.B3, Note.C4]
#ú1ø~yyv~¿2{ü1y
for i in range(len(notes)):
    drone.drone_buzzer(notes[i], 500) # 0.5ûü1y
    time.sleep(0.01) # k~¿~w]w2zq»
drone.close()