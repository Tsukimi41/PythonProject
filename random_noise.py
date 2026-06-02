from codrone_edu.drone import *
import time
import random
#ùýüóçÿ
drone = Drone()
drone.pair()
#oy»¿~ú1ø2_r
notes = [Note.C3, Note.D3, Note.E3, Note.F3, Note.G3, Note.A3, Note.B3, Note.C4]
#ùóðþ ́o2ü1y·u
def random_noise(iterations):
    for i in range(iterations):
        note = random.choice(notes) # ùóð{¿2øþ
        duration = random.randint(100, 500) # 100<500ÿúû~wÿu2gÛ
        drone.drone_buzzer(note, duration) # ¿2ü1y
        time.sleep(0.1) # k~¿~w]wßt

#·u2ß ̄ÿÏÿ5Þ~oßy
random_noise(5)
#ùýüóï2Î
drone.close()
