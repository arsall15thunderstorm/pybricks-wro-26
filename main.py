from wrotools import *
import gc



async def main():
    #await moveAttachmentArms(30, -270)

    

    """await db.straight(-500)
    await moveAttachmentArms(30, 450)
    await db.straight(190)
    await db.turn(90)
    await db.straight(310)
    
    #await db.straight(115)    
    await moveAttachmentArms(30, -390)
    await db.straight(-325)
    await db.turn(90)
    await db.straight(1000)
    await db.turn(-90)
    await db.straight(270)
    await db.turn(90)
    await db.straight(330)
    await moveAttachmentArms(30, 240)
    await db.straight(-270)
    #await db.turn(-90)
    #await db.straight(-270)
    #await db.turn(90)
    #await db.straight(270)
    #await db.turn(-90)"""


    await db.straight(-500)
    gc.collect()
    db.drive(100, 0)

    while color_sensor1.reflection() > 20:
        await wait(10)
        print(f"{color_sensor1.color()}, {color_sensor1.reflection()}")

    db.brake()
    gc.collect()
    
    

    


if __name__ == "__main__":
    detectColor(color_sensor1)
    run_task(main( ))
 
 