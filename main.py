from wrotools import *



async def main():
    #await moveAttachmentArms(30, -270)
    await moveAttachmentArms(30, 450)
    await db.straight(190)
    await db.turn(90)
    await db.straight(300)
    
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
    #await db.turn(-90)
    


    

if __name__ == "__main__":
    db.distance_control.pid()
    run_task(main())
 
 