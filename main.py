from wrotools import *

async def main():
    #await moveAttachmentArms(30, -270)
    await db.straight(90)
    await db.turn(90)
    await db.straight(330)
    await moveAttachmentArms(30, 450)
    await db.straight(190)
    await db.turn(90)
    await db.straight(305)
    
    #await db.straight(115)    
    await moveAttachmentArms(30, -390)
    await db.straight(-325)
    await db.turn(90)
    await db.straight(250)
    await db.turn(-90)
    await db.straight(750)
    await db.turn(-90)
    await db.straight(270)
    await db.turn(90)
    await db.straight(300)
    await moveAttachmentArms(30, -360)
    await db.straight(-270)
    #await db.turn(-90)
    #await db.straight(-270)
    #await db.turn(90)
    #await db.straight(270)
    #await db.turn(-90)
    


    

if __name__ == "__main__":
    run_task(main())
 
 