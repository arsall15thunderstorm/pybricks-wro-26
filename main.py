from wrotools import *

async def main():
    #await moveAttachmentArms(30, -270)
    #await attachment_left.run_target(-20,0)
    #await attachment_right.run_target(-20,0)
    await moveAttachmentArms(30, 360+160)
    await db.straight(200)
    await db.turn(90)
    await db.straight(275)
    await moveAttachmentArms(30, -270)
    await db.straight(-300)
    await db.turn(90)
    await db.straight(1000)
    await db.turn(-90)
    await db.straight(270)
    await db.turn(90)
    await db.straight(270)
    await moveAttachmentArms(30, 160)
    await db.straight(-270)
    #await db.turn(-90)
    #await db.straight(-270)
    #await db.turn(90)
    #await db.straight(270)
    #await db.turn(-90)
    


    

if __name__ == "__main__":
    run_task(main())
