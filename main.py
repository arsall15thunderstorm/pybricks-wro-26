from wrotools import db, moveUntilColor, moveAttachmentArms, move
from pybricks.tools import run_task, multitask
import gc



async def main():
    """the main function"""

    """#await moveAttachmentArms(30, -270)
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

    gc.collect()

    # calibration
    await multitask(move(-500), moveAttachmentArms(30, 450))

    # picking up the towers
    await db.straight(256)
    await db.turn(-90)
    await db.straight(310)
    await moveAttachmentArms(30, -390)

    # placing first tower
    await db.straight(-30)
    await db.turn(90)
    await db.straight(500)
    await moveUntilColor(15, 30)
    await db.straight(445)
    await moveAttachmentArms(30,260)
    await db.straight(-200)

    # calibration
    await db.turn(90)
    await db.straight(-300)
    
    await moveUntilColor(15,30)
    await multitask(move(325), moveAttachmentArms(30, -260))
    await db.turn(-90)
    await db.straight(205)
    await moveAttachmentArms(30,260)
    await db.straight(-200)

if __name__ == "__main__":
    run_task(main())
 
