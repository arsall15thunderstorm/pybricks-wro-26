from pybricks.parameters import Color

from wrotools import db, yellowTowers, watch, resetDB, async_wrapper, colorScanning, moveAttachmentArms, hub
from pybricks.tools import run_task, multitask
import gc



async def main():
    """the main function :)"""
    print(round((hub.battery.voltage()-6500)/19))
    # initialization and running garbage collector
    gc.collect()
    db.settings(240, 650, 120, 250)
    #db.settings(500, 1000, 500, 600)
    watch.reset()
    watch.resume()
    await resetDB()

    # yellow towers + time
    await yellowTowers()
    print(watch.time()/1000)

    # color scanning the artifacts
    await db.turn(-90)
    await db.straight(-77)
    await db.turn(-90)
    await db.straight(-80)
    await db.turn(-3)
    db.settings(210, 700, 150, 300)
    colors = await multitask(async_wrapper(db.straight, 670), colorScanning())[0]

    # picking up artifacts
    await moveAttachmentArms(40,-270)
    await db.turn(90)
    await db.straight(-250)
    await db.straight(320)
    await db.turn(90)
    await db.straight(230)
    await db.turn(90)
    await db.straight(220)
    await moveAttachmentArms(30, 250)
    await db.turn(180)
    await db.straight(600)

    #color thingy

    if colors[3] == Color.YELLOW:
        await db.turn(90)
        await db.straight(120)
        await db.turn(-90)
        await db.straight(60)
        await moveAttachmentArms(40,270)

if __name__ == "__main__":
    run_task(main())

 
