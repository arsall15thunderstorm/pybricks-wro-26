from wrotools import db, yellowTowers, watch, resetDB, async_wrapper, colorScanning, moveAttachmentArms
from pybricks.tools import run_task, multitask
import gc



async def main():
    """the main function :)"""

    # initialization and running garbage collector
    gc.collect()
    db.settings(280, 700, 120, 250)
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
    colors = await multitask(async_wrapper(db.straight, 650), colorScanning())

    # picking up artifacts
    await moveAttachmentArms(40,-270)
    await db.turn(90)
    await db.straight(-250)
    await db.straight(320)
    await db.turn(90)
    await db.straight(260)
    await db.turn(90)
    await db.straight(250)
    await db.turn(180)
    await db.straight(600)
if __name__ == "__main__":
    run_task(main())

 
