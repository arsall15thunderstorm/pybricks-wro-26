from pybricks.parameters import Color

from wrotools import db, yellowTowers, watch, resetDB, async_wrapper, colorScanning
from pybricks.tools import run_task, multitask
import gc



async def main():
    """the main function :)"""

    # initialization and running garbage collector
    gc.collect()
    db.settings(240, 700, 120, 250)
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
    colors = await multitask(async_wrapper(db.straight, 500), colorScanning())



if __name__ == "__main__":
    run_task(main())
 
