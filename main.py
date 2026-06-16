from wrotools import db, yellowTowers, watch, resetDB
from pybricks.tools import run_task
import gc



async def main():
    """the main function"""

    # initialization and running garbage collector
    gc.collect()
    db.settings(280, 800, 160, 300)
    #db.settings(500, 1000, 500, 600)
    watch.reset()
    watch.resume()
    await resetDB()

    # yellow towers + time
    await yellowTowers()
    print(watch.time()/1000)

    # color scanning the artifacts
    await db.turn(180)
    await db.straight(500)

if __name__ == "__main__":
    run_task(main())
 
