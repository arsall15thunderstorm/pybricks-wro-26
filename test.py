from wrotools import db, colorScanning, watch, resetDB, async_wrapper
from pybricks.tools import run_task, multitask
import gc
async def main():
    gc.collect()
    db.settings(240, 700, 120, 250)
    # db.settings(500, 1000, 500, 600)
    watch.reset()
    watch.resume()
    await resetDB()
    # color scanning the artifacts
    """await db.turn(-90)
    await db.straight(-50)
    await db.turn(-90)
    await db.straight(-50)+
    await db.straight(500)"""
    db.settings(120, 400, 120, 250)
    #print(await multitask(async_wrapper(db.straight, 5000), colorScanning(), race=True ))
    await colorScanning()



run_task(main())