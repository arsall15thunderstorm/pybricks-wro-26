from pybricks.parameters import Color

from wrotools import db, yellowTowers, watch, resetDB, async_wrapper, colorScanning, moveAttachmentArms, hub, cleanedList, moveLeftArm, moveRightArm
from pybricks.tools import run_task, multitask
import gc

async def colorScanning() -> list[Color]:
    """
    Scans colors (of artifacts) until a list of 4, unique, valid (as defined by list validColors) is formed

    :return: The list of scanned colors
    :rtype: list[Color]

    """
    cleanedList = []
    black_debounce_count = 0

    while True:
        currentReflection = await color_sensor2.reflection()
        currentScan = await color_sensor2.color()
        currentHSV = await color_sensor2.hsv()
        finalDebounce = 3
        if 3 <= currentReflection <= 10:
            black_debounce_count += 1
            if black_debounce_count >= finalDebounce:
                if Color.BLACK not in cleanedList:
                    cleanedList.append(Color.BLACK)
                    print(Color.BLACK, currentReflection, currentHSV)
        elif currentScan in validColors:
            black_debounce_count = 0
            if currentScan not in cleanedList:
                cleanedList.append(currentScan)
                print(currentScan, currentReflection, currentHSV)
        else:
            black_debounce_count = 0

        if len(cleanedList) == 4:
            print(cleanedList)
            break

        await wait(50)
    print(cleanedList[3])
    return cleanedList

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
    db.settings(250, 700, 150, 300)
    await db.straight(20)
    colors = await multitask(async_wrapper(db.straight, 670), colorScanning())
   
    # picking up artifacts
    await moveAttachmentArms(40,-270)
    await db.turn(90)
    await db.straight(-250)
    await db.straight(320)
    await db.turn(90)
    await db.straight(253)
    await db.turn(90)
    await db.straight(193)
    await moveAttachmentArms(30, 250)
    await db.turn(180)
    await db.straight(600)

    #color thingy
    if cleanedList[3] == "Color.YELLOW":    
        await db.turn(3)
        await db.turn(90)
        await db.straight(400)
        await db.turn(-90)
        await db.straight(160)
        await moveRightArm(40,-270)
        await db.straight(-100)
if __name__ == "__main__":
    run_task(main())

 
