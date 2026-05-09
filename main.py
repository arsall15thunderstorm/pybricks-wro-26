from wrotools import *

def main():
    db.straight(50)
    attachment_right.run_angle(convertSpeed(30, True), 360, wait=False)
    attachment_left.run_angle(convertSpeed(30, True), -360, wait=True)
    
    


if __name__ == "__main__":
    main()
