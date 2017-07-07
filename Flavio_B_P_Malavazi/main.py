from lidar_thread import LiDARThread
from lidar_display import LiDARDisplay
from filtered_lidarDisplay import filteredLiDARDisplay

from move_robot import MoveRobotWithKeyBoard

from automatic_mode import automaticMode

from PEARLDEMO import PEARL

from turning_Function import turning_Function


def on_closing_window():
    global lidarth, lidardsp, pearl, moverbt, turningFunction, automaticMode



    # Stop Threads
    turningFunction.stop()
    automaticMode.stop()
    lidarth.stop()

    # Destroy UIs
    pearl.destroy()
    lidarsp2.destroy()
    moverbt.destroy()
    lidardsp.destroy()
    quit()


if __name__ == "__main__":
    #IP = "192.168.3.111"
    # IP = "127.0.0.1"      # Sim
    IP = "192.168.2.111"    # Robot
    #192.168.3.111

    lidarth = LiDARThread(IP)
    lidarth.start()

    pearl = PEARL()

    turningFunction = turning_Function()
    turningFunction.start()

    lidardsp = LiDARDisplay()
    lidarsp2 = filteredLiDARDisplay()
    lidardsp.protocol("WM_DELETE_WINDOW", on_closing_window)

    moverbt = MoveRobotWithKeyBoard(IP)
    moverbt.lift()
    moverbt.wm_attributes('-topmost', 1)
    moverbt.protocol("WM_DELETE_WINDOW", on_closing_window)

    automaticMode = automaticMode()
    automaticMode.start()

    lidardsp.mainloop()
