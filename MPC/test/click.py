import time
import pyautogui

# Wait 4 hours (4 * 60 * 60 seconds)
time.sleep(35*60)

# Move to position (x, y) and click
pyautogui.moveTo(100, 100)
time.sleep(1)
pyautogui.moveTo(200, 200)
time.sleep(1)
pyautogui.click(x=1500, y=790)