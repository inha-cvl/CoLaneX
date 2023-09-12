import rospy
from std_msgs.msg import Int8
import tkinter as tk
import time

def publish_mode(mode):
    mode_publisher.publish(mode)
    update_button_state(mode)

def enable_mode():
    publish_mode(1)

def disable_mode():
    publish_mode(0)

def publish_left():
    publish_signal(1)
    time.sleep(1)  # Wait for 2 seconds
    publish_signal(0)

def publish_straight():
    publish_signal(3)
    time.sleep(1)  # Wait for 2 seconds
    publish_signal(0)

def publish_right():
    publish_signal(2)
    time.sleep(1)  # Wait for 2 seconds
    publish_signal(0)

def publish_signal(signal):
    signal_publisher.publish(signal)
    
def update_button_state(current_mode):
    enable_button.config(state=tk.NORMAL if current_mode == 0 else tk.DISABLED)
    disable_button.config(state=tk.NORMAL if current_mode == 1 else tk.DISABLED)

def shutdown_hook():
    window.quit()

if __name__ == "__main__":
    rospy.init_node("ui_node")

    mode_publisher = rospy.Publisher("/mode", Int8, queue_size=10)
    signal_publisher = rospy.Publisher("/hlv_signal", Int8, queue_size=10)

    window = tk.Tk()
    window.title("MODE UI")

    button_font = ("Helvetica", 14)

    enable_button = tk.Button(window, text="Enable", command=enable_mode, bg='#00e096', width=10, height=3, font=button_font)
    enable_button.grid(row=0, column=0, padx=5, pady=5)

    disable_button = tk.Button(window, text="Disable", command=disable_mode, bg="#e0004f", width=10, height=3, font=button_font)
    disable_button.grid(row=0, column=1, padx=5, pady=5)

    left_button = tk.Button(window, text="Left", command=publish_left, bg="#3446eb", width=10, height=3, font=button_font)
    left_button.grid(row=1, column=0, padx=5, pady=5)

    straight_button = tk.Button(window, text="Straight", command=publish_straight, bg="#dbdbdb", width=10, height=3, font=button_font)
    straight_button.grid(row=1, column=1, padx=5, pady=5)

    right_button = tk.Button(window, text="Right", command=publish_right, bg="#ff5340", width=10, height=3, font=button_font)
    right_button.grid(row=1, column=2, padx=5, pady=5)

    rospy.on_shutdown(shutdown_hook) 

    window.mainloop()
