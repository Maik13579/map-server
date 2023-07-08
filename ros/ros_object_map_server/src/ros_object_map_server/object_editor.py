#!/usr/bin/env python3

import rospy
import tkinter as tk

from object_map_server import load_objects, Object, ObjectEditor


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 object_creator.py <object_path>")
        sys.exit(1)

    object_path = sys.argv[1]
    objects = load_objects(object_path)
    object_path = sys.argv[1]
    root = tk.Tk()
    app = ObjectEditor(master=root, objects=objects, highlight=lambda selected: print("selected "+selected))
    app.mainloop()