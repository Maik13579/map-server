import tkinter as tk
from tkinter import ttk

from .interfaces import Object, Geometry

class ObjectEditor(ttk.Frame):
    def __init__(self, master=None, objects=None):
        master.title("Object Editor")
        super().__init__(master)
        self.master = master
        self.objects = objects
        self.grid()
        self.create_widgets()

    def create_widgets(self):
        # Create a Listbox and populate it with object names
        self.listbox = tk.Listbox(self)
        self.listbox.grid(column=0, row=0, rowspan=len(self.objects))
        for obj in self.objects:
            self.listbox.insert(tk.END, obj.name)
        
        # Create a Button that calls the handle_selection method when clicked
        self.button = ttk.Button(self, text="Select", command=self.handle_selection)
        self.button.grid(column=1, row=0)

    def handle_selection(self):
        # Get the selected object name
        selected_object = self.listbox.get(self.listbox.curselection())

        # Find the selected object in the list of objects
        for obj in self.objects:
            if obj.name == selected_object:
                new_window = tk.Toplevel(self.master)
                new_window.grab_set()
                AddObject(new_window, obj)


class AddObject(ttk.Frame):
    def __init__(self, master=None, obj: Object=None):
        master.title("Add Object")
        super().__init__(master)
        self.master = master
        self.obj = obj
        self.grid()
        self.create_widgets()

    def create_widgets(self):
        self.object_name_label = ttk.Label(self, text="Object Name:")
        self.object_name_label.grid(column=0, row=0)

        self.object_name_label2 = ttk.Label(self, text=self.obj.name)
        self.object_name_label2.grid(column=1, row=0)

        self.object_ns_label = ttk.Label(self, text="Object Namespace:")
        self.object_ns_label.grid(column=0, row=1)

        self.object_ns_label2 = ttk.Label(self, text=self.obj.ns)
        self.object_ns_label2.grid(column=1, row=1)

        # Create a Listbox and populate it with object names
        self.listbox = tk.Listbox(self)
        self.listbox.grid(column=0, row=2, rowspan=len(self.obj.geometries))
        for name in self.obj.geometries.keys():
            self.listbox.insert(tk.END, name)
        
        # Create a Button that calls the handle_selection method when clicked
        self.button = ttk.Button(self, text="Select", command=self.handle_selection)
        self.button.grid(column=1, row=2)


    def handle_selection(self):
        # Get the selected object name
        selected_geometry = self.listbox.get(self.listbox.curselection())

        new_window = tk.Toplevel(self.master)
        new_window.grab_set()
        AddGeometry(new_window, self.obj.geometries[selected_geometry])


class AddGeometry(ttk.Frame):
    def __init__(self, master=None, geometry: Geometry=None):
        master.title("Add Geometry")
        super().__init__(master)
        self.master = master
        self.geometry = geometry
        self.grid()
        self.create_widgets()

    def create_widgets(self):
        self.geometry_label = ttk.Label(self, text="Geometry Name:")
        self.geometry_label.grid(column=0, row=0)
        
        self.geometry_entry = ttk.Entry(self)
        self.geometry_entry.grid(column=1, row=0)
        self.geometry_entry.insert(0, self.geometry.name)
        
        self.type_label = ttk.Label(self, text="Type:")
        self.type_label.grid(column=0, row=1)
        
        self.type_entry = ttk.Combobox(self, values=["CUBE", "SPHERE", "CYLINDER"])
        self.type_entry.grid(column=1, row=1)
        self.type_entry.set(self.geometry.type)


