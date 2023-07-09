import tkinter as tk
from tkinter import ttk, messagebox

from .interfaces import Object, Geometry

class ObjectEditor(ttk.Frame):
    def __init__(self, master=None, objects=None, selected=None, highlight=lambda selected: print(selected), update_trigger=lambda geometry: print(geometry)):
        master.title("Object Editor")
        master.attributes('-topmost', True)  # Make the window always appear on top
        super().__init__(master)
        self.master = master
        self.objects = objects
        self.selected = selected
        self.highlight = highlight
        self.update_trigger = update_trigger
        self.grid()
        self.create_widgets()

    def create_widgets(self):
        # Create a Listbox and populate it with object names
        self.listbox = tk.Listbox(self)
        self.listbox.grid(column=0, row=0, rowspan=len(self.objects))
        for i, obj in enumerate(self.objects):
            self.listbox.insert(tk.END, obj.name)
            if obj.name == self.selected:
                self.listbox.selection_set(i)

        # Bind the <<ListboxSelect>> event to the highlight_selection method
        self.listbox.bind('<<ListboxSelect>>', self.highlight_selection)
        
        # Create a Button that calls the handle_selection method when clicked
        self.button = ttk.Button(self, text="Select", command=self.handle_selection)
        self.button.grid(column=1, row=0)

    def highlight_selection(self, event):
        # Check if an item is selected
        selection = self.listbox.curselection()
        if selection:
            selected_object = self.listbox.get(selection[0])
            self.highlight("object:"+selected_object)

    def handle_selection(self):
        # Get the selected object name
        selected_object = self.listbox.get(self.listbox.curselection())

        # Find the selected object in the list of objects
        for obj in self.objects:
            if obj.name == selected_object:
                new_window = tk.Toplevel(self.master)
                new_window.grab_set()
                EditObject(new_window, obj, self.highlight, self.update_trigger)


class EditObject(ttk.Frame):
    def __init__(self, master=None, obj: Object=None, highlight=lambda selected: print(selected), update_trigger=lambda geometry: print(geometry)):
        master.title("Edit Object")
        master.attributes('-topmost', True)  # Make the window always appear on top
        super().__init__(master)
        self.master = master
        self.highlight = highlight
        self.update_trigger = update_trigger
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

        # Create a Listbox and populate it with geometry names
        self.listbox = tk.Listbox(self)
        self.listbox.grid(column=0, row=2, rowspan=len(self.obj.geometries))
        for name in self.obj.geometries.keys():
            self.listbox.insert(tk.END, name)

        # Bind the <<ListboxSelect>> event to the highlight_selection method
        self.listbox.bind('<<ListboxSelect>>', self.highlight_selection)
        
        # Create a Button that calls the handle_selection method when clicked
        self.button = ttk.Button(self, text="Select", command=self.handle_selection)
        self.button.grid(column=1, row=2)

    def highlight_selection(self, event):
        # Check if an item is selected
        selection = self.listbox.curselection()
        if selection:
            selected_object = self.listbox.get(selection[0])
            self.highlight("geometry:"+selected_object)

    def handle_selection(self):
        # Get the selected object name
        selected_geometry = self.listbox.get(self.listbox.curselection())

        new_window = tk.Toplevel(self.master)
        new_window.grab_set()
        EditGeometry(new_window, self.obj.geometries[selected_geometry], self.update_trigger)


class EditGeometry(ttk.Frame):
    def __init__(self, master=None, geometry=None, update_trigger=lambda geometry: print(geometry)):
        master.title("Edit Geometry")
        master.attributes('-topmost', True)  # Make the window always appear on top
        super().__init__(master)
        self.master = master
        self.update_trigger = update_trigger
        self.geometry = geometry
        self.grid()
        self.create_widgets()

    def create_widgets(self):
        self.geometry_label = ttk.Label(self, text="Geometry Name:")
        self.geometry_label.grid(column=0, row=0)
        self.geometry_entry = ttk.Label(self, text=self.geometry.name)
        self.geometry_entry.grid(column=1, row=0)

        self.type_label = ttk.Label(self, text="Type:")
        self.type_label.grid(column=0, row=1)
        self.type_entry = ttk.Combobox(self, values=["CUBE", "SPHERE", "CYLINDER", "MESH_RESOURCE"], state='readonly')
        self.type_entry.grid(column=1, row=1)
        self.type_entry.set(self.geometry.type)

        self.color_label = ttk.Label(self, text="Color (r, g, b, a):")
        self.color_label.grid(column=0, row=2)
        self.color_entry = ttk.Entry(self)
        self.color_entry.grid(column=1, row=2)
        self.color_entry.insert(0, f"{self.geometry.color.r}, {self.geometry.color.g}, {self.geometry.color.b}, {self.geometry.color.a}")

        self.pose_label = ttk.Label(self, text="Pose (x, y, z):")
        self.pose_label.grid(column=0, row=3)
        self.pose_entry = ttk.Entry(self)
        self.pose_entry.grid(column=1, row=3)
        self.pose_entry.insert(0, f"{self.geometry.pose.position.x}, {self.geometry.pose.position.y}, {self.geometry.pose.position.z}")

        self.scale_label = ttk.Label(self, text="Scale (x, y, z):")
        self.scale_label.grid(column=0, row=4)
        self.scale_entry = ttk.Entry(self)
        self.scale_entry.grid(column=1, row=4)
        self.scale_entry.insert(0, f"{self.geometry.scale.x}, {self.geometry.scale.y}, {self.geometry.scale.z}")

        self.mesh_label = ttk.Label(self, text="Mesh Resource:")
        self.mesh_label.grid(column=0, row=5)
        self.mesh_entry = ttk.Entry(self)
        self.mesh_entry.grid(column=1, row=5)
        self.mesh_entry.insert(0, self.geometry.mesh_resource)

        self.update_button = ttk.Button(self, text="Update", command=self.update_geometry)
        self.update_button.grid(column=1, row=6)

    def update_geometry(self):
        try:
            self.geometry.type = self.type_entry.get()

            r, g, b, a = map(float, self.color_entry.get().split(', '))
            self.geometry.color.r, self.geometry.color.g, self.geometry.color.b, self.geometry.color.a = r, g, b, a

            x, y, z = map(float, self.pose_entry.get().split(', '))
            self.geometry.pose.position.x, self.geometry.pose.position.y, self.geometry.pose.position.z = x, y, z

            x, y, z = map(float, self.scale_entry.get().split(', '))
            self.geometry.scale.x, self.geometry.scale.y, self.geometry.scale.z = x, y, z

            self.geometry.mesh_resource = self.mesh_entry.get()

            self.update_trigger(self.geometry)
        except Exception as e:
            self.show_error("Update Failed", str(e))
            return

    @staticmethod
    def show_error(title, message):
        messagebox.showerror(title, message)

