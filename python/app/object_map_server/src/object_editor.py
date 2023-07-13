import tkinter as tk
from tkinter import ttk, messagebox, simpledialog

from .interfaces import Object, Geometry

class ObjectEditor(ttk.Frame):
    def __init__(self, master=None, objects=None, selected=None, q=None):
        master.title("Object Editor")
        master.attributes('-topmost', True)  # Make the window always appear on top
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
        super().__init__(master)
        self.master = master
        self.objects = objects
        self.selected = selected
        self.q = q
        self.grid()
        self.create_widgets()

    def on_closing(self):
        self.q.put(('close', None))
        self.master.destroy()

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
            self.q.put(('highlight', "object:"+selected_object))

    def handle_selection(self):
        # Get the selected object name
        selected_object = self.listbox.get(self.listbox.curselection())

        # Find the selected object in the list of objects
        for obj in self.objects:
            if obj.name == selected_object:
                new_window = tk.Toplevel(self.master)
                new_window.grab_set()
                EditObject(new_window, obj, self.q)


class EditObject(ttk.Frame):
    def __init__(self, master=None, obj: Object=None, q=None):
        master.title("Edit Object")
        master.attributes('-topmost', True)  # Make the window always appear on top
        super().__init__(master)
        self.master = master
        self.q = q
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

        # Add Geometry Button
        self.add_geometry_button = ttk.Button(self, text="Add Geometry", command=self.add_geometry)
        self.add_geometry_button.grid(column=1, row=3)

    def add_geometry(self):
        new_geometry_name = simpledialog.askstring("New Geometry", "Enter name of new geometry:", parent=self.master)
        if new_geometry_name in self.obj.geometries:
            messagebox.showerror("Error", "A geometry with that name already exists.")
        else:
            # Add the new geometry here.
            new_geometry = Geometry(name=new_geometry_name, type='CUBE')
            self.obj.geometries[new_geometry_name] = new_geometry
            self.listbox.insert(tk.END, new_geometry_name)

            # Send the new geometry to the object map server
            self.q.put(('add_geometry', new_geometry))

    def highlight_selection(self, event):
        # Check if an item is selected
        selection = self.listbox.curselection()
        if selection:
            selected_object = self.listbox.get(selection[0])
            self.q.put(('highlight', "geometry:"+selected_object))

    def handle_selection(self):
        # Get the selected object name
        selected_geometry = self.listbox.get(self.listbox.curselection())

        new_window = tk.Toplevel(self.master)
        new_window.grab_set()
        EditGeometry(new_window, self.obj.geometries[selected_geometry], self.q)


class EditGeometry(ttk.Frame):
    def __init__(self, master=None, geometry=None, q=None):
        master.title("Edit Geometry")
        master.attributes('-topmost', True)  # Make the window always appear on top
        super().__init__(master)
        self.master = master
        self.q = q
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

        r, g, b, a = self.geometry.color.r, self.geometry.color.g, self.geometry.color.b, self.geometry.color.a
        self.color_entry_r = ttk.Spinbox(self, from_=0.0, to=1.0, increment=0.01, command=self.update_geometry)
        self.color_entry_r.grid(column=1, row=2)
        self.color_entry_r.insert(0, r)

        self.color_entry_g = ttk.Spinbox(self, from_=0.0, to=1.0, increment=0.01, command=self.update_geometry)
        self.color_entry_g.grid(column=2, row=2)
        self.color_entry_g.insert(0, g)

        self.color_entry_b = ttk.Spinbox(self, from_=0.0, to=1.0, increment=0.01, command=self.update_geometry)
        self.color_entry_b.grid(column=3, row=2)
        self.color_entry_b.insert(0, b)

        self.color_entry_a = ttk.Spinbox(self, from_=0.0, to=1.0, increment=0.01, command=self.update_geometry)
        self.color_entry_a.grid(column=4, row=2)
        self.color_entry_a.insert(0, a)

        self.pose_label = ttk.Label(self, text="Pose (x, y, z):")
        self.pose_label.grid(column=0, row=3)

        x, y, z = self.geometry.pose.position.x, self.geometry.pose.position.y, self.geometry.pose.position.z
        self.pose_entry_x = ttk.Spinbox(self, from_=-100.0, to=100.0, increment=0.01, command=self.update_geometry)
        self.pose_entry_x.grid(column=1, row=3)
        self.pose_entry_x.insert(0, x)

        self.pose_entry_y = ttk.Spinbox(self, from_=-100.0, to=100.0, increment=0.01, command=self.update_geometry)
        self.pose_entry_y.grid(column=2, row=3)
        self.pose_entry_y.insert(0, y)

        self.pose_entry_z = ttk.Spinbox(self, from_=-100.0, to=100.0, increment=0.01, command=self.update_geometry)
        self.pose_entry_z.grid(column=3, row=3)
        self.pose_entry_z.insert(0, z)

        self.scale_label = ttk.Label(self, text="Scale (x, y, z):")
        self.scale_label.grid(column=0, row=4)

        x, y, z = self.geometry.scale.x, self.geometry.scale.y, self.geometry.scale.z
        self.scale_entry_x = ttk.Spinbox(self, from_=0.0, to=100.0, increment=0.01, command=self.update_geometry)
        self.scale_entry_x.grid(column=1, row=4)
        self.scale_entry_x.insert(0, x)

        self.scale_entry_y = ttk.Spinbox(self, from_=0.0, to=100.0, increment=0.01, command=self.update_geometry)
        self.scale_entry_y.grid(column=2, row=4)
        self.scale_entry_y.insert(0, y)

        self.scale_entry_z = ttk.Spinbox(self, from_=0.0, to=100.0, increment=0.01, command=self.update_geometry)
        self.scale_entry_z.grid(column=3, row=4)
        self.scale_entry_z.insert(0, z)

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

            r, g, b, a = float(self.color_entry_r.get()), float(self.color_entry_g.get()), float(self.color_entry_b.get()), float(self.color_entry_a.get())
            self.geometry.color.r, self.geometry.color.g, self.geometry.color.b, self.geometry.color.a = r, g, b, a

            x, y, z = float(self.pose_entry_x.get()), float(self.pose_entry_y.get()), float(self.pose_entry_z.get())
            self.geometry.pose.position.x, self.geometry.pose.position.y, self.geometry.pose.position.z = x, y, z

            x, y, z = float(self.scale_entry_x.get()), float(self.scale_entry_y.get()), float(self.scale_entry_z.get())
            self.geometry.scale.x, self.geometry.scale.y, self.geometry.scale.z = x, y, z

            self.geometry.mesh_resource = self.mesh_entry.get()

            self.q.put(('update', self.geometry))
        except Exception as e:
            self.show_error("Update Failed", str(e))
            return

    @staticmethod
    def show_error(title, message):
        messagebox.showerror(title, message)

