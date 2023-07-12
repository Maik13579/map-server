from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose

from interactive_object import InteractiveObject

import queue
import multiprocessing
import tkinter as tk

from object_map_server import ObjectEditor
from conversion import geometry2marker

class InteractiveObjectServer:
    def __init__(self, name, objects, root_frame):
        self.name = name
        self.objects = objects
        self.root_frame = root_frame
        self.server = InteractiveMarkerServer(self.name)
        self.menu_handler = self.init_menu()
        self.interactive_objects = self.load_objects()

        self.last_highlighted = None
        self.is_editing = False
        self.editor_open = False
        self.root = None


    def main_loop(self):
        """
        main loop to keep tkinter away from non-main threads
        """
        if not self.is_editing:
            return
        
        if not self.editor_open: #if editor not open start it in a new process
            self.editor_open = True
            self.q = multiprocessing.Queue()
            self.p = multiprocessing.Process(target=self._create_and_run_editor, args=(self.q,))
            self.p.start()

        #check queue
        try:
            message, data =  self.q.get(block=False)
        except queue.Empty:
            return
        if message == 'highlight':
            self.highlight(data)
        elif message == 'update':
            self.update_object(data)
        elif message == 'close':
            print("closing editor")
            
            #remove highlights
            for obj in self.interactive_objects.values():
                obj.remove_highlight()
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

            self.editor_open = False
            self.is_editing = False
            self.p.join() #wait for editor to close
            del self.p, self.q
        else:
            print("unknown message")
       
    def _create_and_run_editor(self, q):
        self.root = tk.Tk() 
        app = ObjectEditor(master=self.root, objects=self.objects, selected=self.last_highlighted, q=q)
        app.mainloop()
     

    def init_menu(self):
        '''
        creates the menu handler
        '''
        menu_handler = MenuHandler()
        menu_handler.insert("move", callback=self.move)
        menu_handler.insert("edit", callback=self.edit)
        return menu_handler
    
    def load_objects(self):
        '''
        load all objects
        '''
        interactive_objects = {}
        for obj in self.objects:
            frame, _ = self.root_frame.find_frame(obj.name)
            interactive_objects[obj.name] = InteractiveObject(self.server, obj, frame)
            self.server.insert(interactive_objects[obj.name].int_marker, self.click)

            # set different callback for POSE_UPDATE feedback
            self.server.setCallback(interactive_objects[obj.name].int_marker.name, interactive_objects[obj.name].update_callback, InteractiveMarkerFeedback.POSE_UPDATE )

            self.menu_handler.apply(self.server, obj.name)
            self.server.applyChanges()
        return interactive_objects


    ################################## MENU HANDLER CALLBACKS
    def move(self, feedback):
        '''
        callback from menu handler when you click on "move"
        '''
        self.interactive_objects[feedback.marker_name].move(feedback)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def edit(self, feedback):
        '''
        callback from menu handler when you click on "edit"
        opens object editor
        '''
        print('edit '+feedback.marker_name)
        if self.is_editing:
            return
        self.is_editing = True

        self.highlight('object:'+feedback.marker_name)
        self.last_highlighted = feedback.marker_name

        self.menu_handler.reApply(self.server)
        self.server.applyChanges() 

    def click(self, feedback):
        '''
        callback when you click on an object
        '''
        if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        #highlight object
        #if self.highlighted:
        #    self.objects[self.highlighted].remove_highlight()
        #self.highlighted = feedback.marker_name
        #self.objects[self.highlighted].add_highlight()

        self.interactive_objects[feedback.marker_name].click(feedback)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()  


    def update_object(self, geometry):
        print("update object: "+self.last_highlighted)
        print(geometry)
        for obj in self.interactive_objects.values():
            if obj.int_marker.name != self.last_highlighted:
                continue

            for i, marker in enumerate(obj.int_marker.controls[0].markers):
                if marker.text == geometry.name:
                    obj.int_marker.controls[0].markers[i] = geometry2marker(geometry, marker.header.frame_id, marker.header.frame_id, marker.id)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges() 

        
    def highlight(self, selected):
        type_, name = selected.split(':')
        print("selected "+name+" of type "+type_)

        if type_ == 'object':
            self.last_highlighted = name

            for obj in self.interactive_objects.values():
                if obj.int_marker.name == name:
                    obj.remove_highlight()
                else:
                    obj.add_highlight()

        elif type_ == 'geometry':
            for obj in self.interactive_objects.values():
                if obj.int_marker.name == self.last_highlighted:
                    obj.remove_highlight()

                    #highlight selected geometry
                    for marker in obj.int_marker.controls[0].markers:
                        if marker.text == name:
                            marker.color.a = 1.0
                        else:
                            marker.color.a = 0.5
                else:
                    obj.add_highlight(0.0)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges() 
