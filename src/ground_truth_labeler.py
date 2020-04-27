import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import math

from tkinter import messagebox
from tkinter import filedialog
import tkinter as tk
import tkinter.font as tkFont
from scipy.spatial.transform import Rotation as R

server = None
poses = None
frame_id = None
def transform_matrix(pose):
	r = R.from_quat([pose[3], pose[4], pose[5], pose[6]])
	r = r.as_dcm()
	translation = np.array((pose[0], pose[1], pose[2])).reshape((3,1))
	T = np.concatenate((r, translation), axis=1)
	T = np.row_stack((T, [0,0,0,1]))
	return T

def processFeedback(feedback):
	global poses
	p = feedback.pose.position
	o = feedback.pose.orientation
	rospy.loginfo(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z) + ", " + str(o.x) + ", " + str(o.y) +", " + str(o.z) + ", " + str(o.w))

	index = int(feedback.marker_name);
	l = poses[index][7]
	w = poses[index][8]
	h = poses[index][9]
	pose = np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w, l, w, h])
	poses[index] = pose;

	n = np.size(poses, 0)
	arrows = MarkerArray()
	for i in range(0, n):
		pose_cp = poses[i].copy()
		if np.isnan(pose_cp[0]) == False:
			T = transform_matrix(pose_cp)
			p = np.array([0, 0, -pose_cp[9]/2 - 1, 1]).reshape((4,1))
			p = np.dot(T, p)
			pose_cp[0] = p[0]
			pose_cp[1] = p[1]
			pose_cp[2] = p[2]
			this_pose = makeArrow(pose_cp, i, frame_id)
			arrows.markers.append(this_pose)
	orientation_pub.publish(arrows)

def makeArrow(pose, i, frame_id):
	this_pose = Marker()
	this_pose.id = i
	this_pose.header.frame_id = frame_id
	this_pose.header.stamp = rospy.Time.now()
	this_pose.action = Marker.ADD
	this_pose.type = Marker.ARROW
	this_pose.scale.x =  pose[7]/2*1.5
	this_pose.scale.y =  pose[7]/2*0.1
	this_pose.scale.z =  pose[7]/2*0.1
	this_pose.color.r = 0;
	this_pose.color.g = 1;
	this_pose.color.b = 0;
	this_pose.color.a = 1;
	this_pose.pose.position.x = pose[0]
	this_pose.pose.position.y = pose[1]
	this_pose.pose.position.z = pose[2]
	this_pose.pose.orientation.x = pose[3]
	this_pose.pose.orientation.y = pose[4]
	this_pose.pose.orientation.z = pose[5]
	this_pose.pose.orientation.w = pose[6]
	return this_pose

def makeBoxControl(pose, name_id, frame_id):
	int_marker = InteractiveMarker()
	int_marker.header.frame_id = frame_id
	int_marker.name = name_id
	int_marker.description = ""
	int_marker.pose.position.x = pose[0];
	int_marker.pose.position.y = pose[1];
	int_marker.pose.position.z = pose[2];
	int_marker.pose.orientation.x = pose[3];
	int_marker.pose.orientation.y = pose[4];
	int_marker.pose.orientation.z = pose[5];
	int_marker.pose.orientation.w = pose[6];


	box_marker = Marker();
	text_marker = Marker();

	control = InteractiveMarkerControl()
	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "move_xy";
	control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE;

	color_marker = Marker();
	color_marker.type = Marker.CYLINDER;
	color_marker.color.r = 1.0;
	color_marker.color.g = 1.0;
	color_marker.color.b = 1.0;
	color_marker.color.a = 0.5;
	color_marker.scale.x = 0.5;
	color_marker.scale.y = 0.5;
	color_marker.scale.z = 0.005;
	control.markers.append(color_marker)
	int_marker.controls.append(control);


	z_control = InteractiveMarkerControl()
	z_control.orientation.w = 1;
	z_control.orientation.x = 0;
	z_control.orientation.y = 1;
	z_control.orientation.z = 0;
	z_control.name = "move_z";
	z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
	int_marker.controls.append(z_control);

	rotx_control = InteractiveMarkerControl()
	rotx_control.always_visible = True;
	rotx_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
	rotx_control.orientation.x = 1;
	rotx_control.orientation.y = 0;
	rotx_control.orientation.z = 0;
	rotx_control.orientation.w = 1;
	rotx_control.name = "rot_x";
	int_marker.controls.append(rotx_control);

	roty_control = InteractiveMarkerControl()
	roty_control.always_visible = True;
	roty_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
	roty_control.orientation.x = 0;
	roty_control.orientation.y = 1;
	roty_control.orientation.z = 0;
	roty_control.orientation.w = 1;
	roty_control.name = "rot_y";
	int_marker.controls.append(roty_control);


	rotz_control = InteractiveMarkerControl()
	rotz_control.always_visible = True;
	rotz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
	rotz_control.orientation.x = 0;
	rotz_control.orientation.y = 0;
	rotz_control.orientation.z = 1;
	rotz_control.orientation.w = 1;
	rotz_control.name = "rot_z";
	int_marker.controls.append(rotz_control); 

	box_marker.type = Marker.CUBE
	box_marker.scale.x = pose[7];
	box_marker.scale.y = pose[8];
	box_marker.scale.z = pose[9];
	box_marker.color.r = 1.0;
	box_marker.color.g = 1.0;
	box_marker.color.b = 1.0;
	box_marker.color.a = 0.6;
	box_marker.pose.position.z = -pose[9]/2 - 1;

	text_marker.type = Marker.TEXT_VIEW_FACING;
	text_marker.scale.z = 0.45;
	text_marker.text = name_id;
	text_marker.color.r = 1.0;
	text_marker.color.g = 1.0;
	text_marker.color.b = 1.0;
	text_marker.color.a = 1.0;
	text_marker.pose.position.x = 0;
	text_marker.pose.position.y = 0;
	text_marker.pose.position.z = 1;
	text_marker.pose.orientation.x = 0.0;
	text_marker.pose.orientation.y = 0.0;
	text_marker.pose.orientation.z = 0.0;
	text_marker.pose.orientation.w = 1.0;

	box_control = InteractiveMarkerControl()
	box_control.always_visible = True
	box_control.markers.append(box_marker)
	box_control.markers.append(text_marker)
	int_marker.controls.append(box_control)

	return int_marker

class Labeler():
	def __init__(self):
		self.ask_window = tk.Tk()
		self.ask_window.title('What is your coordinate frame?')
		self.ask_fontStyle = tkFont.Font(root = self.ask_window, family="Arial", size = 15)
		self.ask_l_frame = tk.Label(self.ask_window, text = 'Frame ID: ', font = self.ask_fontStyle)
		self.ask_e_frame = tk.Entry(self.ask_window, font = self.ask_fontStyle)
		self.ask_l_frame.grid(row = 0, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		self.ask_e_frame.grid(row = 0, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		self.ask_button_enter = tk.Button(self.ask_window, text = 'Enter', width = 15, height = 2, command = self.set_frame, font = self.ask_fontStyle)
		self.ask_button_enter.grid(row = 1, column = 0, padx = 10, pady = 10, columnspan = 2, sticky = tk.E + tk.W + tk.N + tk.S)
		self.ask_window.mainloop()
		global frame_id
		if frame_id == None or frame_id == '':
			frame_id = 'map'

		self.window = tk.Tk()
		self.window.title('Labeler')
		self.fontStyle = tkFont.Font(root = self.window, family="Arial", size = 15)

		self.choice = tk.Frame(self.window)
		self.choice.grid(row = 0, column = 0, sticky = tk.E + tk.W + tk.N + tk.S)
		self.choice.grid_propagate(True)

		self.button_new = tk.Button(self.choice, text = 'New Marker', width = 15, height = 2, command = self.new_marker, font = self.fontStyle)
		self.button_erase = tk.Button(self.choice, text = 'Erase Marker', width = 15, height = 2, command = self.erase_marker, font = self.fontStyle)
		self.button_update = tk.Button(self.choice, text = 'Update Marker', width = 15, height = 2, command = self.update_marker, font = self.fontStyle)
		self.button_exit = tk.Button(self.choice, text = 'Exit', width = 15, height = 2, command = self.window.quit, font = self.fontStyle)
		self.button_new.grid(row = 0, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		self.button_erase.grid(row = 0, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		self.button_update.grid(row = 0, column = 2, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		self.button_exit.grid(row = 0, column = 3, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		self.choice.grid_columnconfigure(0, weight = 1)
		self.choice.grid_columnconfigure(1, weight = 1)
		self.choice.grid_columnconfigure(2, weight = 1)
		self.choice.grid_columnconfigure(3, weight = 1)
		self.window.grid_columnconfigure(0, weight = 1)


		self.marker_info = tk.Frame(self.window)

		self.lb = tk.Listbox(self.marker_info, font = self.fontStyle)
		self.sby = tk.Scrollbar(self.marker_info, width = 20)
		self.sbx = tk.Scrollbar(self.marker_info, orient = tk.HORIZONTAL, width = 20)
		self.lb.configure(xscrollcommand = self.sbx.set, yscrollcommand = self.sby.set)
		self.sby['command'] = self.lb.yview
		self.sbx['command'] = self.lb.xview
		self.marker_info.grid(row = 1, column = 0, sticky = tk.E + tk.W + tk.N + tk.S)
		self.sby.grid(row = 0, column = 1, sticky = tk.E + tk.W + tk.N + tk.S)
		self.sbx.grid(row = 1, column = 0, sticky = tk.E + tk.W + tk.N + tk.S)
		self.lb.grid(row = 0, column = 0, sticky = tk.E + tk.W + tk.N + tk.S)
		self.marker_info.grid_columnconfigure(0, weight = 1)

		self.output = tk.Frame(self.window)
		self.button_refresh = tk.Button(self.output, text = 'Refresh', width = 15, height = 2, command = self.refrersh, font = self.fontStyle)
		self.button_save = tk.Button(self.output, text = 'Save', width = 15, height = 2, command = self.save_file, font = self.fontStyle)
		self.output.grid(row = 2, column = 0, sticky = tk.E + tk.W + tk.N + tk.S)
		self.button_refresh.grid(row = 0, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		self.button_save.grid(row = 0, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		self.window.mainloop()

	def set_frame(self):
		global frame_id
		frame_id = self.ask_e_frame.get()
		self.ask_window.destroy()

	def new_marker(self):
		def new_marker_():
			global poses
			n = int(e_num.get())
			l = float(e_length.get())
			w = float(e_width.get())
			h = float(e_height.get())
			for i in range(0, n):
				pose = [0, i*w, 0., 0., 0., 0., 1., l, w, h]
				if np.size(poses, 0) == 0:
					poses = np.array([pose])
					self.lb.insert('end', 'id, x, y, z, ox, oy, oz, ow, length, width, height')
				else:
					poses = np.row_stack((poses, pose))
				name_id = str(np.size(poses, 0) - 1)
				global frame_id
				int_marker = makeBoxControl(pose, name_id, frame_id);
				server.insert(int_marker, processFeedback);

				pose = poses[i].copy()
				T = transform_matrix(pose)
				p = np.array([0, 0, -pose[9]/2 - 1, 1]).reshape((4,1))
				p = np.dot(T, p)
				pose[0] = p[0]
				pose[1] = p[1]
				pose[2] = p[2]
				r = str(np.size(poses, 0) - 1) + ', ' + str(pose[0]) + ', ' + str(pose[1]) + ', ' + str(pose[2]) + ', '\
					+ str(pose[3]) + ', ' + str(pose[4]) + ', ' + str(pose[5]) + ', ' + str(pose[6]) + ', '\
					+ str(pose[7]) + ', ' + str(pose[8]) + ', ' + str(pose[9])
				self.lb.insert('end', r)
			server.applyChanges();
			window_new_marker.destroy()

		window_new_marker = tk.Toplevel(self.window)
		window_new_marker.title('New Marker')
		l_num = tk.Label(window_new_marker, text = 'Number(s): ', font = self.fontStyle)
		l_length = tk.Label(window_new_marker, text = 'Length(m): ', font = self.fontStyle)
		l_width = tk.Label(window_new_marker, text = 'Width(m): ', font = self.fontStyle)
		l_height = tk.Label(window_new_marker, text = 'Height(m): ', font = self.fontStyle)
		l_num.grid(row = 0, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_length.grid(row = 1, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_width.grid(row = 2, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_height.grid(row = 3, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)

		num = tk.StringVar()
		num.set('1')
		length = tk.StringVar()
		length.set('4')
		width = tk.StringVar()
		width.set('2')
		height = tk.StringVar()
		height.set('1')

		e_num = tk.Entry(window_new_marker, textvariable = num, font = self.fontStyle)
		e_length = tk.Entry(window_new_marker, textvariable = length, font = self.fontStyle)
		e_width = tk.Entry(window_new_marker, textvariable = width, font = self.fontStyle)
		e_height = tk.Entry(window_new_marker, textvariable = height, font = self.fontStyle)
		e_num.grid(row = 0, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		e_length.grid(row = 1, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)	
		e_width.grid(row = 2, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		e_height.grid(row = 3, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)

		button_new = tk.Button(window_new_marker, text = 'New', width = 15, height = 2, command = new_marker_, font = self.fontStyle)
		button_quit = tk.Button(window_new_marker, text = 'Quit', width = 15, height = 2, command = window_new_marker.destroy, font = self.fontStyle)
		button_new.grid(row = 4, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		button_quit.grid(row = 4, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)

		window_new_marker.grid_columnconfigure(0, weight = 1)
		window_new_marker.grid_columnconfigure(1, weight = 1)


	def erase_marker(self):
		def erase_marker_():
			str_id = e_id.get()
			int_id = int(str_id)
			if int_id < np.size(poses, 0)  and int_id >= 0:
				server.erase(str_id);
				for i in range(0, 10):
					poses[int_id][i]= np.nan;
				server.applyChanges();
				window_erase_marker.destroy()
			else:
				tk.messagebox.showerror(message='The marker with id = ' + str_id + ' has never been generated.')

		window_erase_marker = tk.Toplevel(self.window)
		window_erase_marker.title('Erase Marker')
		l_id = tk.Label(window_erase_marker, text = 'Erased ID: ', font = self.fontStyle)
		e_id = tk.Entry(window_erase_marker, font = self.fontStyle)
		l_id.grid(row = 0, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		e_id.grid(row = 0, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)

		button_erase = tk.Button(window_erase_marker, text = 'Enter', width = 15, height = 2, command = erase_marker_, font = self.fontStyle)
		button_quit = tk.Button(window_erase_marker, text = 'Quit', width = 15, height = 2, command = window_erase_marker.destroy, font = self.fontStyle)
		button_erase.grid(row = 1, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		button_quit.grid(row = 1, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)


	def update_marker(self):
		def show():
			int_id = int(e_id.get())
			if int_id < np.size(poses, 0)  and int_id >= 0:
				var_x.set(str(poses[int_id][0]))
				var_y.set(str(poses[int_id][1]))
				var_z.set(str(poses[int_id][2]))
				var_length.set(str(poses[int_id][7]))
				var_width.set(str(poses[int_id][8]))
				var_height.set(str(poses[int_id][9]))
			else:
				var_x.set('')
				var_y.set('')
				var_z.set('')
				var_length.set('')
				var_width.set('')
				var_height.set('')
			e_x.configure(state = 'normal')
			e_y.configure(state = 'normal')
			e_z.configure(state = 'normal')
			e_length.configure(state = 'normal')
			e_width.configure(state = 'normal')
			e_height.configure(state = 'normal')
			return True
		def update_marker_():
			str_id = e_id.get()
			int_id = int(str_id)
			if int_id < np.size(poses, 0)  and int_id >= 0:
				if np.isnan(poses[int_id][0]) == True:
					poses[int_id][3] = 0
					poses[int_id][4] = 0
					poses[int_id][5] = 0
					poses[int_id][6] = 1
				poses[int_id][0] = float(e_x.get())
				poses[int_id][1] = float(e_y.get())
				poses[int_id][2] = float(e_z.get())
				poses[int_id][7] = float(e_length.get())
				poses[int_id][8] = float(e_width.get())
				poses[int_id][9] = float(e_height.get())
				pose = poses[int_id]
				global frame_id
				int_marker = makeBoxControl(pose, str_id, frame_id);
				server.erase(str_id)
				server.insert(int_marker, processFeedback);
				server.applyChanges();
				window_update_marker.destroy()
			else:
				tk.messagebox.showerror(message='The marker with id = ' + str_id + ' has never been generated.')
			
		window_update_marker = tk.Toplevel(self.window)
		window_update_marker.title('Update Marker')
		l_instruction = tk.Label(window_update_marker, text = 'Update the position of the control object or the size of the cube.', font = self.fontStyle)
		l_id = tk.Label(window_update_marker, text = 'Updated ID: ', font = self.fontStyle)
		l_x = tk.Label(window_update_marker, text = 'tx(m): ', font = self.fontStyle)
		l_y = tk.Label(window_update_marker, text = 'ty(m): ', font = self.fontStyle)
		l_z = tk.Label(window_update_marker, text = 'tz(m): ', font = self.fontStyle)
		l_l = tk.Label(window_update_marker, text = 'Length(m): ', font = self.fontStyle)
		l_w = tk.Label(window_update_marker, text = 'Width(m): ', font = self.fontStyle)
		l_h = tk.Label(window_update_marker, text = 'Height(m): ', font = self.fontStyle)
		l_instruction.grid(row = 0, column = 0, columnspan = 2, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_id.grid(row = 1, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_x.grid(row = 2, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_y.grid(row = 3, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_z.grid(row = 4, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_l.grid(row = 5, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_w.grid(row = 6, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		l_h.grid(row = 7, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)

		var_x = tk.StringVar()
		var_y = tk.StringVar()
		var_z = tk.StringVar()
		var_length = tk.StringVar()
		var_width = tk.StringVar()
		var_height = tk.StringVar()
		e_id = tk.Entry(window_update_marker, validate = 'focusout', validatecommand = show, font = self.fontStyle)
		e_x = tk.Entry(window_update_marker, textvariable = var_x, font = self.fontStyle, state='readonly')
		e_y = tk.Entry(window_update_marker, textvariable = var_y, font = self.fontStyle, state='readonly')
		e_z = tk.Entry(window_update_marker, textvariable = var_z, font = self.fontStyle, state='readonly')
		e_length = tk.Entry(window_update_marker, textvariable = var_length, font = self.fontStyle, state='readonly')
		e_width = tk.Entry(window_update_marker, textvariable = var_width, font = self.fontStyle, state='readonly')
		e_height = tk.Entry(window_update_marker, textvariable = var_height, font = self.fontStyle, state='readonly')
		e_id.grid(row = 1, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		e_x.grid(row = 2, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)	
		e_y.grid(row = 3, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		e_z.grid(row = 4, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		e_length.grid(row = 5, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)	
		e_width.grid(row = 6, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		e_height.grid(row = 7, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)

		button_update = tk.Button(window_update_marker, text = 'Update', width = 15, height = 2, command = update_marker_, font = self.fontStyle)
		button_quit = tk.Button(window_update_marker, text = 'Quit', width = 15, height = 2, command = window_update_marker.destroy, font = self.fontStyle)
		button_update.grid(row = 8, column = 0, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)
		button_quit.grid(row = 8, column = 1, padx = 10, pady = 10, sticky = tk.E + tk.W + tk.N + tk.S)

	def refrersh(self):
		global poses
		n = np.size(poses, 0)
		self.lb.delete(0, tk.END)
		self.lb.insert('end', 'id, x, y, z, ox, oy, oz, ow, length, width, height')
		for i in range(0, n):
			pose = poses[i].copy()
			if np.isnan(pose[0]) == False:
				T = transform_matrix(pose)
				p = np.array([0, 0, -pose[9]/2 - 1, 1]).reshape((4,1))
				p = np.dot(T, p)
				pose[0] = p[0]
				pose[1] = p[1]
				pose[2] = p[2]

			r = str(i) + ', ' + str(pose[0]) + ', ' + str(pose[1]) + ', ' + str(pose[2]) + ', '\
				+ str(pose[3]) + ', ' + str(pose[4]) + ', ' + str(pose[5]) + ', ' + str(pose[6]) + ', '\
				+ str(pose[7]) + ', ' + str(pose[8]) + ', ' + str(pose[9])
			self.lb.insert('end', r)

	def save_file(self):
		f = filedialog.asksaveasfile(mode = 'w', defaultextension = '.csv')
		if f is None:
			return
		f.write('id, x, y, z, ox, oy, oz, ow, length, width, height\n')
		for i in range(0, np.size(poses, 0)):
			pose = poses[i].copy()
			if np.isnan(pose[0]) == False:
				T = transform_matrix(pose)
				p = np.array([0, 0, -pose[9]/2 - 1, 1]).reshape((4,1))
				p = np.dot(T, p)
				pose[0] = p[0]
				pose[1] = p[1]
				pose[2] = p[2]
				r = str(i) + ', ' + str(pose[0]) + ', ' + str(pose[1]) + ', ' + str(pose[2]) + ', '\
					+ str(pose[3]) + ', ' + str(pose[4]) + ', ' + str(pose[5]) + ', ' + str(pose[6]) + ', '\
					+ str(pose[7]) + ', ' + str(pose[8]) + ', ' + str(pose[9]) + '\n'
				f.write(r)

if __name__=="__main__":
	rospy.init_node("simple_marker")
	# rospy.Subscriber("/move_base_simple/goal", PoseStamped, navCallback)
	orientation_pub = rospy.Publisher('cube_poses', MarkerArray, queue_size = 100)
	server = InteractiveMarkerServer("simple_marker")
	poses = np.array([])



	Labeler()
	# rospy.spin()
	# window.mainloop()
