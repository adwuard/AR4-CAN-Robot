
class AR4_HMI_Command_Proxy:
    
    
    
	def CMD_spline_start(self):
		cmd = "SL"
		#  splineTrue = true;
		#   delay(self5);
		#   Serial.print(self"SL");
		#   moveSequence = "";
		#   flag = "";
		#   rndTrue = false;
		#   splineEndReceived = false;
		
		pass


	def CMD_spline_stop(self):	
		cmd = "SS"
		# if (selffunction == "SS") {
		#   delay(self5);
		#   sendRobotPos(self);
		#   splineTrue = false;
		#   splineEndReceived = false;
		# }
		pass
		
	def CMD_close_device(self):
		cmd = "CL"
		# if (selffunction == "CL")
		# {
		#   delay(self5);
		#   Serial.end(self);
		# }
		pass

	def CMD_test_limit_switch(self):
		# if (selffunction == "TL") {
		pass  
			
	def CMD_set_encoder_to_1000(self):
		# if (selffunction == "SE")
		# {
		#   J1encPos.write(self1000);
		#   J2encPos.write(self1000);
		#   J3encPos.write(self1000);
		#   J4encPos.write(self1000);
		#   J5encPos.write(self1000);
		#   J6encPos.write(self1000);
		#   delay(self5);
		#   Serial.print(self"Done");
		# }
		pass





	def CMD_read_encoders(self):
		# if (selffunction == "RE")
		# {
		#   J1EncSteps = J1encPos.read(self);
		#   J2EncSteps = J2encPos.read(self);
		#   J3EncSteps = J3encPos.read(self);
		#   J4EncSteps = J4encPos.read(self);
		#   J5EncSteps = J5encPos.read(self);
		#   J6EncSteps = J6encPos.read(self);
		#   String Read = " J1 = " + String(selfJ1EncSteps) + "   J2 = " + String(selfJ2EncSteps) + "   J3 = " + String(selfJ3EncSteps) + "   J4 = " + String(selfJ4EncSteps) + "   J5 = " + String(selfJ5EncSteps) + "   J6 = " + String(selfJ6EncSteps);
		#   delay(self5);
		#   Serial.println(selfRead);
		# }
		pass


	def CMD_request_position(self):
		# if (selffunction == "RP")
		# {
		#   //close serial so next command can be read in
		#   delay(self5);
		#   if (selfAlarm == "0") {
		#     sendRobotPos(self);
		#   }
		#   else {
		#     Serial.println(selfAlarm);
		#     Alarm = "0";
		#   }
		# }
		pass



	def CMD_home_position(self):

		# if (selffunction == "HM")
		# {

		pass


	def CMD_update_params(self):
		# if (selffunction == "UP")
		pass


	def CMD_calibrate_external_axis(self):
		# if (selffunction == "CE")
		pass

	def CMD_zero_z7(self):
		# if (selffunction == "Z7")
		pass

	def CMD_zero_z8(self):
		# if (selffunction == "Z8")
		pass

	def CMD_zero_z9(self):
		# if (selffunction == "Z9")
		pass

	def CMD_wait_time(self):
		# if (selffunction == "WT")
		pass

		
	def CMD_if_input_then_jump(self):
		# if (selffunction == "JF")
		pass

		
	def CMD_io_on(self):
		# if (selffunction == "ON")
		pass
		
	def CMD_io_off(self):
		# if (selffunction == "OF")
		pass

	def CMD_wait_input_on(self):
		# if (selffunction == "WI")
		pass

	def CMD_wait_input_off(self):
		# if (selffunction == "WO")
		pass

	def CMD_send_position(self):
		# if (selffunction == "SP")
		pass
		
	def CMD_test_mesg(self):
		# -----COMMAND ECHO TEST MESSAGE---------------------------------------------------
		# if (selffunction == "TM")
		pass
		
	def CMD_calibrate(self):
		# -----COMMAND TO CALIBRATE---------------------------------------------------
		# if (selffunction == "LL")
		pass
		
	def CMD_motion_cartesian_jog(self):
		# ----- LIVE CARTESIAN JOG  ---------------------------------------------------
		# if (selffunction == "LC")
		pass
		
	def CMD_motion_live_joint_jog(self):
		# ----- LIVE JOINT JOG  ---------------------------------------------------
		# if (selffunction == "LJ")
		pass
		
	def CMD_motion_live_tool_jog(self):
		# ----- LIVE TOOL JOG  ---------------------------------------------------
		# if (selffunction == "LT")
		pass
		
		
	def CMD_motion_jog_t(self):
		# ----- Jog T ---------------------------------------------------
		# if (selffunction == "JT")
		pass
		
		
	def CMD_motion_move_v(self):
		# ----- MOVE V ------ VISION OFFSET ----------------------------------
		# if (selffunction == "MV")
		pass
		
		
		
	def CMD_motion_move_with_joint_rotation(self):
		# ----- MOVE IN JOINTS ROTATION  ---------------------------------------------------
		# if (selffunction == "RJ") {
		pass
			
		
		
	def CMD_motion_move_linear(self):
		# ----- MOVE L ---------------------------------------------------
		# if (selffunction == "ML" and flag == "")
		pass
		
		
	def CMD_motion_move_j(self):
		# ----- MOVE J ---------------------------------------------------
		# if (selffunction == "MJ")
		pass
		
		
	def CMD_motion_move_g(self):
		# ----- MOVE G ---------------------------------------------------
		# if (selffunction == "MG")
		pass
		
		
	def CMD_motion_move_circle(self):
		# ----- MOVE C (selfCirlce) ---------------------------------------------------
		# if (selffunction == "MC") {
		pass
			
		
	def CMD_motion_move_arc(self):
		"""
		Move A(selfArc)
		"""
		# ----- MOVE A (selfArc) ---------------------------------------------------
		# if (selffunction == "MA" and flag == "") {
		pass
			
			
	def message_dispatcher(self, function_type, payload):
		# -----MESSAGE DISPATCHER---------------------------------------------------

		if function_type == "SL":
			return self.CMD_spline_start(payload)
		elif function_type == "SS":
			return self.CMD_spline_stop(payload)
		elif function_type == "CL":
			return self.CMD_close_device(payload)
		elif function_type == "TL":
			return self.CMD_test_limit_switch(payload)
		elif function_type == "SE":
			return self.CMD_set_encoder_to_1000(payload)
		elif function_type == "RE":
			return self.CMD_read_encoders(payload)
		elif function_type == "RP":
			return self.CMD_request_position(payload)
		elif function_type == "HM":
			return self.CMD_home_position(payload)
		elif function_type == "UP":
			return self.CMD_update_params(payload)
		elif function_type == "CE":
			return self.CMD_calibrate_external_axis(payload)
		elif function_type == "Z7":
			return self.CMD_zero_z7(payload)
		elif function_type == "Z8":
			return self.CMD_zero_z8(payload)
		elif function_type == "Z9":
			return self.CMD_zero_z9(payload)
		elif function_type == "WT":
			return self.CMD_wait_time(payload)
		elif function_type == "JF":
			return self.CMD_if_input_then_jump(payload)
		elif function_type == "ON":
			return self.CMD_io_on(payload)
		elif function_type == "OF":
			return self.CMD_io_off(payload)
		elif function_type == "WI":
			return self.CMD_wait_input_on(payload)
		elif function_type == "WO":
			return self.CMD_wait_input_off(payload)
		elif function_type == "SP":
			return self.CMD_send_position(payload)
		elif function_type == "TM":
			return self.CMD_test_mesg(payload)
		elif function_type == "LL":
			return self.CMD_calibrate(payload)
		elif function_type == "LC":
			return self.CMD_motion_cartesian_jog(payload)
		elif function_type == "LJ":
			return self.CMD_motion_live_joint_jog(payload)
		elif function_type == "LT":
			return self.CMD_motion_live_tool_jog(payload)
		elif function_type == "JT":
			return self.CMD_motion_jog_t(payload)
		elif function_type == "MV":
			return self.CMD_motion_move_v(payload)
		elif function_type == "RJ":
			return self.CMD_motion_move_with_joint_rotation(payload)
		elif function_type == "ML":
			# - Move L - Move L is a linear move, this will execute a perfectly straight line to the position you teach. This program must send a series of waypoints that form the line to the Arduino, there is a couple second delay before any Move L while all waypoints are being transmitted.
			return self.CMD_motion_move_linear(payload)
		elif function_type == "MJ":
			# - Move J - Move J is a joint move where all joints work together to complete the move, this will not necessarily be a straight line but a sweeping motion with all joints working together.  This is the simplest and most common move to use.  
			return self.CMD_motion_move_j(payload)
		elif function_type == "MG":
			# - OFFS J - this is a joint move that is offset by the values of a stored position.  this move will apply whatever stored position number you have set in the stored position field that is just above the "Teach New Position" button. (stored positions are explained in further detail below in the entry for Move SP).
			return self.CMD_motion_move_g(payload)
		elif function_type == "MC":
			# - Move C - Move C is a circle move.  You must teach 3 points to form a circle.  First select "Move C center" and teach the center point for your circle - the speed and orientation values for this first point will be applied to the entire circle move.  Second you need to teach the start point on the circumference of the circle where you want the robot to begin and end the circle - select "Move C Start" and teach the second point.  Finally select "Move C Plane" and teach a point anywhere on the same plane you want your circle,  this point is only used to know which direction you want the circle to go and this third point defines the plane - in other words just teach another point on the circles circumference - it doesn’t really matter where it is, it’s not an executed point and only used for calculation.  Your command window will now have 3 lines of code in a row for each of the 3 points.  When a "Move C Start" line of code is executed the program will automatically run the next 2 lines of code to calculate the circle. The move will not work if these are out of order. There is a couple second delay before the Move C will execute while all waypoints to form the arc are being transmitted to the Arduino.
			return self.CMD_motion_move_circle(payload)
		elif function_type == "MA":
			# - Move A - Move A is an arc move.  You must teach 3 points to form an arc.  First select "Move A beg" and teach the start point for your arc - the speed and orientation values for this first point will be applied to the entire arc move.  Second you need to teach any mid-point on the arc - select "Move A Mid" and teach the second point.  Finally select "Move A End" and teach the point you want at the end or your arc.  Your command window will now have 3 lines of code in a row for each of the 3 points.  When a "Move A Beg" line of code is executed the program will automatically run the next 2 lines of code to calculate the arc. The move will not work if these are out of order. There is a couple second delay before the Move A will execute while all waypoints to form the arc are being transmitted to the Arduino.
			return self.CMD_motion_move_arc(payload)
		else:
			raise Exception("message_dispatcher: unknown function_type: " + function_type + " payload: " + payload)

"""
# - Move SP - SP stands for stored position.  In the registers tab there are 16 stored positions you can set.  You can set or save the X,Y,Z,Y,P,R for any position you want to execute later or multiple places in your program.  When you teach a Move SP the robot will move to the position you have entered for the stored position on the register tab.  *Stored positions can also be used for offsets - for example if you want the robot to come in above your part you may want to use an offset move with a stored position 25mm up in the Z direction - example: (0,0,-25,0,0,0).
# - OFFS SP - this moves the robot to a stored position and then offsets that position by the value in another stored position.  This is useful for stacking and placing parts in rows.  This move will use the value in the stored position field that is just above the "Teach New Position" button for the primary move to execute, then for the stored position that it will be offset by it automatically picks the next stored position - but you can use the manual entry field to edit which stored position the move will be offset by (see section below on editing).
# - Teach SP - this move command will insert 6 lines into your program which when executed will store the robots current position into a stored position register of your choice.  This makes it easier to populate stored positions as you need.
"""