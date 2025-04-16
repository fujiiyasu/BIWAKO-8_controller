class parameter:
	def __init__(self):

		# ROBOT CONTROL PARAMETER
		self.distance_Kp = 10
		self.distance_Kd = 5
		self.degree_Kp = 1.3
		self.degree_Kd = 0.9

		# WAY POINT
		self.way_point_file = './way_point/iwakura_park.csv'

		# MODE
		self.data_log_mode = True
		self.debug_mode = False
		self.wt_log_mode = False

		# CONTROL MODE
		self.control_mode = 0
		# 0:FBLR MODE, 1:DIAGNALCONTROL MODE
		# 2:FIXED HEAD CONTROL MODE, 3:OCT-DIRECTIONAL MODE
		# self.thruster_control = 0
		# 0:SIMPLE CONTROL, 1:PHASE CONTROL

		# CONTROL STRATEGY
		self.strategy = 1
		# 0:SIMPLE STRATEGY, 1:FLEX STRATEGY

		# OTHER
		self.r = 2.0
		self.main_target_distance_tolerance = 3.0
		self.temp_target_distance_tolerance = 5.0
		self.heading_torelance = 5.0
		self.way_point_tolerance = 1.0
		self.duration = 10.0
		self.timer = 0.10
