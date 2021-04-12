#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import *
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import thread
import time
import numpy as np
from follower.msg import flw as flw_msg

# Função para ajustar o valor do ângulo de guinada entre -180 e 180
def ajusta_yaw(_yaw):
	if abs(_yaw) > np.pi:
		if _yaw > 0:
			_yaw = _yaw - 2*np.pi
		if _yaw < 0:
			_yaw = 2*np.pi + _yaw
	
	return _yaw

# Classe contendo a posição inercial e ângulo de guinada atuais
class DronePosition:
	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.yaw = 0.0

# Classe para transição entre modos de voo
class fcuModes:
	def setTakeoff(self):
		rospy.wait_for_service('mavros/cmd/takeoff')
		try:
			takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
			takeoffService(altitude = 3)
		except rospy.ServiceException, e:
			print "Service takeoff call failed: %s"%e

	def setArm(self, _to_arm):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(_to_arm)
		except rospy.ServiceException, e:
			print "Service arming call failed: %s"%e

	def setMode(self, _mode):
		if _mode in ("STABILIZED",
					 "OFFBOARD",
					 "ALTCTL",
					 "POSCTL",
					 "AUTO.LAND",
					 "AUTO.RTL"):
			rospy.wait_for_service('mavros/set_mode')
			try:
				flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
				while not state.mode == _mode:
					flightModeService(custom_mode = _mode)
					rate.sleep()
			except rospy.ServiceException, e:
				print "service set_mode call failed: %s. Offboard Mode could not be set."%e

# Classe para geração de setpoints de posição inercial e ângulo de guinada
class SetpointPosition:
	def init(self, _x, _y, _z, _yaw):
		self.x = _x
		self.y = _y
		self.z = _z
		self.yaw = ajusta_yaw(_yaw)

		self.done_evt = threading.Event()

		# Publisher para mavros/setpoint_raw/local
		self.pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
		# Subscriber para mavros/local_position/pose
		self.sub = rospy.Subscriber("/mavros/local_position/pose", SP.PoseStamped, self.reached)

	def start(self):
		self.activated = True

		try:
			thread.start_new_thread(self.navigate_position, ())
		except:
			fault("Erro: Incapaz de iniciar a thread")

	def finish(self):
		self.activated = False            
    
	def navigate_position(self):
		msg = PositionTarget(
			header=SP.Header(
				frame_id="world",
				stamp=rospy.Time.now()),
			coordinate_frame=1
		)

		while not rospy.is_shutdown():
			if not self.activated:
				break

			# Preenchendo mensagem
			msg.position.x = self.x
			msg.position.y = self.y
			msg.position.z = self.z
			msg.yaw = self.yaw

			# Publica a mensagem
			self.pub.publish(msg)
			rate.sleep()

	def set(self, _x, _y, _z, _yaw, delay=0, wait=False):
		self.done_evt.clear()
		self.x = _x
		self.y = _y
		self.z = _z
		self.yaw = ajusta_yaw(_yaw)

		if wait:
			while not self.done_evt.is_set() and not rospy.is_shutdown():
				rate.sleep()

		if delay > 0:
			time.sleep(delay)

	def reached(self, topic):
		def is_near(msg, x, y, valor):
			return abs(x - y) < valor

		quat = topic.pose.orientation
		euler_angles = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])

		if is_near('X', topic.pose.position.x, self.x, 0.5) and \
			is_near('Y', topic.pose.position.y, self.y, 0.5) and \
			is_near('Z', topic.pose.position.z, self.z, 0.5) and \
			is_near('Yaw', euler_angles[2], self.yaw, 0.1):
			
			self.done_evt.set()

		DronePose.x = topic.pose.position.x
		DronePose.y = topic.pose.position.y
		DronePose.z = topic.pose.position.z
		DronePose.yaw = euler_angles[2]

# Classe para geração de setpoints
class SetpointGenerator:
	def init(self, _vx, _z, _yaw):
		self.vx = _vx
		self.z = _z
		self.yaw = ajusta_yaw(_yaw)
		self.y = 0.0
		self.erro_y = 0.0
		self.in_place = False
		self.waiting = False
		self.t = time.time()

		# Publisher para mavros/setpoint_raw/local
		self.pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
		# Subscriber para mavros/local_position/pose
		self.sub = rospy.Subscriber("/mavros/local_position/pose", SP.PoseStamped, self.reached)
		# Subscriber para follower/output_data
		self.sub_dtc = rospy.Subscriber("/follower/output_data", flw_msg, self.follower)

	def start(self):
		self.activated = True

		try:
			thread.start_new_thread(self.navigate, ())
		except:
			fault("Erro: Incapaz de iniciar a thread")

	def finish(self):
		self.activated = False            
    
	def navigate(self):
		msg = PositionTarget(
			header=SP.Header(
				frame_id="body",
				stamp=rospy.Time.now()),
			type_mask=0b0000101111000111,
			coordinate_frame=8
		)

		while not rospy.is_shutdown():
			if not self.activated:
				break
			
			# Controla a velocidade "lateral" do drone
			if self.erro_y == 0.0:
				self.y = DronePose.y
				vy = 0.0
			else:
				KPy = 0.05
				vy = KPy * self.erro_y

			# Aplica um controlador P para altitude
			KPz = 0.5
			erro_z = self.z - DronePose.z
			vz = KPz * erro_z

			# Limitador para a velocidade vertical máxima
			max_up_vel = 5.0
			vz = min(vz, max_up_vel)

			msg.velocity.x = self.vx
			msg.velocity.y = vy
			msg.velocity.z = vz
			msg.yaw = self.yaw

			# Publica a mensagem
			self.pub.publish(msg)
			rate.sleep()

	def reached(self, topic):
		def is_near(msg, x, y, valor):
			return abs(x - y) < valor

		quat = topic.pose.orientation
		euler_angles = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])

		if is_near('Y', topic.pose.position.y, self.y, 0.5) and \
			is_near('Z', topic.pose.position.z, self.z, 0.5) and \
			is_near('Yaw', euler_angles[2], self.yaw, 0.1):
			
			self.erro_y = 0.0
			self.in_place = True

		DronePose.x = topic.pose.position.x
		DronePose.y = topic.pose.position.y
		DronePose.z = topic.pose.position.z
		DronePose.yaw = euler_angles[2]

	def follower(self, topic):
		if self.in_place and (not self.waiting):
			if topic.linhas_detectadas > 0:
				# Seleciona o maior ângulo dentre os segmentos detectados
				#i = np.argmax(np.absolute(topic.angulos))
				yaw_sp =  DronePose.yaw + radians(topic.angulos[0])
				yaw_sp = ajusta_yaw(yaw_sp)
				if topic.angulos[0] < 5.0:
					self.vx = 2.5
					self.yaw = yaw_sp				
				else:
					self.vx = 0.0
					self.yaw = yaw_sp

				# Cálculo da altura relativa
				h_alvo = 35.0 # Define a altura relativa alvo
				h_tol = 1.0 # Define a tolerância admita para a altura relativa
				delta_h = round(h_alvo - max(topic.altura_relativa),1)
				if abs(delta_h) > h_tol and max(topic.altura_relativa) > 0.0:
					z_sp = DronePose.z + delta_h
					self.z = z_sp

				# Cálculo do deslocamento lateral
				y_tol = 0.2 # Define a tolerância admita para o deslocamento lateral
				delta_y = round(max(topic.deslocamento_lateral),1)
				if abs(delta_y) > y_tol:
					self.y = DronePose.y + delta_y
					self.erro_y = delta_y

				self.t = time.time()
			
		self.waiting = not self.waiting

state = State()
def stateCb1(msg):
	global state
	state = msg

extend_state = ExtendedState()
def stateCb2(msg):
	global extend_state
	extend_state = msg

modes = fcuModes()
DronePose = DronePosition()
setpoint_pos = SetpointPosition()
setpoint_gen = SetpointGenerator()

if __name__ == '__main__':
    try:
        # Nome do nó: rospy necessita do nome para comunicar com o ROS Master
        # anonymous = True: garante que o nó terá um nome único
        rospy.init_node('offboard_control', anonymous = True)
        mavros.set_namespace()
        rate = rospy.Rate(20)

        state = State()
        rospy.Subscriber('mavros/state', State, stateCb1)
        extend_state = ExtendedState()
        rospy.Subscriber('mavros/extended_state', ExtendedState, stateCb2)

        rospy.loginfo("Armando o drone")
        while not state.armed:
			modes.setArm(True)
			rate.sleep()
        
        rospy.loginfo("Iniciando módulo de controle de posição")
        setpoint_pos.init(0.0, 0.0, 0.0, 0.0)
        setpoint_pos.start()

        rospy.loginfo("Alterando o modo para Offboard")
        modes.setMode("OFFBOARD")

        rospy.loginfo("Takeoff")
        setpoint_pos.set(0.0, 0.0, 85.0, 0.0, wait=True)

        rospy.loginfo("Movendo para o ponto inicial")
        setpoint_pos.set(-28.0, 0.0, 85.0, 2.96706, wait=True)
        #setpoint_pos.set(-20.0, 196.0, 75.0, 0.523598775, wait=True)

        rospy.loginfo("Encerrando o módulo de controle de posição")
        setpoint_pos.finish()

        rospy.loginfo("Iniciando módulo de geração de setpoints")
        setpoint_gen.init(0.0, DronePose.z, DronePose.yaw)
        setpoint_gen.start()
		
        setpoint_gen.t = time.time()
        tempo = 0.0
        rospy.loginfo("Aguardando mensagens do detector")
        while tempo < 30.0:
			rate.sleep()
			tempo = time.time() - setpoint_gen.t

        rospy.loginfo("Retornando para o local de origem")
        modes.setMode("AUTO.RTL")

        while state.armed:
			if extend_state.landed_state == 1:
				modes.setArm(False)
			rate.sleep()
        rospy.loginfo("Drone desarmado")

        rospy.loginfo("Encerrando o módulo de geração de setpoints")
        setpoint_gen.finish()
        rospy.sleep(1)
        rospy.loginfo("Programa encerrado")

    # Evita que o código continue rodando acidentalmente após o sleep()
    except rospy.ROSInterruptException:
        pass
