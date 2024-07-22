import tkinter as tk
from tkinter import ttk
import pyvista
from vtk.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor
import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from std_msgs.msg import Int32, Bool, Int32MultiArray, Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import time

class ROSListenerApp:
    def __init__(self):
        self.node = rclpy.create_node('Interface_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.ventana = tk.Tk()

        self.ventana.geometry("1920x1080")  # Tamaño de la ventana

        #Variables
        self.pwm1 = tk.DoubleVar()
        self.pwm1.set('0.00')
        self.pwm2 = tk.DoubleVar()
        self.pwm2.set('0.00')
        self.pwm3 = tk.DoubleVar()
        self.pwm3.set('0.00')

        self.is_on = False
        self.is_on2 = False
        self.is_on3 = False
        self.cr_on = False

        self.style = ttk.Style()
        self.style.configure("TScale1.Horizontal.TScale", background="grey", troughcolor="grey")
        self.style.configure("TScale2.Horizontal.TScale", background="#34a3bf", troughcolor="grey")

        self.on = tk.PhotoImage(file = "~/microros_ws/src/images/on.png")
        self.off = tk.PhotoImage(file = "~/microros_ws/src/images/off.png")
        self.command = tk.PhotoImage(file = "~/microros_ws/src/images/command.png")
        self.reading = tk.PhotoImage(file = "~/microros_ws/src/images/reading.png") 
        self.arcorw = tk.PhotoImage(file = "~/microros_ws/src/images/rw.png")
        self.on = self.on.subsample(10,10)
        self.off = self.off.subsample(10,10)

        # Sección portada

        self.frame_mask = tk.Frame(self.ventana, bg="grey15", width="449", height="1015", highlightbackground="DarkSlateGray2", highlightthickness=1)
        self.frame_mask.place(x=0, y=0)

        self.project_title =tk.Label(self.frame_mask, text="DESARROLLO DE UN MODULO ADCS \n PARA DISEÑO DE ALGORITMOS DE CONTROL DE ACTITUD\n EN CUBESATS DE ORBITA BAJA", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.project_title.place(x=5, y=10)

        self.image = tk.PhotoImage(file="~/microros_ws/src/images/image.png")
        self.image = self.image.subsample(2,2)
        self.image_label = tk.Label(self.frame_mask, image=self.image, bg="grey15")
        self.image_label.place(x=120, y=80)

        #-----------REACTION WHEEL 1------------
        self.rw1_frame = tk.Frame(self.frame_mask, bg="grey15", width="350", height="100", highlightbackground="grey15", highlightthickness=1)
        self.rw1_frame.place(x=50, y=350)
        self.rw1_title =tk.Label(self.rw1_frame, text="Reaction Wheel 1", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.rw1_title.place(x=5, y=13)
        
        self.buttonRW1 = tk.Button(self.rw1_frame, image = self.off, bd = 0, command = self.switch1, bg="grey15", highlightbackground="grey15", activebackground="grey15")
        self.buttonRW1.place(x=280, y=5)   

        self.slider1 = ttk.Scale(self.rw1_frame, from_=-100, to=100, length=330, style="TScale1.Horizontal.TScale", variable=self.pwm1, command=self.command_pwm1)
        self.slider1.place(x=6, y=40)
        self.slider1['state'] = 'disabled'
        self.pwm1_label =tk.Label(self.rw1_frame, textvariable=self.pwm1, bg="grey15", fg="white", font='Helvetica 11 bold')
        self.pwm1_label.place(x=155, y=70)
        self.CCW =tk.Label(self.rw1_frame, text="CCW", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.CCW.place(x=5, y=70)
        self.CW =tk.Label(self.rw1_frame, text="CW", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.CW.place(x=308, y=70)

        #-----------REACTION WHEEL 2------------
        self.rw2_frame = tk.Frame(self.frame_mask, bg="grey15", width="350", height="100", highlightbackground="grey15", highlightthickness=1)
        self.rw2_frame.place(x=50, y=450)
        self.rw1_title =tk.Label(self.rw2_frame, text="Reaction Wheel 2", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.rw1_title.place(x=5, y=13)
        
        self.buttonRW2 = tk.Button(self.rw2_frame, image = self.off, bd = 0, command = self.switch2, bg="grey15", highlightbackground="grey15", activebackground="grey15")
        self.buttonRW2.place(x=280, y=5)   

        self.slider2 = ttk.Scale(self.rw2_frame, from_=-1200, to=1200, length=330, style="TScale1.Horizontal.TScale", variable=self.pwm2, command=self.command_pwm2)
        self.slider2.place(x=6, y=40)
        self.slider2['state'] = 'disabled'
        self.pwm2_label =tk.Label(self.rw2_frame, textvariable=self.pwm2, bg="grey15", fg="white", font='Helvetica 11 bold')
        self.pwm2_label.place(x=155, y=70)
        self.CCW2 =tk.Label(self.rw2_frame, text="CCW", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.CCW2.place(x=5, y=70)
        self.CW2 =tk.Label(self.rw2_frame, text="CW", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.CW2.place(x=308, y=70)

        #-----------REACTION WHEEL 3------------
        self.rw3_frame = tk.Frame(self.frame_mask, bg="grey15", width="350", height="100", highlightbackground="grey15", highlightthickness=1)
        self.rw3_frame.place(x=50, y=550)
        self.rw1_title =tk.Label(self.rw3_frame, text="Reaction Wheel 3", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.rw1_title.place(x=5, y=13)
        
        self.buttonRW3 = tk.Button(self.rw3_frame, image = self.off, bd = 0, command = self.switch3, bg="grey15", highlightbackground="grey15", activebackground="grey15")
        self.buttonRW3.place(x=280, y=5)   

        self.slider3 = ttk.Scale(self.rw3_frame, from_=-100, to=100, length=330, style="TScale1.Horizontal.TScale", variable=self.pwm3, command=self.command_pwm3)
        self.slider3.place(x=6, y=40)
        self.slider3['state'] = 'disabled'
        self.pwm3_label =tk.Label(self.rw3_frame, textvariable=self.pwm3, bg="grey15", fg="white", font='Helvetica 11 bold')
        self.pwm3_label.place(x=155, y=70)
        self.CCW3 =tk.Label(self.rw3_frame, text="CCW", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.CCW3.place(x=5, y=70)
        self.CW3 =tk.Label(self.rw3_frame, text="CW", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.CW3.place(x=308, y=70)

        self.buttonCR = tk.Button(self.frame_mask, image = self.command, bd = 0, command = self.switchCR, bg="grey15", highlightbackground="grey15", activebackground="grey15")
        self.buttonCR.place(x=201, y=660)

        #----------PWM READ --------
        self.im1_title =tk.Label(self.frame_mask, text="REACTION\nWHEEL 1", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.im1_title.place(x=70, y=780)
        self.wheel1_frame = tk.Frame(self.frame_mask, bg="grey15", width="105", height="105", highlightbackground="grey15", highlightthickness=1)
        self.wheel1_frame.place(x=60, y=820)
        self.image_rw1 = tk.Label(self.wheel1_frame, image=self.arcorw, bg="grey15")
        self.image_rw1.place(relx=0, rely=0)
        self.pwm1_dir=tk.Label(self.wheel1_frame, text="DIR", bg="#cccccc", fg="grey15", font='Helvetica 8 bold')
        self.pwm1_dir.place(relx=0.5, rely=0.15, anchor='n')
        self.pwm1_data =tk.Label(self.wheel1_frame, bg="#cccccc", fg="grey15", font='Helvetica 26 bold', state="disabled")
        self.pwm1_data.place(relx=0.5, rely=0.5, anchor='center')
        self.pwm1_perc=tk.Label(self.wheel1_frame, text="%", bg="#cccccc", fg="grey15", font='Helvetica 10 bold')
        self.pwm1_perc.place(relx=0.5, rely=0.85, anchor='s')
        
        self.im2_title =tk.Label(self.frame_mask, text="REACTION\nWHEEL 2", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.im2_title.place(x=180, y=780)
        self.wheel2_frame = tk.Frame(self.frame_mask, bg="grey15", width="105", height="105", highlightbackground="grey15", highlightthickness=1)
        self.wheel2_frame.place(x=170, y=820)
        self.image_rw2 = tk.Label(self.wheel2_frame, image=self.arcorw, bg="grey15")
        self.image_rw2.place(relx=0, rely=0)
        self.pwm2_dir=tk.Label(self.wheel2_frame, text="DIR", bg="#cccccc", fg="grey15", font='Helvetica 8 bold')
        self.pwm2_dir.place(relx=0.5, rely=0.15, anchor='n')
        self.pwm2_data =tk.Label(self.wheel2_frame, bg="#cccccc", fg="grey15", font='Helvetica 26 bold', state="disabled")
        self.pwm2_data.place(relx=0.5, rely=0.5, anchor='center')
        self.pwm2_perc=tk.Label(self.wheel2_frame, text="%", bg="#cccccc", fg="grey15", font='Helvetica 10 bold')
        self.pwm2_perc.place(relx=0.5, rely=0.85, anchor='s')


        self.im3_title =tk.Label(self.frame_mask, text="REACTION\nWHEEL 3", bg="grey15", fg="white", font='Helvetica 11 bold')
        self.im3_title.place(x=290, y=780)
        self.wheel3_frame = tk.Frame(self.frame_mask, bg="grey15", width="105", height="105", highlightbackground="grey15", highlightthickness=1)
        self.wheel3_frame.place(x=280, y=820)
        self.image_rw3 = tk.Label(self.wheel3_frame, image=self.arcorw, bg="grey15")
        self.image_rw3.place(relx=0, rely=0)
        self.pwm3_dir=tk.Label(self.wheel3_frame, text="DIR", bg="#cccccc", fg="grey15", font='Helvetica 8 bold')
        self.pwm3_dir.place(relx=0.5, rely=0.15, anchor='n')
        self.pwm3_data =tk.Label(self.wheel3_frame, bg="#cccccc", fg="grey15", font='Helvetica 26 bold', state="disabled")
        self.pwm3_data.place(relx=0.5, rely=0.5, anchor='center')
        self.pwm3_perc=tk.Label(self.wheel3_frame, text="%", bg="#cccccc", fg="grey15", font='Helvetica 10 bold')
        self.pwm3_perc.place(relx=0.5, rely=0.85, anchor='s')

        #---------CALIBRACION--------
        self.Calib =tk.Label(self.frame_mask, text="Mover para calibrar...", bg="grey15", fg="white", font='Helvetica 12 bold')
        self.Calib.place(x=100, y=940)
        self.pb = ttk.Progressbar(self.frame_mask, orient='horizontal', mode='determinate', length=280)
        self.pb.place(x=85, y=970)

        # Sección plot: gráficas de sensores
        self.frame_plot = tk.Frame(width="697", height="800", highlightbackground="DarkSlateGray2", highlightthickness=1)
        self.frame_plot.pack(side=tk.RIGHT, anchor="n")

        self.fig = Figure()
        self.ax1 = self.fig.add_subplot(311)
        self.ax1.tick_params(axis='x', colors='white')
        self.ax1.tick_params(axis='y', colors='white')
        self.ax1.set_facecolor('black')

        self.fig.patch.set_facecolor('#262626')
        self.fig.suptitle('PLOTTER', color='white')

        self.ax2 = self.fig.add_subplot(312)
        self.ax2.tick_params(axis='x', colors='white')
        self.ax2.tick_params(axis='y', colors='white')
        self.ax2.set_facecolor('black')

        self.ax3 = self.fig.add_subplot(313)
        self.ax3.tick_params(axis='x', colors='white')
        self.ax3.tick_params(axis='y', colors='white')
        self.ax3.set_facecolor('black')

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_plot)
        self.canvas.draw()
        self.canvas.get_tk_widget().place(x=0, y=0)
        self.canvas.get_tk_widget().config(width="695", height="798")

        # Sección 3D VIZ
        self.frame_viz = tk.Frame(width="701", height="800", highlightbackground="DarkSlateGray2", highlightthickness=1)
        self.frame_viz.pack(side=tk.TOP, anchor="e")
        self.mesh = pyvista.read('~/microros_ws/src/images/CubeSat.stl')
        self.pl = pyvista.Plotter()
          
        self.pl.reset_camera()
        self.pl.show_axes()
        self.pl.set_background('black', top='white')
        self.pl.add_text('CUBESAT VISUALIZATION', font='arial', color='k', font_size=10, position='upper_edge')
        self.renwininteract = vtkTkRenderWindowInteractor(self.frame_viz, rw=self.pl.ren_win, width=700, height=800)
        #self.renwininteract.Initialize()
        self.renwininteract.pack(side='top', fill='both', expand=1)
        #self.renwininteract.Start()

        # Sección inferior: etiquetas de los datos de la IMU
        self.frame_labels = tk.Frame(bg="grey15", width="1399", height="215", highlightbackground="DarkSlateGray2", highlightthickness=1)
        self.frame_labels.place(x=449, y=800)

        #---------GIROSCOPIO----------
        self.gyro_frame =tk.Frame(self.frame_labels, bg="grey15", width="185", height="135", highlightbackground="white", highlightthickness=1)
        self.gyro_frame.place(x=100, y=40)
        self.gyro_title =tk.Label(self.gyro_frame, text="ACCEL. ANGULAR", bg="grey15", fg="white", font='Helvetica 12 bold')
        self.gyro_title.place(x=14, y=2)
        self.gyro_data =tk.Frame(self.frame_labels, bg="grey80")
        self.gyro_data.place(x=105, y=70)
        self.gyro_data =tk.Label(self.gyro_data, bg="grey15", fg="white", font='Arial 12')
        self.gyro_data.grid(row=0, column=0, rowspan=3)
        self.gyro_data.config(text=f"Gyro X  :  {0.0000:.4f}\n\nGyro Y  :  {0.0000:.4f}\n\nGyro Z  :  {0.0000:.4f}")

        #--------ORIENTACION----------
        self.euler_frame =tk.Frame(self.frame_labels, bg="grey15", width="185", height="135", highlightbackground="white", highlightthickness=1)
        self.euler_frame.place(x=400, y=40)
        self.euler_title =tk.Label(self.euler_frame, text="ORIENTACION ", bg="grey15", fg="white", font='Helvetica 12 bold')
        self.euler_title.place(x=35, y=2)
        self.euler_data =tk.Frame(self.frame_labels, bg="grey80")
        self.euler_data.place(x=405, y=70)
        self.euler_data =tk.Label(self.euler_data, bg="grey15", fg="white", font='Arial 12')
        self.euler_data.grid(row=0, column=0, rowspan=3)
        self.euler_data.config(text=f"Yaw   :  {0.0000:.4f}\n\nPitch  :  {0.0000:.4f}\n\nRoll   :  {0.0000:.4f}")

        #--------QUATERNION----------
        self.quat_frame =tk.Frame(self.frame_labels, bg="grey15", width="185", height="135", highlightbackground="white", highlightthickness=1)
        self.quat_frame.place(x=700, y=40)
        self.quat_title =tk.Label(self.quat_frame, text="ORIENTACION (QUAT)", bg="grey15", fg="white", font='Helvetica 12 bold')
        self.quat_title.place(x=3, y=2)
        self.quat_data =tk.Frame(self.frame_labels, bg="grey80")
        self.quat_data.place(x=705, y=80)
        self.quat_data =tk.Label(self.quat_data, bg="grey15", fg="white", font='Arial 12')
        self.quat_data.grid(row=0, column=0, rowspan=4)
        self.quat_data.config(text=f"Quat w  :  {0.0000:.4f}\nQuat x  :  {0.0000:.4f}\nQuat y  :  {0.0000:.4f}\nQuat z  :  {0.0000:.4f}")

        #--------TEMP----------
        self.temp_frame =tk.Frame(self.frame_labels, bg="grey15", width="185", height="135", highlightbackground="grey15", highlightthickness=1)
        self.temp_frame.place(x=950, y=40)
        self.temp_title =tk.Label(self.temp_frame, text="TEMPERATURE:", bg="grey15", fg="white", font='Helvetica 12 bold')
        self.temp_title.place(x=20, y=2)
        self.circle =tk.Canvas(self.temp_frame, width=115, height=115, bg="grey15", highlightbackground="grey15")
        self.circle.place(x=32, y=20)
        self.circle.create_oval(2,2,113,113, fill="grey15", outline="DarkSlateGray2")
        self.temp =tk.Label(self.temp_frame, text=f"{0} °C", bg="grey15", fg="white", font='Arial 23')
        self.temp.place(relx=0.49, rely=0.73, anchor='s')

        #--------BATTERY----------
        self.batt_frame =tk.Frame(self.frame_labels, bg="grey15", width="185", height="135", highlightbackground="grey15", highlightthickness=1)
        self.batt_frame.place(x=1160, y=40)
        self.batt_title =tk.Label(self.batt_frame, text="BATTERY:", bg="grey15", fg="white", font='Helvetica 12 bold')
        self.batt_title.place(x=38, y=2)
        self.batt =tk.Label(self.batt_frame, text=f"{100.0:.1f} %", bg="grey15", fg="white", font='Arial 23')
        self.batt.place(relx=0.44, rely=0.73, anchor='s')

        self.ventana.protocol("WM_DELETE_WINDOW", self.shutdown)
        
        formatter = plt.FuncFormatter(lambda x, _: '{:.3f}'.format(x))
        
        self.imu_data = {'time': [], 'x': [], 'y': [], 'z': []}
        self.line1, = self.ax1.plot([], [], 'r-')
        self.ax1.set_ylabel('Gyro(X)', color='white')
        self.ax1.set_title('Gyroscope Data', color='white')
        self.ax1.yaxis.set_major_formatter(formatter)
        
        self.line2, = self.ax2.plot([], [], 'g-')
        self.ax2.set_ylabel('Gyro (Y)', color='white')
        self.ax2.yaxis.set_major_formatter(formatter)
        
        self.line3, = self.ax3.plot([], [], 'b-')
        self.ax3.set_xlabel('Time', color='white')
        self.ax3.set_ylabel('Gyro (Z)', color='white')
        self.ax3.yaxis.set_major_formatter(formatter)


        self.subscription = self.node.create_subscription(
            Imu,
            'imu',
            self.callback,
            qos_profile  # Tamaño del búfer de mensajes
        )

        self.subscription_pwm = self.node.create_subscription(
            Int32MultiArray,
            'pwm_topic',
            self.callback_pwm,
            qos_profile  # Tamaño del búfer de mensajes
        )

        self.subscription_calib = self.node.create_subscription(
            Int32MultiArray,
            'calib_status',
            self.callback_calibration,
            qos_profile  # Tamaño del búfer de mensajes
        )

        self.subscription_euler = self.node.create_subscription(
            Float32MultiArray,
            'imu_euler',
            self.callback_euler,
            qos_profile  # Tamaño del búfer de mensajes
        )

        self.subscription_temp = self.node.create_subscription(
            Int32,
            'temp',
            self.callback_temp,
            qos_profile  # Tamaño del búfer de mensajes
        )

        self.subscription_battery = self.node.create_subscription(
            Int32,
            'battery',
            self.callback_battery,
            qos_profile  # Tamaño del búfer de mensajes
        )
        self.publisher_mode = self.node.create_publisher(
            Bool,
            'mode',
            qos_profile  # Tamaño del búfer de mensajes
        )

        self.publisher_dir1 = self.node.create_publisher(
            Bool,
            'dir1',
            10  # Tamaño del búfer de mensajes
        )
        self.publisher_dir2 = self.node.create_publisher(
            Bool,
            'dir2',
            10  # Tamaño del búfer de mensajes
        )
        self.publisher_dir3 = self.node.create_publisher(
            Bool,
            'dir3',
            10  # Tamaño del búfer de mensajes
        )

        self.publisher_en1 = self.node.create_publisher(
            Int32,
            'en1',
            10  # Tamaño del búfer de mensajes
        )
        self.publisher_en2 = self.node.create_publisher(
            Int32,
            'en2',
            10  # Tamaño del búfer de mensajes
        )
        self.publisher_en3 = self.node.create_publisher(
            Int32,
            'en3',
            10  # Tamaño del búfer de mensajes
        )

        self.publisher_pwm1 = self.node.create_publisher(
            Int32,
            'pwm1',
            10  # Tamaño del búfer de mensajes
        )
        self.publisher_pwm2 = self.node.create_publisher(
            Int32,
            'pwm2',
            10  # Tamaño del búfer de mensajes
        )
        self.publisher_pwm3 = self.node.create_publisher(
            Int32,
            'pwm3',
            10  # Tamaño del búfer de mensajes
        )

        self.ventana.after(0, self.process_ros_messages)

    def process_ros_messages(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.ventana.after(1, self.process_ros_messages)

    def callback(self, msg):
        # Obtener el tiempo de recepción actual
        current_time = time.time()

        # Actualizar la interfaz gráfica con los datos de la IMU
        self.imu_data['time'].append(current_time)
        self.imu_data['x'].append(msg.angular_velocity.x)
        self.imu_data['y'].append(msg.angular_velocity.y)
        self.imu_data['z'].append(msg.angular_velocity.z)

        # Limitar la longitud de los datos a los últimos 30 valores
        self.imu_data['time'] = self.imu_data['time'][-30:]
        self.imu_data['x'] = self.imu_data['x'][-30:]
        self.imu_data['y'] = self.imu_data['y'][-30:]
        self.imu_data['z'] = self.imu_data['z'][-30:]

        # Actualizar las gráficas
        self.line1.set_data(self.imu_data['time'], self.imu_data['x'])
        self.line2.set_data(self.imu_data['time'], self.imu_data['y'])
        self.line3.set_data(self.imu_data['time'], self.imu_data['z'])

        # Ajustar los límites de los ejes X e Y
        self.ax1.relim()
        self.ax1.autoscale_view()

        self.ax2.relim()
        self.ax2.autoscale_view()

        self.ax3.relim()
        self.ax3.autoscale_view()

        self.gyro_data.config(text=f"Gyro X  :  {msg.angular_velocity.x:.4f}\n\nGyro Y  :  {msg.angular_velocity.y:.4f}\n\nGyro Z  :  {msg.angular_velocity.z:.4f}")
        self.quat_data.config(text=f"Quat w  :  {msg.orientation.w:.4f}\nQuat x  :  {msg.orientation.x:.4f}\nQuat y  :  {msg.orientation.y:.4f}\nQuat z  :  {msg.orientation.z:.4f}")

        self.canvas.draw()
    def callback_pwm(self, msg):
        if msg is not None:
            self.pwm1_data.config(text=f"{msg.data[0]}")
            self.pwm2_data.config(text=f"{msg.data[1]}")
            self.pwm3_data.config(text=f"{msg.data[2]}")
            if msg.data[0] <= 0:
                self.pwm1_dir.config(text="CCW")
            else:
                self.pwm1_dir.config(text="CW")
            if msg.data[1] <= 0:
                self.pwm2_dir.config(text="CCW")
            else:
                self.pwm2_dir.config(text="CW")
            if msg.data[2] <= 0:
                self.pwm3_dir.config(text="CCW")
            else:
                self.pwm3_dir.config(text="CW")
                
        else:
            self.pwm1_data.config(text="00")
            self.pwm2_data.config(text="00")
            self.pwm3_data.config(text="00")

    
    def callback_calibration(self, msg):
        if 0 <= msg.data[0] <= 3:
            self.Calib.config(text="Mover para calibrar")
            self.pb.step(msg.data[0] * (100 / 3))
        
            if msg.data[0] == 3:
                self.Calib.config(text="Calibración completada")
                time.sleep(2)
                self.remove_progressbar_and_label()
        else:
            # Handle values outside the desired range (0-3) if needed
            pass

    def remove_progressbar_and_label(self):
        self.Calib.config(text="")
        self.pb.destroy()

    def callback_euler(self, msg):
        self.euler_data.config(text=f"Pitch   :  {msg.data[2]:.4f}\n\nRoll  :  {msg.data[1]:.4f}\n\n  Yaw   :  {msg.data[0]:.4f}")
        self.pl.clear()
        rot_mesh = self.mesh.copy()
        rot_mesh.rotate_x(-msg.data[1], point=self.mesh.center, inplace=True)
        rot_mesh.rotate_y(msg.data[2], point=self.mesh.center, inplace=True)
        rot_mesh.rotate_z(-msg.data[0], point=self.mesh.center, inplace=True)
        self.pl.add_mesh(rot_mesh, silhouette=True)
        self.pl.camera.zoom(1)
        self.renwininteract.Initialize()
        self.renwininteract.Start()

    def callback_temp(self, msg):
        self.temp.config(text=f"{msg.data} °C")

    def callback_battery(self, msg):
        battery_value = msg.data
        if 2287 <= battery_value <= 2628:
            battery = (battery_value - 2287) / (2628 - 2287) * 100
            self.batt.config(text=f"{battery:.2f} %")
        else:
            self.batt.config(text="100%")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.ventana.destroy()

    def run(self):
        self.pl.render()    
        self.ventana.mainloop()

    def switch1(self):
        if self.is_on:
            self.buttonRW1.config(image=self.off)
            self.is_on = False
            msg = Int32()
            msg.data = 0
            self.publisher_en1.publish(msg) 
            self.slider1.configure(style="TScale1.Horizontal.TScale") 
            self.slider1['state'] = 'disabled'      
        else:
            self.buttonRW1.config(image=self.on)
            self.is_on = True
            msg = Int32()
            msg.data = 1
            self.publisher_en1.publish(msg)
            self.slider1.configure(style="TScale2.Horizontal.TScale")
            self.slider1['state'] = 'normal'

    def switch2(self):
        if self.is_on2:
            self.buttonRW2.config(image=self.off)
            self.is_on2 = False
            msg = Int32()
            msg.data = 0
            self.publisher_en2.publish(msg) 
            self.slider2.configure(style="TScale1.Horizontal.TScale") 
            self.slider2['state'] = 'disabled'      
        else:
            self.buttonRW2.config(image=self.on)
            self.is_on2 = True
            msg = Int32()
            msg.data = 1
            self.publisher_en2.publish(msg)
            self.slider2.configure(style="TScale2.Horizontal.TScale")
            self.slider2['state'] = 'normal'

    def switch3(self):
        if self.is_on3:
            self.buttonRW3.config(image=self.off)
            self.is_on3 = False
            msg = Int32()
            msg.data = 0
            self.publisher_en3.publish(msg) 
            self.slider3.configure(style="TScale1.Horizontal.TScale") 
            self.slider3['state'] = 'disabled'      
        else:
            self.buttonRW3.config(image=self.on)
            self.is_on3 = True
            msg = Int32()
            msg.data = 1
            self.publisher_en3.publish(msg)
            self.slider3.configure(style="TScale2.Horizontal.TScale")
            self.slider3['state'] = 'normal'
    
    def switchCR(self):
        if self.cr_on:
            self.buttonCR.config(image=self.command)
            self.cr_on = False
            self.is_on = False
            self.is_on2 = False
            self.is_on3 = False
            self.buttonRW1.config(state="normal", image=self.off)
            self.buttonRW2.config(state="normal", image=self.off)
            self.buttonRW3.config(state="normal", image=self.off)
            self.pwm1_data.config(state="disabled")
            self.pwm2_data.config(state="disabled")
            self.pwm3_data.config(state="disabled")
            mode_msg = Bool()
            mode_msg.data = False
            self.publisher_mode.publish(mode_msg)
            self.pwm_msg = Int32()
            self.pwm_msg.data=0
            self.publisher_pwm1.publish(self.pwm_msg)
            self.publisher_pwm2.publish(self.pwm_msg)
            self.publisher_pwm3.publish(self.pwm_msg)
                
        else:
            self.buttonCR.config(image=self.reading)
            self.cr_on = True
            self.buttonRW1.config(state="disabled")
            self.buttonRW2.config(state="disabled")
            self.buttonRW3.config(state="disabled")
            self.slider1.set(0)
            self.slider2.set(0)
            self.slider3.set(0)
            self.slider1['state'] = 'disabled' 
            self.slider2['state'] = 'disabled' 
            self.slider3['state'] = 'disabled'
            self.pwm1_data.config(state="normal") 
            self.pwm2_data.config(state="normal") 
            self.pwm3_data.config(state="normal")
            mode_msg = Bool()
            mode_msg.data = True
            self.publisher_mode.publish(mode_msg)

    def command_pwm1(self, s):
        self.pwm1.set('%0.2f' % float(s))
        pwm1 = abs(self.pwm1.get())
        data1 = int(pwm1 * (1024 / 100))
        self.pwm_msg = Int32()
        self.pwm_msg.data=data1
        self.publisher_pwm1.publish(self.pwm_msg)
        if self.pwm1.get() <= 0:
            dir_msg = Bool()
            dir_msg.data = False  # Signo negativo, se publica False
            self.publisher_dir1.publish(dir_msg)
        else:
            dir_msg = Bool()
            dir_msg.data = True  # Signo positivo, se publica True
            self.publisher_dir1.publish(dir_msg)

    def command_pwm2(self, s):
        self.pwm2.set('%0.2f' % float(s))
        pwm2 = abs(self.pwm2.get())
        data2 = int(pwm2 * (2000 / 1200))
        self.pwm_msg = Int32()
        self.pwm_msg.data=data2
        self.publisher_pwm2.publish(self.pwm_msg)
        if self.pwm2.get() <= 0:
            dir_msg = Bool()
            dir_msg.data = False  # Signo negativo, se publica False
            self.publisher_dir2.publish(dir_msg)
        else:
            dir_msg = Bool()
            dir_msg.data = True  # Signo positivo, se publica True
            self.publisher_dir2.publish(dir_msg)
    
    def command_pwm3(self, s):
        self.pwm3.set('%0.2f' % float(s))
        pwm3 = abs(self.pwm3.get())
        data3 = int(pwm3 * (1024 / 100))
        self.pwm_msg = Int32()
        self.pwm_msg.data=data3
        self.publisher_pwm3.publish(self.pwm_msg)
        if self.pwm3.get() <= 0:
            dir_msg = Bool()
            dir_msg.data = False  # Signo negativo, se publica False
            self.publisher_dir3.publish(dir_msg)
        else:
            dir_msg = Bool()
            dir_msg.data = True  # Signo positivo, se publica True
            self.publisher_dir3.publish(dir_msg)

def main():
    rclpy.init()
    app = ROSListenerApp()
    app.run()

if __name__ == '__main__':
    main()
