#!/usr/bin/env python3
import pygame
import sys
import math
from abc import *

import rospy
from std_msgs.msg import String
from utils import ControlMessage, MAVROSCommander, MAVStateReciever

COLOR_WHITE = (255, 255, 255)
COLOR_RED = (255, 50, 50)
COLOR_GREEN = (50, 255, 50)
COLOR_BLUE = (50, 50, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_ORANGE = (255, 128, 0)
COLOR_YELLOW = (255, 255, 0)
COLOR_GRAY = (77, 77, 77)

screen = None
SCREEN_WIDTH = 520
SCREEN_HEIGHT = 200



class ClickableGUI(metaclass=ABCMeta):
    block_click = False

    @abstractmethod
    def draw(self, screen):
        pass

    @abstractmethod
    def check_collide(self, pos) -> bool:
        pass

    def check_click(self, pos):
        if not self.block_click and self.check_collide(pos):
            return True
        return False

    def click_initialize(self):
        self.block_click = False


class Stick(ClickableGUI):
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y

        self.power = 5.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.updated = False
        self.radius = radius
        self.set_color((255, 255, 255))

        self.is_drag = False

    def set_color(self, color):
        self.color = color
        
    def release(self):
        self.release_H()
        self.release_V()

    def release_H(self):
        if abs(self.pos_x) < self.power:
            self.pos_x = 0
        elif self.pos_x > 0:
            self.pos_x -= self.power
        elif self.pos_x < 0:
            self.pos_x += self.power

        self.updated = False

    def release_V(self):
        if abs(self.pos_y) < self.power:
            self.pos_y = 0
        elif self.pos_y > 0:
            self.pos_y -= self.power
        elif self.pos_y < 0:
            self.pos_y += self.power

        self.updated = False

    def set_pose(self, x=float, y=float):
        dist = math.sqrt(x * x + y * y)

        if (dist <= self.radius):
            self.pos_x = x
            self.pos_y = y
        else:
            self.pos_x = x / math.sqrt(x * x + y * y) * self.radius
            self.pos_y = y / math.sqrt(x * x + y * y) * self.radius

    def get_center(self):
        return (self.center_x, self.center_y)

    def get_pixel(self):
        return (self.center_x + self.pos_x, self.center_y + self.pos_y)

    def draw(self, screen):
        pygame.draw.circle(screen, COLOR_WHITE, self.get_center(), self.radius, width=1)
        pygame.draw.circle(screen, self.color, self.get_pixel(), 15)

    def check_collide(self, pos) -> bool:
        if self.is_drag:
            return True

        diff = (pos[0] - self.get_center()[0], pos[1] - self.get_center()[1])
        dist = math.sqrt(diff[0] * diff[0] + diff[1] * diff[1])

        if dist < self.radius + 25:
            return True
        return False

    def update(self, pos):
        self.block_click = False
        diff = (pos[0] - self.get_center()[0], pos[1] - self.get_center()[1])
        self.set_pose(diff[0], diff[1])
        self.updated = True
        self.is_drag = True

    def click_initialize(self):
        super().click_initialize()
        self.is_drag = False


class Button(ClickableGUI):
    def __init__(self, center_x, center_y, w, h):
        self.pos_x = center_x - w // 2
        self.pos_y = center_y - h // 2
        self.width = w
        self.height = h
        self.set_border(-1)
        self.set_color((255, 255, 255))
        self.set_text("")

    def set_color(self, color):
        self.color = color

    def set_border(self, radius):
        self.border = radius

    def set_text(self, text, color=(0, 0, 0)):
        self.font = pygame.font.SysFont('', 30)
        self.text_surface = self.font.render(text, True, color)
        self.off_x = self.width // 2 - self.text_surface.get_width() // 2
        self.off_y = self.height // 2 - self.text_surface.get_height() // 2

    def draw(self, screen):
        rect = [self.pos_x, self.pos_y, self.width, self.height]
        pygame.draw.rect(screen, self.color, rect, border_radius=self.border)
        screen.blit(self.text_surface, (self.pos_x + self.off_x, self.pos_y + self.off_y))

    def check_collide(self, pos) -> bool:
        if self.pos_x < pos[0] and pos[0] < self.pos_x + self.width:
            if self.pos_y < pos[1] and pos[1] < self.pos_y + self.height:
                return True
        return False


class Controller:
    def __init__(self):
        self.control_msg = ControlMessage()
        # self.pygame_img = None
        self.mouse_pressed = False

        self.GUIs = []

        self.stick_1 = Stick(100,100,60)
        self.stick_1.set_color(COLOR_WHITE)
        self.GUIs.append(self.stick_1)

        self.stick_2 = Stick(300+120,100,60)
        self.stick_2.set_color(COLOR_RED)
        self.GUIs.append(self.stick_2)

        self.my_font = pygame.font.SysFont('', 30)
        # self.text_surface_1 = self.my_font.render('Forward Lock', True, COLOR_BLUE)
        # self.text_surface_2 = self.my_font.render('Lock', True, COLOR_BLUE)

        self.shoud_exit = False

        self.commander = MAVROSCommander()
        self.vehicle_state = MAVStateReciever()

        self.button_1 = Button(SCREEN_WIDTH // 2, 45, 120, 24)
        self.button_1.set_color(COLOR_BLUE)
        self.button_1.set_text("MODE")
        self.GUIs.append(self.button_1)

        self.button_2 = Button(SCREEN_WIDTH // 2, 75, 120, 24)
        self.button_2.set_color(COLOR_BLUE)
        self.button_2.set_text("Delete Path")
        self.GUIs.append(self.button_2)

        self.button_3 = Button(SCREEN_WIDTH // 2, 105, 120, 24)
        self.button_3.set_color(COLOR_BLUE)
        self.button_3.set_text("Ctr mode")
        self.GUIs.append(self.button_3)

        self.button_4 = Button(SCREEN_WIDTH // 2, 135, 120, 24)
        self.button_4.set_color(COLOR_GRAY)
        self.button_4.set_text("Plan")
        self.GUIs.append(self.button_4)


        self.waypoint_mode_enabled = False
        self.last_control_pub_time = rospy.Time.now()

    def send_mode_change_cmd(self, mode):
        if self.commander.set_mode(mode):
            print('Set ' + mode + '!')
        else:
            print('Set mode fail!')

    def send_arm_cmd(self, value):
        if self.commander.set_arm(value):
            print('Arm!')
        else:
            print('Arming denied! Check is aleady armed.')

    def draw(self, screen):
        screen.fill(COLOR_BLACK)

        for GUI in self.GUIs:
            GUI.draw(screen)

    def get_clicked_gui(self, pos) -> ClickableGUI:
        for GUI in self.GUIs:
            if GUI.check_click(pos):
                return GUI
        return None

    def initialize_clicked_state(self):
        for GUI in self.GUIs:
            GUI.click_initialize()

    def control_callback(self, event):
        key_event = pygame.key.get_pressed()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == pygame.KEYUP:
                pass
                # if key_event[pygame.K_TAB]:
                #     self.send_mode_change_cmd("OFFBOARD")

        # # mouse pressed event
        if pygame.mouse.get_pressed()[0]:
            self.mouse_pressed = True
            mouse_pos = pygame.mouse.get_pos()
            clicked_gui = self.get_clicked_gui(mouse_pos)

            if type(clicked_gui) == Stick:
                clicked_gui.update(mouse_pos)

            elif clicked_gui == self.button_1:
                self.button_1.block_click = True
                if not self.vehicle_state.armed:
                    self.send_mode_change_cmd("AUTO.TAKEOFF")
                    self.send_arm_cmd(True)
                elif self.vehicle_state.mode == "OFFBOARD":
                    self.send_mode_change_cmd("POSCTL")
                else:
                    self.send_mode_change_cmd("OFFBOARD")

            elif clicked_gui == self.button_2:
                self.button_2.block_click = True
                self.control_msg.send_delete_path_cmd()

            elif clicked_gui == self.button_3:
                self.button_3.block_click = True
                if self.waypoint_mode_enabled:
                    self.waypoint_mode_enabled = False
                    self.button_3.set_text("Ctr Mode")
                    self.button_3.set_color(COLOR_BLUE)
                    self.button_4.set_color(COLOR_GRAY)
                else:
                    self.waypoint_mode_enabled = True
                    self.button_3.set_text("WP Mode")
                    self.button_3.set_color(COLOR_GREEN)
                    self.button_4.set_color(COLOR_GREEN)

            elif clicked_gui == self.button_4:
                self.button_4.block_click = True
                # if self.waypoint_mode_enabled:
                self.control_msg.send_plan_start()

        else:
            if self.mouse_pressed:
                self.mouse_pressed = False
                self.initialize_clicked_state()

        # ESC
        if key_event[pygame.K_ESCAPE]:
            self.shoud_exit = True
            rospy.signal_shutdown('close')
            return


        # set mode button
        if not self.vehicle_state.armed:
            self.button_1.set_text("Takeoff")
            self.button_1.set_color(COLOR_YELLOW)
        elif self.vehicle_state.mode == "OFFBOARD":
            self.button_1.set_text("POSCTL")
            self.button_1.set_color(COLOR_BLUE)
        else:
            self.button_1.set_text("Offboard")
            self.button_1.set_color(COLOR_ORANGE)


        self.draw(screen)

        text_control_state = None

        # Publish
        if self.stick_2.updated or self.waypoint_mode_enabled:
            roll = float(-self.stick_2.pos_x) / float(self.stick_2.radius)
            pitch = float(-self.stick_2.pos_y) / float(self.stick_2.radius)
            if roll and pitch:
                text_control_state = "Y : {:0.0f}%, X : {:0.0f}%".format(roll*100, pitch*100)
            elif roll:
                text_control_state = "Y : {:0.0f}%".format(roll*100)
            elif pitch:
                text_control_state = "X : {:0.0f}%".format(pitch*100)
        else: # skip motion
            roll = 0
            pitch = 0

        if self.stick_1.updated or self.waypoint_mode_enabled:
            yaw = float(-self.stick_1.pos_x) / float(self.stick_1.radius)
            throttle = float(-self.stick_1.pos_y) / float(self.stick_1.radius)
            if yaw and throttle:
                text_control_state = "Z : {:0.0f}%, yaw : {:0.0f}%".format(throttle*100, yaw*100)
            elif yaw:
                text_control_state = "yaw : {:0.0f}%".format(yaw*100)
            elif throttle:
                text_control_state = "Z : {:0.0f}%".format(throttle*100)
        else: # skip motion
            yaw = 0
            throttle = 0

        if text_control_state:
            screen.blit(self.my_font.render(text_control_state, True, COLOR_BLUE), (5, 5))

        text_vehicle_state = f"Armed : {self.vehicle_state.armed}, Mode : {self.vehicle_state.mode}"
        screen.blit(self.my_font.render(text_vehicle_state, True, COLOR_WHITE), (5, SCREEN_HEIGHT - 20))

        pygame.display.update()

        if (rospy.Time.now() - self.last_control_pub_time).to_sec() > 0.05:
            self.last_control_pub_time = rospy.Time.now()
            self.control_msg.send_control(roll, pitch, yaw, throttle, not self.waypoint_mode_enabled)

        self.stick_1.release_H()
        if not self.waypoint_mode_enabled:
            self.stick_1.release_V()
            self.stick_2.release()


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)

    rospy.init_node("rc_demo_node", anonymous=True)

    pygame.init()
    pygame.display.set_caption("Simple PyGame Example")
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    pygame.font.init()

    controller = Controller()
    clock = pygame.time.Clock()
    while not controller.shoud_exit and not rospy.is_shutdown():
        clock.tick(60)
        controller.control_callback(1)
