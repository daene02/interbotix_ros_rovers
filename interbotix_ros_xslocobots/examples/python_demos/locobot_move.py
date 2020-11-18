from interbotix_xs_modules.locobot import InterbotixLocobotXS

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_wx200", arm_model="mobile_wx200")
    # success = locobot.base.move_to_pose(-1, -4.5, 0)

if __name__=='__main__':
    main()
