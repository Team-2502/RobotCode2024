ssh lvuser@roboRIO-2502-frc.local /usr/local/frc/bin/frcKillRobot.sh

scp ./target/arm-unknown-linux-gnueabi/release/libRobotCode2024.so scp://admin@10.25.2.2:22//home/lvuser/deploy/libRobotCode2024.so

ssh lvuser@roboRIO-2502-frc.local /usr/local/frc/bin/frcRunRobot.sh